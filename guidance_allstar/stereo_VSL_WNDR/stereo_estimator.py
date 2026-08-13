"""
stereo_estimator.py -- bearings-only target tracker built on the product IMM.

Pipeline per frame:

    detections (angles)
        |
        +-- triangulate (ML in angle space) ---> seed / turn-rate hint / skew
        |                                        (never the primary measurement)
        |
        +-- predict IMM to now  --------------> filterwndr.predict_imm_over_dt
        |
        +-- update IMM on ANGLES -------------> stereo_measurement.imm_bearing_update

Why triangulation still exists at all, given the whole point is to avoid it:
  1. Track seeding needs a position and a covariance, and the GDOP inverse-
     information matrix is exactly the right anisotropic seed.
  2. filterwndr's HeadingTurnRateEstimator wants positions to derive a turn
     rate for the CT mode. Feeding it the INDEPENDENT per-frame fix keeps that
     hint non-circular (feeding it the filter's own output would not).
  3. Skew and conditioning are per-frame health signals worth logging.

State machine: INIT -> TRACK -> (COAST) -> TRACK, or -> LOST -> INIT.
COAST means the filter propagates with no measurement -- normal during short
dropouts, and the IMM's growing covariance is the honest report of that.
"""

import time

import numpy as np

import filterwndr as fw
import stereo_config as scfg
import stereo_geometry as sg
import stereo_measurement as sm

_EPS = 1e-12


class TrackState:
    INIT = "INIT"
    TRACK = "TRACK"
    COAST = "COAST"
    LOST = "LOST"


class StereoTracker:
    """Bearings-only IMM tracker for a stereo (or single) camera rig."""

    def __init__(self, cameras, cfg=scfg, nominal_dt=0.05):
        self.cameras = list(cameras)
        self.cfg = cfg
        self.nominal_dt = float(nominal_dt)

        self.imm = fw.setup_imm_filter(self.nominal_dt)
        self.turn_rate_estimator = fw.HeadingTurnRateEstimator()
        self.boresight = sm.BoresightEstimator(
            len(self.cameras),
            tau_s=float(getattr(cfg, "BORESIGHT_BIAS_TAU_S", 30.0)),
            max_bias_rad=float(np.radians(getattr(cfg, "BORESIGHT_BIAS_MAX_DEG", 1.0))),
            enabled=bool(getattr(cfg, "ESTIMATE_BORESIGHT_BIAS", False)),
        )
        # Whatever boresight was entered on the cameras is the starting point
        # the online estimator refines, not something it overwrites.
        self.boresight.set_base(self.cameras)

        self.state = TrackState.INIT
        self.last_stamp = None
        self.last_good_stamp = None
        self._last_hint_stamp = None
        self._last_seen_stamp = None   # newest stamp accepted, any state
        self._init_fixes = []          # [(stamp, xyz, P3)] while seeding
        self.frames = 0
        self.updates = 0
        self.coast_frames = 0
        self.reinits = 0
        self.stale_frames = 0          # consecutive non-advancing stamps
        self.rejected_frames = 0
        self.rejected_dets = 0
        self.last_diag = {}

    # ------------------------------------------------------------------
    #  Per-frame geometry
    # ------------------------------------------------------------------
    def _frame_geometry(self, detections):
        """Triangulate when two cameras see the target. -> dict or None."""
        if len(detections) < 2:
            return None
        d0, d1 = detections[0], detections[1]
        c0, c1 = self.cameras[d0.cam_index], self.cameras[d1.cam_index]
        seed, info = sg.triangulate_midpoint(c0, d0, c1, d1)
        if not np.all(np.isfinite(seed)):
            return None
        fix, ml_info = sg.triangulate_ml(
            self.cameras, detections, seed,
            iters=int(getattr(self.cfg, "TRIANGULATION_GN_ITERS", 6)),
        )
        if not np.all(np.isfinite(fix)):
            fix, ml_info = seed, {"converged": False, "cost": float("nan")}
        cov, cov_info = sg.triangulation_covariance(self.cameras, detections, fix)
        # Residuals AT the ML fix: the component of the measurement no position
        # can explain. This -- not the filter innovation -- is what carries
        # boresight misalignment (see BoresightEstimator).
        resid = {}
        for det in detections:
            cam = self.cameras[det.cam_index]
            _, pred = cam.bearing_jacobian(fix)
            # det is already de-biased (see _debias), so this is the residual
            # that REMAINS after the entered correction -- which is exactly
            # what BoresightEstimator should accumulate on top of it.
            resid[(det.cam_index, 0)] = float(sg.wrap_pi(det.yaw - pred[0]))
            resid[(det.cam_index, 1)] = float(sg.wrap_pi(det.pitch - pred[1]))
        out = dict(info)
        out["ml_residuals"] = resid
        out.update(
            {
                "fix": fix, "seed": seed, "cov": cov,
                "cond": cov_info["cond"], "sigma_max_m": cov_info["sigma_max_m"],
                "rank_deficient": cov_info["rank_deficient"],
                "ml_cost": ml_info.get("cost", float("nan")),
            }
        )
        return out

    # ------------------------------------------------------------------
    #  Initialisation
    # ------------------------------------------------------------------
    def _try_init(self, geom, stamp):
        """Accumulate clean dual-camera fixes, then seed the IMM."""
        if geom is None:
            return False
        max_skew = float(getattr(self.cfg, "INIT_MAX_SKEW_M", 15.0))
        min_par = float(np.radians(getattr(self.cfg, "TRIANGULATION_MIN_PARALLAX_DEG", 0.05)))
        if (
            geom["skew_m"] > max_skew
            or geom["behind"]
            or geom["parallel"]
            or geom["parallax_rad"] < min_par
            or geom["rank_deficient"]
        ):
            self._init_fixes.clear()   # require CONSECUTIVE clean frames
            return False

        self._init_fixes.append((stamp, np.asarray(geom["fix"], dtype=float), geom["cov"]))
        need = int(getattr(self.cfg, "INIT_MIN_FRAMES", 4))
        if len(self._init_fixes) < need:
            return False

        t0, x0, P0 = self._init_fixes[0]
        t1, x1, P1 = self._init_fixes[-1]
        span = max(t1 - t0, 1e-3)

        # Velocity seed: finite difference of the first and last fix, with the
        # covariance propagated rather than guessed. This matters -- an
        # isotropic velocity sigma would be wildly wrong, since the along-LOS
        # component of a differenced fix is ~100x noisier than the cross-LOS
        # component. (P0+P1)/span^2 carries that anisotropy through.
        vel = (x1 - x0) / span
        vmax = 80.0
        vnorm = float(np.linalg.norm(vel))
        if vnorm > vmax:
            vel = vel * (vmax / vnorm)
        P_vel = (P0 + P1) / (span * span)

        inflate = float(getattr(self.cfg, "INIT_POS_SIGMA_INFLATE", 2.0)) ** 2
        P_pos = np.asarray(P1, dtype=float) * inflate
        vel_cap = float(getattr(self.cfg, "INIT_VEL_SIGMA_MPS", 12.0))
        # Floor the velocity block so a suspiciously tight seed cannot lock the
        # filter onto a wrong initial velocity, and cap it so it stays sane.
        P_vel = P_vel + np.eye(3) * (vel_cap ** 2)
        acc_sig = float(getattr(self.cfg, "INIT_ACC_SIGMA_MPS2", 5.0))

        for kf in self.imm.filters:
            n = kf.x.shape[0]
            x = np.zeros(n)
            x[0:3] = x1
            x[3:6] = vel
            P = np.array(kf.P, dtype=float, copy=True)
            P[0:3, 0:3] = P_pos
            P[3:6, 3:6] = P_vel
            P[6:9, 6:9] = np.eye(3) * (acc_sig ** 2)
            # cross-blocks between position/velocity/accel start uncorrelated
            P[0:3, 3:] = 0.0; P[3:, 0:3] = 0.0
            P[3:6, 6:] = 0.0; P[6:, 3:6] = 0.0
            # leave P[9,9] (omega) at the per-mode value setup_imm_filter chose
            kf.x = x
            kf.P = 0.5 * (P + P.T)

        fw.stabilize_omega_states(self.imm)
        self.imm._compute_state_estimate()
        self.state = TrackState.TRACK
        self.last_stamp = stamp
        self.last_good_stamp = stamp
        self._init_fixes.clear()
        return True

    # ------------------------------------------------------------------
    #  Main entry point
    # ------------------------------------------------------------------
    @staticmethod
    def _finite_det(d):
        return (np.isfinite(d.yaw) and np.isfinite(d.pitch)
                and np.isfinite(d.stamp))

    def _debias(self, dets):
        """Subtract each camera's boresight offset ONCE, at the boundary.

        Everything downstream -- triangulation, skew, the init seed, the
        covariance and the angular update -- then works in the same corrected
        frame. Returns new Detection objects; the caller's are not mutated,
        since a caller may legitimately reuse or re-send them.
        """
        out = []
        for d in dets:
            cam = self.cameras[d.cam_index]
            if cam.bias_yaw == 0.0 and cam.bias_pitch == 0.0:
                out.append(d)
                continue
            out.append(sg.Detection(d.cam_index, d.yaw - cam.bias_yaw,
                                    d.pitch - cam.bias_pitch, d.stamp,
                                    valid=d.valid, meta=d.meta))
        return out

    def process(self, detections, stamp):
        """Consume one frame of detections. Returns a snapshot dict."""
        self.frames += 1

        # --- input validation ----------------------------------------------
        # Order matters: reject before any geometry runs, so a NaN never
        # reaches the trig and never becomes a NaN "fix".
        if not np.isfinite(stamp):
            self.rejected_frames += 1
            return self._snapshot(self.last_stamp, None,
                                  {"error": "non-finite-stamp", "updated": False})

        dets = [d for d in detections if d is not None and d.valid]
        if bool(getattr(self.cfg, "REJECT_NONFINITE_DETECTIONS", True)):
            kept = [d for d in dets if self._finite_det(d)]
            self.rejected_dets += len(dets) - len(kept)
            dets = kept

        # A stamp that does not advance is either a duplicate or out of order.
        # Either way it carries no new information, and applying it corrupts
        # both the covariance and (by rewinding last_stamp) the next real dt.
        if (bool(getattr(self.cfg, "REJECT_NONMONOTONIC_STAMPS", True))
                and self._last_seen_stamp is not None
                and stamp <= self._last_seen_stamp):
            self.stale_frames += 1
            self.rejected_frames += 1
            limit = int(getattr(self.cfg, "STALE_FRAME_LIMIT", 0))
            if (limit > 0 and self.stale_frames >= limit
                    and self.state in (TrackState.TRACK, TrackState.COAST)):
                # The sender's clock has stopped. Measurement-time timeouts can
                # never fire in that state, so this is the only thing that will.
                self.state = TrackState.LOST
                self.reinits += 1
                self._init_fixes.clear()
            return self._snapshot(self._last_seen_stamp, None,
                                  {"error": "stale-stamp", "updated": False})
        self._last_seen_stamp = stamp
        self.stale_frames = 0

        dets = self._debias(dets)
        geom = self._frame_geometry(dets)

        # Turn-rate hint from the independent per-frame fix (non-circular).
        #
        # Gated on both fix quality and elapsed time: omega is a differentiated
        # quantity, so a fast stream of noisy fixes turns into a large fake turn
        # rate and drives the CT modes into divergence (see TURN_HINT_MIN_DT_S).
        omega_hint = None
        if geom is not None and getattr(self.cfg, "USE_TRIANGULATION_TURN_HINT", True):
            min_dt = float(getattr(self.cfg, "TURN_HINT_MIN_DT_S", 0.0))
            max_skew = float(getattr(self.cfg, "TURN_HINT_MAX_SKEW_M", float("inf")))
            fresh = (self._last_hint_stamp is None
                     or (stamp - self._last_hint_stamp) >= min_dt)
            clean = (float(geom.get("skew_m", 0.0)) <= max_skew
                     and not geom.get("rank_deficient", False))
            if fresh and clean:
                omega_hint = self.turn_rate_estimator.update(geom["fix"], stamp)
                self._last_hint_stamp = stamp

        if self.state in (TrackState.INIT, TrackState.LOST):
            # Stay in LOST (rather than flipping straight back to INIT) until a
            # re-acquisition actually succeeds -- a consumer polling state must
            # be able to tell "never had a track" from "lost the one I had".
            # _try_init() promotes to TRACK only on success.
            self._try_init(geom, stamp)
            return self._snapshot(stamp, geom, None)

        # --- propagate -----------------------------------------------------
        dt_raw = stamp - (self.last_stamp if self.last_stamp is not None else stamp)
        # A gap longer than the coast timeout is not a frame to propagate
        # through -- it is a lost track. Clamping it instead would propagate
        # for less time than elapsed and silently corrupt the velocity.
        gap_limit = float(getattr(self.cfg, "COAST_TIMEOUT_S", 2.0))
        if dt_raw > gap_limit:
            self.state = TrackState.LOST
            self.reinits += 1
            self._init_fixes.clear()
            self.last_stamp = stamp
            return self._snapshot(stamp, geom,
                                  {"error": "gap-exceeds-coast-timeout",
                                   "gap_s": float(dt_raw), "updated": False})
        dt = float(np.clip(dt_raw,
                           float(getattr(self.cfg, "MIN_PREDICT_DT_S", 1e-3)),
                           float(getattr(self.cfg, "MAX_PREDICT_DT_S", 2.0))))
        if omega_hint is not None:
            fw.apply_fast_turn_onset_hint(
                self.imm, self.turn_rate_estimator.raw_omega,
                self.turn_rate_estimator.speed_xy,
            )
            fw.apply_turn_rate_hint(self.imm, omega_hint)
        try:
            fw.predict_imm_over_dt(
                self.imm, dt,
                max_substep=float(getattr(self.cfg, "PREDICT_SUBSTEP_S", 0.1)),
            )
        except np.linalg.LinAlgError:
            # UKF sigma-point Cholesky failure: the track is unrecoverable.
            self.state = TrackState.LOST
            self.reinits += 1
            return self._snapshot(stamp, geom, {"error": "predict-LinAlgError"})
        self.last_stamp = stamp

        # --- frame quality -> angular noise scaling -------------------------
        noise_scale = 1.0
        frame_dropped = False
        if geom is not None:
            noise_scale = sm.skew_noise_scale(geom["skew_m"], self.cfg)
            if not np.isfinite(noise_scale):
                frame_dropped = True

        # --- update on angles ----------------------------------------------
        mdiag = None
        if dets and not frame_dropped:
            try:
                mdiag = sm.imm_bearing_update(
                    self.imm, self.cameras, dets,
                    frame_noise_scale=noise_scale, cfg=self.cfg, filterwndr=fw,
                )
            except np.linalg.LinAlgError:
                mdiag = {"error": "update-LinAlgError", "updated": False}
            if mdiag.get("updated"):
                self.updates += 1
                self.last_good_stamp = stamp
                self.coast_frames = 0
                self.state = TrackState.TRACK
                # Persistent residual -> boresight misalignment, not noise.
                if geom is not None:
                    self.boresight.observe(geom.get("ml_residuals", {}), dt)
                    self.boresight.apply(self.cameras)
                if omega_hint is not None:
                    fw.apply_turn_rate_hint(self.imm, omega_hint)
                    fw.stabilize_omega_states(self.imm)
            else:
                self.coast_frames += 1
        else:
            self.coast_frames += 1

        # --- coast / loss ---------------------------------------------------
        if self.last_good_stamp is not None:
            age = stamp - self.last_good_stamp
            timeout = float(getattr(self.cfg, "COAST_TIMEOUT_S", 2.0))
            if age > timeout:
                self.state = TrackState.LOST
                self.reinits += 1
                self._init_fixes.clear()
            elif age > 1e-6 and self.state == TrackState.TRACK and self.coast_frames > 0:
                self.state = TrackState.COAST

        return self._snapshot(stamp, geom, mdiag)

    # ------------------------------------------------------------------
    #  Reporting
    # ------------------------------------------------------------------
    def _snapshot(self, stamp, geom, mdiag):
        tracking = self.state in (TrackState.TRACK, TrackState.COAST)
        x = np.asarray(self.imm.x, dtype=float).reshape(-1).copy() if tracking else None
        P = np.asarray(self.imm.P, dtype=float).copy() if tracking else None

        snap = {
            "stamp": stamp,
            "state": self.state,
            "tracking": tracking,
            "x": x,
            "P": P,
            "position": None if x is None else x[0:3].copy(),
            "velocity": None if x is None else x[3:6].copy(),
            "mode_probs": fw.aggregate_mode_probabilities(self.imm) if tracking else None,
            "geom": geom,
            "meas": mdiag,
            "coast_frames": self.coast_frames,
            "stale_frames": self.stale_frames,
            "rejected_frames": self.rejected_frames,
            "rejected_dets": self.rejected_dets,
        }
        if P is not None:
            pos_var = np.clip(np.diag(P)[0:3], 0.0, None)
            snap["sigma_pos_m"] = float(np.sqrt(np.sum(pos_var)))
            # Split the reported uncertainty the way the sensor actually fails:
            # along the line of sight vs across it.
            ref = self.cameras[0].position
            los = x[0:3] - ref
            n = np.linalg.norm(los)
            if n > _EPS:
                u = los / n
                snap["range_from_rig_m"] = float(n)
                snap["sigma_along_los_m"] = float(np.sqrt(max(u @ P[0:3, 0:3] @ u, 0.0)))
                perp = np.eye(3) - np.outer(u, u)
                Pc = perp @ P[0:3, 0:3] @ perp.T
                snap["sigma_cross_los_m"] = float(np.sqrt(max(np.trace(Pc) / 2.0, 0.0)))
        self.last_diag = snap
        return snap

    def reset(self):
        self.imm = fw.setup_imm_filter(self.nominal_dt)
        self.turn_rate_estimator = fw.HeadingTurnRateEstimator()
        self.state = TrackState.INIT
        self.last_stamp = None
        self.last_good_stamp = None
        self._last_hint_stamp = None
        self._last_seen_stamp = None
        self._init_fixes.clear()
        self.coast_frames = 0
        self.stale_frames = 0


# ----------------------------------------------------------------------
#  Per-frame triangulation-only baseline (for comparison in the tester)
# ----------------------------------------------------------------------
class TriangulationOnlyTracker:
    """No filtering: ML triangulate each frame. The thing to beat.

    Kept so the tester can quantify what the IMM actually buys -- particularly
    in depth, where accumulating angles over a moving baseline should crush a
    per-frame fix.
    """

    def __init__(self, cameras, cfg=scfg):
        self.cameras = list(cameras)
        self.cfg = cfg
        self.last_fix = None

    def process(self, detections, stamp):
        dets = [d for d in detections if d is not None and d.valid]
        if len(dets) < 2:
            return {"stamp": stamp, "position": self.last_fix, "tracking": self.last_fix is not None}
        d0, d1 = dets[0], dets[1]
        seed, _ = sg.triangulate_midpoint(self.cameras[d0.cam_index], d0,
                                          self.cameras[d1.cam_index], d1)
        fix, _ = sg.triangulate_ml(self.cameras, dets, seed,
                                   iters=int(getattr(self.cfg, "TRIANGULATION_GN_ITERS", 6)))
        if np.all(np.isfinite(fix)):
            self.last_fix = fix
        return {"stamp": stamp, "position": self.last_fix, "tracking": self.last_fix is not None}
