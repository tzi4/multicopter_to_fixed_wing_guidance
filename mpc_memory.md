# mpc_memory.md — MPC stroke law: permanent memory (session 2026-08-05)

This file is the "what we learned + why we made which decision" memory. Two files next to it: `TO_TEST.md` (experiment plan + situation table, LIVE) and `FOLLOW_UP.md` (rounds 1..4 campaign log). Log reading: `guidance_allstar/LOG_DICTIONARY.md`. Run the environment: `MPC_NOTES.md`.

---

## 1. ROOT CAUSE (why doesn't it always crash) — three-layer chain

It's not the only mistake; each type closed one ring, the next appeared, **total closure monotonic melt** (e.g. range velocity −3.47 → −0.68 m/s, CIRCUIT1→CIRCUIT4).

1. **CLOSURE CODEED AS BONUS (root).** In the forward speed channel the level penalty is conscious 0 (`mpc_guidance.py:1586` `r_lvl=[0.0,...]`) but the iron `acceleration_run` `‖u−w‖²` is also applied to the forward channel (`:1620`), the only term that asks for closure (linear bbox field reward) Dividing into `/N`=20 (`:2018`). Result: **Valid solution to the cost "three side by side with the target".** Proof: 24-32 Median u1 at m CIRCUIT1 +34.7 → CIRCUIT4 −2.63; 12 12 of WATCH timeout is in this band; 11 is replaced by segment 11, which takes over with bearing angle ≥45°.
2. **Most of the horizon AFTER impact.** Horizon constant 2.4 s, t_go median 0.61 s; Average of the horizon in STRIKE. In the future without %73. Constraint horizon scaled with range (`:1767`), cost horizon did not → 5x asymmetry.
3. **Terminal blindness overlaps by design.** In the last 5 m, 78% of the cycles are blind; four gates in phase (disrupter freeze r<15 m, hard FOV release, blind glide, skeleton hold/coast MPC not called at all ~⅓ cycle).

### Fixed two false beliefs (orchestrator self-check)
- **"FOV %96.8 binding" is FALSE.** `fov_free=0` = "constraint APPLIED", not binding (`:2185`; `LOG_DICTIONARY.md:263` is incorrectly defined). Tuning decisions made with this metric are questionable.
- **Solver's logged instantaneous one-shot redecoding is UNRELIABLE** (hot start not logging; cold solution failed log command 38 m/s). Solver experiments with CLOSED-LOOP replay → `mpc_test.py` `Simulation`/`scenario_run`.

---

## 2. OFFLINE EXPERIMENT INFRASTRUCTURE (item 0 — END)

`class Simulation` + `scenario_run()` in `mpc_test.py` FULL closed-loop: point-mass fighter + real virtual gimbal + target + ONE-TO-ONE actuator chain (LPF τ=0.35 → |v|≤35 clamp → acceleration-limited velocity loop WPNAV_ACCEL 5 m/s²). Speed ceiling `environment_speed_ceiling()`=35. Cycle geometries are available: `tail`(r0=30,β=0), `crossing`(45,40°), `lateral`(55,80°). Metrics: `min_range_value`, `idle_guidance_s`, `miss_reason`, `ceiling_contact_pct`, `pitch_rate_med`, `loss_loop`, `finish`.

**Occlusion GENERATED offline** (accepting item 0): transverse min at baseline 30.7 m (solid)/18.1 m (ellipse), lateral 26.3 m — all WTC timeout; tail closes to 1.6 m. It's exactly like the painting in Sim.

Experiment method: In-memory monkeypatch of `MpcSolver.__init__/solve_value/_step_durations` (WITHOUT WRITE TO SHARE FILE — in another agent sim). ≥8 seed for acceptance: results are bipolar (either ~2 m closes or ~23 m hangs), median misleading at few seeds; "closing rate" (min<10 m %) is the more robust metric.

---

## 3. VERIFIED FIXES (offline; sim waiting)

### Item 1 — enlarge bbox AREA reward (`q_area ×3-4`)
The first hypothesis (exempt u1 from the acceleration bar) is DISRUPTIVE — it alone does not break the occlusion (min 23→13 m, TO %75 constant, frame loss increases). **Correct lever: `q_area`** — drives closing from BBOX AREA, not from range (correct with user philosophy). Knee point ×3-4: blocked closure %25→%75, TO %75→%25, **tail intact**. On ×6 the min range goes back (penetration). Location: `MpcConfig.q_area`, `:2018`.

### Item 3 — scale cost horizon WITH RANGE (NOT t_go)
`mpc_guidance.py:1765` note: t_go scaling has already been tried, it is ineffective in diagonal geometry (closing~0 → t_go∞). Scale by range (`step_s *= clip(r/ref, floor_value, 1)` in ref≈60 m, `_step_durations`). Did NOT fix terminal repeatability; real mechanism: shortening the horizon at close range makes the MPC myopic, highlighting the immediate space reward, disrupting the "three abreast" long-horizon balance. Clogged closure %25→%75, TO %75→%25.

### ★ 1+3 TOGETHER (2×2 factorial, n=32/group) — THEY ARE COLLECTED
| arm | cross closure% | TO% | framing loss% | tail closure% |
|---|---|---|---|---|
| baseline | 25 | 75 | 18.3 | 67 |
| q_area ×4 | 75 | 22 | 19.6 | 67 |
| horizon ref=60 | 75 | 25 | 23.8 | 67 |
| **1+3** | **100** | **0** | **29.3** | 67 |

Reason for ending (main evidence): baseline cross **crash 0/32** (24 "I flew side by side I gave up"); 1+3 → **collision 7/32**. Tail on both arms 16/16 crash (intact), no ground contact (min altitude 47.2 m). **Cost: frame loss end 8→16** — not regression, engagement cost, but NEW BOTTLENECK → item target 5+6.

**OFFLINE→SIM SPACE:** min-range/timeout is reliable (horizontal occlusion is modeled well offline). Frame loss/judder is less reliable (offline engine enters the terminal flat). Verify in sim.

---

## 4. CLEAR VERSION OF THE RULE (user, 2026-08-05)

"HE CAN SEE WITH HIS EYES AND MAKE INFERENCES; ONLY get RANGE from telemetry for now (low confidence)." → **Anything derived from the camera is allowed** (including bearing, bbox size, target kinematics estimation). Prohibited: relying on telemetry/ground detection. The ground detection system will be discussed in a SEPARATE BRANCH. Primary signal bbox area. → Bearing-angle TMA (item 8) REGULAR, not scrapped; but TMA is no longer a requirement, as item 1 solves the closure without it, it is a CEILING RISER.

BEWARE: at one point I interpreted this rule too narrowly and accidentally shelved TMA; user corrected. Ask before narrowing down the rule.

---

## 5. NEXT (see status table TO_TEST.md)

- **SIM QUEUE (after another agent runs out):** `q_area ×4 + horizon ref=60` A/B. Two lines of code, easy to undo. Acceptance: does the cross wall break, does the frame loss/shake not rise above the baseline?
- **CONTINUE OFFLINE:** item 5 (pitch compensation with vertical acceleration: 5.8°/(m/s²)≡1/g, `a_up`+3 → nose 27→21°) + item 6 (0.30 s pitch delay in beta). Both target the new bottleneck (frame loss), pure guidance code, tested offline.
- Then: item 4 (blind terminal PN), item 7 (solver budget), item 2 (geometry gate — COMMON FILE, sim required), item 9/10.

## 6. ENVIRONMENT/PROCEDURE NOTES (corrected in this session)
- Fixed `tools/launch_mission.py` standoff hint: old YILDIZ_MOUNT=30 copy was suggesting `--down 13`; now stems from `standoff_geom.sh` → `--down 4`.
- MPC_NOTES.md (old NOTES.md): path block (everything from repository root) + `[--plan]` bracket warning (optional representation, not part of the command).
- Yaw on ground: 0.00 fixed outside SIMSTATE; the drone does NOT spin on the ground, ±6° normal quad behavior on takeoff. Physically identical to the model legacy `drone_with_camera`.
