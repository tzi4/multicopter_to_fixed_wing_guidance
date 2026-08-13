# Guidance and Estimation Code for the AllStar Competition

This repo has multiple location-based guidance and target-following methods.

## pronavwndr2.py
Old pronav code that uses APN/TPN/PPN. Last issue: Stability.

## frpnwndr.py
Newer pronav code that uses FRPN. Last issue: Stability.

## simple_guided_follow.py
Current practical follower. It uses `filterwndr.py` to estimate and predict target state, then sends a moving slot position/velocity target to ArduCopter GUIDED mode.

## terminal_los_gudum.py
Camera-only terminal PN/LOS collision guidance. Target telemetry is not used
for guidance (only the measured range channel); commands are acceleration and
direction reachable from the current vehicle velocity. The `YERLES/VUR/DON`
state machine releases authority after a miss instead of attempting an unsafe
head-on second pass.

## hibrit_gudum.py
Current research candidate. Position guidance first establishes the rear slot;
after visual handover, MPC remains the mid-course FOV/constraint planner above
18 m and terminal PN/LOS takes over at or below 18 m. Both branches use the
same reachability projection and vertical safety limits.

```bash
# Unit/contract checks
python3 guidance_allstar/terminal_los_test.py

# Fresh-stack ellipse experiment (20 s estimator settling is intentional)
SURE=360 KONTROL_BEKLE_S=20 GORUNTULU="hibrit_gudum.py" \
  PLAN=missions/hedef_elips.plan METOT=hibrit_elips tools/senaryo.sh

# One factorial cell, without editing source
SURE=360 KONTROL_BEKLE_S=20 \
  GORUNTULU="hibrit_gudum.py --n-pn 5 --gecis-menzil 20 --tirmanma-hiz-max 2.0" \
  PLAN=missions/hedef_elips.plan METOT=h_n5_r20_vz2 tools/senaryo.sh
```

Live A/B results and the next factorial test matrix are recorded in
`../TO_TEST.md` under **CANLI T1 HÜKMÜ**.

## filterwndr.py
Position-only target estimator. It now uses a six-mode product IMM:

```text
{CVxy, CTxy, CAxy} x {CVz, CAz}
```

This lets horizontal turns and vertical motion be modeled independently, e.g. `CTxy_CVz` for an upward/downward helix and `CTxy_CAz` for a turn with height loss/recovery. See `filterwndr_estimator.md` for the detailed estimator reference.
