# Public release procedure

This repository is prepared as a sanitized source snapshot. Raw real-flight
telemetry, GPS-bearing CSV files, local camera captures, generated SITL state
and historical source archives are deliberately excluded from its Git history.
The full development/evidence history is retained separately in private
storage and must remain private.

## Release gate

Run from the repository root on a clean `main` checkout:

```bash
git status --short
python3 tools/public_release_check.py
python3 donanim/test_balon_menzil.py

(
  cd guidance_allstar
  python3 terminal_los_test.py
  python3 mpc_test.py
  python3 los_test.py
  python3 pid_test.py
)

python3 -m compileall -q \
  bbox_to_redis.py donanim guidance_allstar tools yarışma
bash -n yildizlar_gudum.sh tools/senaryo.sh guidance_allstar/gudum_chase.sh
```

The expected offline counts are 15/15 Terminal LOS/PN, 88/88 MPC, 66/66 LOS,
51/51 PID and 4/4 balloon/range tests. Stop if any count or release check
differs.

## Human review

- Confirm that `LICENSE`, `NOTICE` and `THIRD_PARTY_NOTICES.md` still describe
  every bundled model and asset.
- Confirm that no real GPS coordinates, credentials, vehicle identifiers,
  faces or private test-site information were added after this preparation.
- Confirm that new evidence follows `DATA_POLICY.md`.
- Review the README's results and clearly distinguish simulation, bench and
  real-flight claims.
- Check the latest dependency/security alerts and resolve applicable findings.
- Verify geofence, manual override, minimum-altitude, HOLD/BRAKE and shutdown
  safeguards have not been weakened.
- Create a signed version tag for the first public release.

## GitHub visibility change

The repository must remain `PRIVATE` until the owner intentionally releases it.
After every gate above passes, change visibility in GitHub repository settings
or with:

```bash
gh repo edit tzi4/savasan_iha_yildizlar \
  --visibility public \
  --accept-visibility-change-consequences
```

After publication, verify the README, license detection, Actions run, Security
policy, issue templates and clone instructions while signed out of GitHub.

Do not attach or publish the private history archive as a branch, release asset
or pull request of the public repository.
