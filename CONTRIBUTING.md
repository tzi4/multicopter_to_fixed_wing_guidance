# Contributing

Thank you for helping improve `savasan_iha_yildizlar`. The project is currently
maintained privately and will accept public contributions after its public
release.

## Before opening a change

1. Use an issue to describe large behavioural or architecture changes.
2. Keep flight-control changes separate from formatting or generated data.
3. Never commit real GPS coordinates, raw telemetry, faces, credentials,
   vehicle identifiers or private test-site information.
4. Do not weaken pre-arm, geofence, altitude, manual-override or shutdown
   safeguards to make a test pass.
5. State whether a result came from simulation, bench testing or real flight.

## Development check

Install the verified Python set and run the offline suite:

```bash
python3 -m pip install -r requirements-lock.txt
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
bash -n yildizlar_gudum.sh tools/senaryo.sh
```

Simulation results should include the route, vehicle model, configuration,
random/initial condition, true CPA, contact signature, aborts, wall-clock to
simulation-time ratio and the relevant log bundle.

## Pull requests

- Explain the problem and the chosen solution.
- Include exact reproduction and validation commands.
- Add or update tests for behaviour changes.
- Update documentation and third-party notices when required.
- Confirm that the public-release check passes.

By contributing original work, you agree that it may be distributed under the
project's MIT License. Third-party material must keep its original license and
attribution.
