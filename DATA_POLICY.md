# Data policy

## Public material

The source repository may contain:

- source code, configuration and simulation models with documented licenses;
- synthetic SITL/Gazebo missions using the public CMAC reference origin;
- small aggregate or sanitized result tables under `examples/results/`;
- screenshots or videos that contain no people, private locations or vehicle
  identifiers and have an explicit redistribution right.

## Private material

The following material must remain outside the public Git repository:

- raw `.tlog`, `.rlog`, `.ulg`, DataFlash and EEPROM files;
- real latitude/longitude, home positions and test-site descriptions;
- unredacted real-flight CSV/JSONL logs;
- faces, voices, license plates, serial numbers and radio identifiers;
- credentials, tokens, private keys and local `.env` files;
- unpublished detector weights or datasets without a redistribution license.
- binary documents or presentations without an explicit redistribution right.

Real-flight evidence is retained in separate private storage. Public claims
must be backed by a sanitized aggregate table or a reproducible simulation
campaign, and must state which evidence class was used.

## Adding a dataset

Before adding data, document its origin, consent/collection basis, license,
redaction method, coordinate treatment, fields, units and checksum. Prefer a
versioned dataset release or an external archival service over committing
large binary data to the source repository.

Run `python3 tools/public_release_check.py` before every release. The checker is
a minimum gate, not a substitute for human privacy and license review.
