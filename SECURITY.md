# Security policy

## Supported version

Until the first public release, only the latest commit on `main` is supported.
After release, supported tags will be listed here.

## Reporting a vulnerability

Do not open a public issue for vulnerabilities that could affect a physical
vehicle, expose credentials or reveal private flight/test-site data. Use
GitHub's private vulnerability reporting feature on the repository Security
page. If that feature is unavailable, contact the repository owner privately
through the GitHub profile.

Include the affected commit, environment, reproduction conditions, potential
physical impact and a minimal proof of concept. Remove GPS coordinates,
vehicle identifiers, credentials and images of people from the report.

## Safety scope

This repository is research software, not certified flight-control or
airworthiness software. A software defect can cause loss of control, impact,
property damage or injury. Test changes in this order:

1. offline/unit test;
2. SITL and headless simulation;
3. propeller-free bench test;
4. dry-run command observation;
5. low-energy, geofenced flight with an independent safety pilot.

Never use a person, animal or unprotected property as a target.
