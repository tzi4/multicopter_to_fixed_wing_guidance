# Sanitized result summaries

This directory contains small, non-sensitive tables used to reproduce the
aggregate claims in the main README. Raw real-flight telemetry, GPS data and
site imagery are intentionally not distributed.

`terminal_los_n4_20260811.csv` contains the six true-CPA measurements from the
RoboFly/elips Terminal LOS/PN `N=4`, `%3` area, five-large-frame campaign.
Contact rows cannot be mapped reliably from this aggregate handoff table, so
the file does not invent a per-row contact label. The campaign-level result
was 6/6 below 5 m and 2/6 Gazebo contact signatures.
