# Third-party components

The physical motor implementation and Gazebo transport protobuf definitions in
`upstream/` come from PX4's `PX4-SITL_gazebo-classic` repository at commit
`807b67bb3007113ea52fd36ec2cefae4a5fa3f85`.

The Hummingbird model under `../../models_hummingbird/hummingbird/` was
generated from ETH Zurich ASL's `rotors_simulator` repository at commit
`cd813b7a8c375d677352aa20ad20047feb661126`, using
`rotors_description/urdf/hummingbird_base.xacro`. Its Hummingbird and propeller
meshes are copied from the same repository. Mesh URIs were made local so the
model remains self-contained. The upstream RotorS controller, IMU and motor
plugin tags were removed from the inner model because the outer `suru_drone_N`
wrapper supplies ArduPilot's IMU contract and the same RotorS motor constants
through the vendored Gazebo Classic motor plugin. Physical links, joints,
inertias, collisions and meshes are unchanged.

Both upstream projects publish these files under the Apache License 2.0. See
`LICENSE-Apache-2.0` and the copyright/license headers in the source files.

The RoboFly physical model under `../../models_robofly/robofly/` was rendered
from CTU-MRS's Gazebo Classic `robofly.sdf.jinja` at commit
`020cdcb0268fb1f98c128181355f8d4a827798a9`. Body/propeller mass and inertia,
arm geometry, collisions, meshes, body resistance, and all motor constants are
the upstream values. PX4/MRS flight-interface and sensor plugins were removed
from the inner model; the outer `suru_drone_N` wrapper supplies the existing
ArduPilot IMU contract, the project pitch gimbal, and the same motor physics
through the vendored Gazebo motor plugin.

`upstream/fluid_resistance_plugin.cpp` is the official CTU-MRS body-resistance
plugin from `mrs_gazebo_common_resources` commit
`5ee221fb438515d428abb35f896176b6ef14a874`. It is built with its upstream
library name so the rendered RoboFly model can use it without changing its
plugin contract. CTU-MRS publishes both repositories under Apache License 2.0.

Upstream repositories:

- https://github.com/PX4/PX4-SITL_gazebo-classic
- https://github.com/ethz-asl/rotors_simulator
- https://github.com/ctu-mrs/mrs_uav_gazebo_simulator
- https://github.com/ctu-mrs/mrs_gazebo_common_resources
