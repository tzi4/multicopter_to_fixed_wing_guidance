# Third-party notices

The MIT license in [`LICENSE`](LICENSE) applies to original
`savasan_iha_yildizlar` code and documentation. It does not replace licenses
attached to bundled third-party source code, models, meshes or other assets.

## PX4 Gazebo Classic / RotorS components

Parts of the Iris, fixed-wing and Hummingbird simulation models, together with
supporting Gazebo plugin code, originate from:

- [PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic)
- [RotorS](https://github.com/ethz-asl/rotors_simulator)

Copyright notices and Apache License 2.0 headers belonging to the upstream
authors are retained in the corresponding files. The consolidated bridge
provenance and a copy of Apache-2.0 are available in
[`plugins/hummingbird_bridge/THIRD_PARTY.md`](plugins/hummingbird_bridge/THIRD_PARTY.md)
and
[`plugins/hummingbird_bridge/LICENSE-Apache-2.0`](plugins/hummingbird_bridge/LICENSE-Apache-2.0).

## CTU-MRS RoboFly components

RoboFly model material is derived from the CTU-MRS Gazebo simulation projects:

- [mrs_uav_gazebo_simulator](https://github.com/ctu-mrs/mrs_uav_gazebo_simulator)
- [mrs_gazebo_common_resources](https://github.com/ctu-mrs/mrs_gazebo_common_resources)

These upstream repositories are distributed under the BSD 3-Clause License.
Their author attributions are retained in the RoboFly model metadata.

## Gimbal Small 2D mesh

The `gimbal_kurulum/gimbal_small_2d` model references **MotorPixie 2D gimbal
for Phantom 2 Vision**, created by Motorpixiegimbals:

- Source: <https://www.thingiverse.com/thing:397579>
- License: [Creative Commons Attribution-ShareAlike 4.0 International](https://creativecommons.org/licenses/by-sa/4.0/)

The asset is used as part of a Gazebo model and may have been converted or
adapted for simulation. Redistribution and modifications of this asset remain
under CC BY-SA 4.0; attribution and ShareAlike terms must be preserved.

The accompanying Gazebo gimbal plugin source contains its own Open Source
Robotics Foundation copyright and Apache-2.0 license headers.

## External runtime dependencies

ArduPilot, Gazebo, ROS, MAVProxy, pymavlink, Redis, NumPy, OpenCV, Matplotlib,
Picamera2 and Raspberry Pi IMX500 packages are external dependencies and are
not relicensed by this repository. Consult each upstream project for its exact
license and distribution terms.

## Reporting an attribution issue

If an attribution or license record is incomplete, please report it privately
using the process in [`SECURITY.md`](SECURITY.md) before redistributing the
affected asset.
