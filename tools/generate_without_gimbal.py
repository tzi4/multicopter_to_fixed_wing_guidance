#!/usr/bin/env python3
"""It produces the NON-GIMBLE variant of the selected copter model.

WHY IT HAS: The gimbal branch connected the camera to the tilt joint of the gimbal_small_2d. When
the gimbal suspects a malfunction (e.g. a physics instability preventing the ARM) or A/B is required
with the old behavior, it is necessary to be able to revert to an arm WITHOUT GIMBAL. This script
DOES NOT TOUCH THE ORIGINAL SDF's: it produces a separate tree.

HOW TO CHOOSE: yildizlar_guidance.sh --without-gimbal (or YILDIZ_GIMBAL=0) models_fixed_camera for Launcher Iris,
models_hummingbird_fixed_camera for Hummingbird, models_robofly_fixed_camera for RoboFly puts it at the beginning
of GAZEBO_MODEL_PATH. The world file does NOT change.

Three transformations: 1) <inlude> model://gimbal_small_2d block deleted 2) <joint
name="gimbal_mount"> deleted 3) parent of camera_mount gimbal_1::tilt_link -> selected body becomes
base_link (i.e. camera still FIXED to the body, pre-gimbal behavior)

USAGE:
    python3 tools/generate_without_gimbal.py                         # Iris
    python3 tools/generate_without_gimbal.py --vehicle-value hummingbird      # Hummingbird
    python3 tools/generate_without_gimbal.py --vehicle-value robofly          # RoboFly
    python3 tools/generate_without_gimbal.py --vehicle-value iris --control   # control without typing
"""

import argparse
import os
import re
import shutil
import sys

ROOT_VALUE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
def convert(raw_value: str, source_name: str, body_link: str) -> str:
    heading_item = (f'<!-- OTOMATIK URETILDI: {source_name}/swarm_drone_N/model.sdf\'in '
              'WITHOUT_GIMBAL\n'
              '     (body-fixed camera) variant. Manual editing; source '
              'degisince\n'
              '     Reproduced with tools/generate_without_gimbal.py. -->\n')
    raw_value = re.sub(
        r'\n\s*<include>\s*\n\s*<uri>model://gimbal_small_2d</uri>.*?</include>\n',
        '\n', raw_value, flags=re.S)
    raw_value = re.sub(r'\n\s*<joint name="gimbal_mount".*?</joint>\n',
                 '\n', raw_value, flags=re.S)
    raw_value = raw_value.replace('<parent>gimbal_1::tilt_link</parent>',
                      f'<parent>{body_link}</parent>')
    # The XML declaration must be the absolute first line of the file. The village after reporting the
    # production information; otherwise standard XML validators will reject the file even if Gazebo
    # tolerates it.
    if raw_value.startswith('<?xml'):
        initial_row_end = raw_value.find('\n') + 1
        return raw_value[:initial_row_end] + heading_item + raw_value[initial_row_end:]
    return heading_item + raw_value


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--vehicle-value', choices=('iris', 'hummingbird', 'robofly'), default='iris',
                   help='body model (default: iris)')
    p.add_argument('--control', action='store_true',
                   help='just check the update, DO NOT WRITE')
    a = p.parse_args()

    if a.vehicle_value == 'hummingbird':
        source_directory = os.path.join(ROOT_VALUE, 'models_hummingbird')
        target_directory = os.path.join(ROOT_VALUE, 'models_hummingbird_fixed_camera')
        body_link = 'hummingbird::hummingbird/base_link'
        source_name = 'models_hummingbird'
    elif a.vehicle_value == 'robofly':
        source_directory = os.path.join(ROOT_VALUE, 'models_robofly')
        target_directory = os.path.join(ROOT_VALUE, 'models_robofly_fixed_camera')
        body_link = 'robofly::base_link'
        source_name = 'models_robofly'
    else:
        source_directory = os.path.join(ROOT_VALUE, 'models')
        target_directory = os.path.join(ROOT_VALUE, 'models_fixed_camera')
        body_link = 'iris::base_link'
        source_name = 'models'

    stale_value = []
    for label_item in sorted(os.listdir(source_directory)):
        source_value = os.path.join(source_directory, label_item, 'model.sdf')
        if not label_item.startswith('swarm_drone_') or not os.path.isfile(source_value):
            continue
        target_dir = os.path.join(target_directory, label_item)
        target_value = os.path.join(target_dir, 'model.sdf')
        new_value = convert(open(source_value).read(), source_name, body_link)
        previous = open(target_value).read() if os.path.isfile(target_value) else None
        if previous == new_value:
            print(f'  current: {label_item}')
            continue
        stale_value.append(label_item)
        if a.control:
            print(f'  STALE   : {label_item}')
            continue
        os.makedirs(target_dir, exist_ok=True)
        cfg = os.path.join(source_directory, label_item, 'model.config')
        if os.path.isfile(cfg):
            shutil.copy(cfg, target_dir)
        open(target_value, 'w').write(new_value)
        # There should be no STRUCTURAL gimbal reference left in the generated file (comments are allowed).
        for pattern_value in ('<uri>model://gimbal_small_2d',
                      '<joint name="gimbal_mount"',
                      '<parent>gimbal_1::'):
            if pattern_value in new_value:
                raise SystemExit(f'ERROR: {pattern_value!r} still exists in {label_item}')
        print(f'  URETILDI: {label_item}')

    if a.control and stale_value:
        print(f'\n{len(stale_value)} model STALE -> python3 tools/generate_without_gimbal.py')
        return 1
    print(f'\n{os.path.basename(target_directory)}/ ready. '
          'Usage: ./yildizlar_guidance.sh --without-gimbal')
    return 0


if __name__ == '__main__':
    sys.exit(main())
