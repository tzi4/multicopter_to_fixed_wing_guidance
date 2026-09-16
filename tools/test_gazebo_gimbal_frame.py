#!/usr/bin/env python3
"""Gazebo prevents the gimbal status frame from being mistaken for a joint angle."""

import math
import pathlib
import sys

ROOT_VALUE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_VALUE))

from bbox_to_redis import SwarmRedisDetector
from yildizlar_gimbal import joint_angle


class MockState:
    def __init__(self, eps_deg, age_s_value):
        self.value_rad = math.radians(eps_deg)
        self._age_s = age_s_value

    def age_s_value(self):
        return self._age_s


class MockCommander:
    def __init__(self, target_deg):
        self.target_deg = target_deg


def calculate(eps_status, age_s_value, eps_cmd, roll_deg, pitch_deg):
    detector = SwarmRedisDetector.__new__(SwarmRedisDetector)
    detector.tilt_reader = MockState(eps_status, age_s_value)
    detector.tilt_commander = MockCommander(eps_cmd)
    return detector._joint_calculate(
        math.radians(roll_deg), math.radians(pitch_deg))


def main():
    # The Gazebo plugin broadcasts +9 degrees EARTH camera elevation in stabilized mode. The joint is
    # approximately -11 degrees when the body is at +20 degrees pitch; Counting stats directly as q drops
    # body pitch from the chain.
    q, eps, age_value = calculate(9.0, 0.02, 7.0, 0.0, 20.0)
    expected_value = joint_angle(9.0, math.radians(20.0), 0.0)
    assert abs(q - expected_value) < 1e-9
    assert abs(q + 11.0) < 1e-9
    assert eps == 9.0 and age_value == 0.02

    # If the status is stale, the command is also in the same WORLD frame and is translated to q.
    q, eps, age_value = calculate(99.0, 2.0, 7.0, -12.0, -15.0)
    expected_value = joint_angle(
        7.0, math.radians(-15.0), math.radians(-12.0))
    assert abs(q - expected_value) < 1e-9
    assert eps == 7.0 and age_value == 2.0

    print("Gazebo gimbal status frame test OK")


if __name__ == "__main__":
    main()
