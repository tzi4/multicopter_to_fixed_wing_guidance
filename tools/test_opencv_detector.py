#!/usr/bin/env python3
"""Check compatibility of OpenCV versions with real HSV target detection."""

import cv2
import numpy as np

from gimbal_bench_tracking import purple_find


def main():
    empty_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    assert purple_find(empty_frame) is None, "Target detected in empty square"

    frame_value = empty_frame.copy()
    cv2.rectangle(frame_value, (430, 250), (850, 510), (255, 0, 255), -1)
    result_value = purple_find(frame_value)
    assert result_value is not None, "Purple target not detected"

    center_x_value, center_y_value, width_value, height_value = result_value
    assert abs(center_x_value - 640.0) <= 2.0
    assert abs(center_y_value - 380.0) <= 2.0
    assert 419 <= width_value <= 423
    assert 259 <= height_value <= 263

    small = cv2.resize(frame_value, (640, 360), interpolation=cv2.INTER_AREA)
    successful, coded = cv2.imencode(".jpg", small)
    assert successful and coded.size > 0
    backward_increasing = cv2.imdecode(coded, cv2.IMREAD_COLOR)
    assert backward_increasing.shape == (360, 640, 3)

    print(
        "OpenCV HSV detector smoke test OK: "
        f"v {cv2.__version__} , center=( {center_x_value:.1f} , {center_y_value:.1f} )"
    )


if __name__ == "__main__":
    main()
