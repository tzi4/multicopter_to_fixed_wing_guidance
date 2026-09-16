import time
import unittest

try:
    from hardware.balloon_range import RawTelemetryRange, range_norm
except ImportError:  # Runs directly from test hardware on Pi
    from balloon_range import RawTelemetryRange, range_norm


class _Reader:
    def __init__(self, target_value, wall):
        self.target_value = target_value
        self.wall = wall

    def get_with_times(self):
        return self.target_value, (99.0, 99.0, 99.0), 123.0, self.wall


class BalloonRangeTest(unittest.TestCase):
    def test_norm_only_length(self):
        self.assertAlmostEqual(range_norm((1, 2, 3), (4, 6, 3)), 5.0)

    def test_invalid_input_fail_closed(self):
        self.assertIsNone(range_norm(None, (1, 2, 3)))
        self.assertIsNone(range_norm((0, 0, 0), (float("nan"), 0, 0)))

    def test_fresh_telemetry_range(self):
        source_value = RawTelemetryRange.__new__(RawTelemetryRange)
        source_value.stale_s = 1.0
        source_value._reader_value = _Reader((3, 4, 0), time.monotonic())
        self.assertAlmostEqual(source_value.range_value((0, 0, 0)), 5.0)
        self.assertTrue(all(v is None for v in
                            source_value.ref_target_state().values()))

    def test_stale_telemetry_fail_closed(self):
        source_value = RawTelemetryRange.__new__(RawTelemetryRange)
        source_value.stale_s = 1.0
        source_value._reader_value = _Reader((3, 4, 0), time.monotonic() - 2.0)
        self.assertIsNone(source_value.range_value((0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
