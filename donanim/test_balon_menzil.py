import time
import unittest

try:
    from donanim.balon_menzil import HamTelemetriMenzil, menzil_normu
except ImportError:  # Pi'de test donanim/ icinden dogrudan calistirilir
    from balon_menzil import HamTelemetriMenzil, menzil_normu


class _Okuyucu:
    def __init__(self, hedef, wall):
        self.hedef = hedef
        self.wall = wall

    def get_with_times(self):
        return self.hedef, (99.0, 99.0, 99.0), 123.0, self.wall


class BalonMenzilTest(unittest.TestCase):
    def test_norm_yalniz_uzunluk(self):
        self.assertAlmostEqual(menzil_normu((1, 2, 3), (4, 6, 3)), 5.0)

    def test_gecersiz_girdi_fail_closed(self):
        self.assertIsNone(menzil_normu(None, (1, 2, 3)))
        self.assertIsNone(menzil_normu((0, 0, 0), (float("nan"), 0, 0)))

    def test_taze_telemetri_menzili(self):
        kaynak = HamTelemetriMenzil.__new__(HamTelemetriMenzil)
        kaynak.bayat_s = 1.0
        kaynak._okuyucu = _Okuyucu((3, 4, 0), time.monotonic())
        self.assertAlmostEqual(kaynak.menzil((0, 0, 0)), 5.0)
        self.assertTrue(all(v is None for v in
                            kaynak.ref_hedef_durum().values()))

    def test_bayat_telemetri_fail_closed(self):
        kaynak = HamTelemetriMenzil.__new__(HamTelemetriMenzil)
        kaynak.bayat_s = 1.0
        kaynak._okuyucu = _Okuyucu((3, 4, 0), time.monotonic() - 2.0)
        self.assertIsNone(kaynak.menzil((0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
