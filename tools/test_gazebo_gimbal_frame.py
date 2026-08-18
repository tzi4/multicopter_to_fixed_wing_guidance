#!/usr/bin/env python3
"""Gazebo gimbal status cercevesinin eklem acisi sanilmasini engeller."""

import math
import pathlib
import sys

KOK = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KOK))

from bbox_to_redis import SuruRedisDetector
from yildizlar_gimbal import eklem_acisi


class SahteDurum:
    def __init__(self, eps_deg, yas_s):
        self.deger_rad = math.radians(eps_deg)
        self._yas_s = yas_s

    def yas_s(self):
        return self._yas_s


class SahteKomutcu:
    def __init__(self, hedef_deg):
        self.hedef_deg = hedef_deg


def hesapla(eps_status, yas_s, eps_cmd, roll_deg, pitch_deg):
    detector = SuruRedisDetector.__new__(SuruRedisDetector)
    detector.tilt_okuyucu = SahteDurum(eps_status, yas_s)
    detector.tilt_komutcu = SahteKomutcu(eps_cmd)
    return detector._eklem_hesapla(
        math.radians(roll_deg), math.radians(pitch_deg))


def main():
    # Gazebo plugin stabilize modunda +9 derece DUNYA kamera elevasyonu
    # yayinlar. Govde +20 derece pitch'teyken eklem yaklasik -11 derecedir;
    # status'u dogrudan q saymak body pitch'ini zincirden dusurur.
    q, eps, yas = hesapla(9.0, 0.02, 7.0, 0.0, 20.0)
    beklenen = eklem_acisi(9.0, math.radians(20.0), 0.0)
    assert abs(q - beklenen) < 1e-9
    assert abs(q + 11.0) < 1e-9
    assert eps == 9.0 and yas == 0.02

    # Status bayatsa komut da ayni DUNYA cercevesindedir ve q'ya cevrilir.
    q, eps, yas = hesapla(99.0, 2.0, 7.0, -12.0, -15.0)
    beklenen = eklem_acisi(
        7.0, math.radians(-15.0), math.radians(-12.0))
    assert abs(q - beklenen) < 1e-9
    assert eps == 7.0 and yas == 2.0

    print("Gazebo gimbal status frame test OK")


if __name__ == "__main__":
    main()
