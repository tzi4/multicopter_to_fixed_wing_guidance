#!/usr/bin/env python3
"""OpenCV surumlerinin gercek HSV hedef tespit yoluyla uyumunu denetle."""

import cv2
import numpy as np

from gimbal_bench_takip import mor_bul


def main():
    bos_kare = np.zeros((720, 1280, 3), dtype=np.uint8)
    assert mor_bul(bos_kare) is None, "Bos karede hedef tespit edildi"

    kare = bos_kare.copy()
    cv2.rectangle(kare, (430, 250), (850, 510), (255, 0, 255), -1)
    sonuc = mor_bul(kare)
    assert sonuc is not None, "Mor hedef tespit edilemedi"

    merkez_x, merkez_y, genislik, yukseklik = sonuc
    assert abs(merkez_x - 640.0) <= 2.0
    assert abs(merkez_y - 380.0) <= 2.0
    assert 419 <= genislik <= 423
    assert 259 <= yukseklik <= 263

    kucuk = cv2.resize(kare, (640, 360), interpolation=cv2.INTER_AREA)
    basarili, kodlu = cv2.imencode(".jpg", kucuk)
    assert basarili and kodlu.size > 0
    geri_acilan = cv2.imdecode(kodlu, cv2.IMREAD_COLOR)
    assert geri_acilan.shape == (360, 640, 3)

    print(
        "OpenCV HSV detector smoke test OK: "
        f"v{cv2.__version__}, merkez=({merkez_x:.1f}, {merkez_y:.1f})"
    )


if __name__ == "__main__":
    main()
