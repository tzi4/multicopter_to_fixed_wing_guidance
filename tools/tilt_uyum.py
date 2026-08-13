#!/usr/bin/env python3
"""YIGININ KOMUTLADIGI KAMERA TILT'i ile SENARYONUN VARSAYDIGI TILT ayni mi?

NEDEN VAR (2026-08-09, ucuncu "sessiz ayrisma" dersi):
Standoff dikey geometrisi UC AYRI TUKETICIYE gidiyor ve hepsi ayni sayiya
inanmak ZORUNDA:
  (a) bbox_to_redis  -- fiziksel gimbale tilt KOMUTUNU verir. Bu, YIGIN
      ACILISINDA okunur (yildizlar_gudum.sh:425-431).
  (b) simple_guided_follow --back/--down  -- konumlu standoff'u kurar.
      KOSU basinda, senaryo.sh'den gelir.
  (c) mpc_gudum.cevre_mount_deg() (:214-235) -- kadraj referansi ey_ref'i
      belirler; ONCELIK $YILDIZ_TILT'tedir.
(b) ve (c) senaryo.sh'nin ortamindan, (a) ise YIGININ ortamindan gelir.
Ikisi ayri zamanlarda ve ayri env ile kuruldugu icin SESSIZCE AYRISABILIR:
  * yigin YILDIZ_DOWN=0 ile kalkar (kamera 0.00 deg'e bakar),
  * senaryo DOWN vermeden kosar -> standoff_geom TASARIM degerine (4) doner,
    konumlu 4 m alttan standoff kurar ve MPC "kamera ekseni +9.09" sanar.
Sonuc: kamera 0.00'a bakarken yasa 9.09'a baktigini varsayar. Hicbir hata
mesaji yoktur. (Tersi de olur: yigin 4, senaryo 0.)
Ayni sinif: tools/plan_uyum.py (hangi ROTA yuklu) -- bu ise hangi TILT komutlu.

NASIL OKUR: bbox_to_redis acilista tilt'i loga basar:
    "tilt (standoff geometrisi): back=25 down=4 -> +9.09 deg"   (turetilmis)
    "tilt elle verildi: +0.00 deg"                              (--tilt ile)
    "FIZIKSEL GIMBAL: iris-1 <- tilt +0.00 deg"                 (kesinlesen)
Log'daki SON kesinlesen degeri alir ve beklenen tilt ile karsilastirir.

CIKIS: 0 uyumlu | 1 UYUMSUZ | 2 karar verilemedi (log/deger okunamadi).
"""
import argparse, os, re, sys
from pathlib import Path

VARSAYILAN_LOG = str(Path(__file__).resolve().parents[1] / 'logs' / 'bbox.log')


def yigin_tilti(yol):
    """bbox.log'dan yiginin KESINLESEN tilt komutu (son gecerli kayit)."""
    if not os.path.exists(yol):
        return None, 'bbox.log yok'
    metin = open(yol, errors='replace').read()
    # Oncelik: "FIZIKSEL GIMBAL: <model> <- tilt +X deg" (kesinlesen deger)
    m = re.findall(r'FIZIKSEL GIMBAL: \S+ <- tilt ([+-]?[0-9.]+) deg', metin)
    if m:
        return float(m[-1]), 'FIZIKSEL GIMBAL satiri'
    m = re.findall(r'tilt elle verildi: ([+-]?[0-9.]+) deg', metin)
    if m:
        return float(m[-1]), 'tilt elle verildi satiri'
    m = re.findall(r'tilt \(standoff geometrisi\):.*?-> ([+-]?[0-9.]+) deg', metin)
    if m:
        return float(m[-1]), 'standoff geometrisi satiri'
    return None, 'bbox.log icinde tilt satiri bulunamadi'


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--beklenen', type=float, required=True,
                   help='senaryonun varsaydigi tilt [deg] (= $YILDIZ_TILT)')
    p.add_argument('--log', default=VARSAYILAN_LOG)
    p.add_argument('--tolerans', type=float, default=0.5, help='[deg]')
    a = p.parse_args()

    t, kaynak = yigin_tilti(a.log)
    if t is None:
        print(f"tilt_uyum: YIGIN tilt'i okunamadi ({kaynak})", file=sys.stderr)
        return 2
    fark = abs(t - a.beklenen)
    print(f"tilt_uyum: YIGIN (bbox) tilt = {t:+.2f} deg  [{kaynak}]")
    print(f"tilt_uyum: SENARYO beklenen  = {a.beklenen:+.2f} deg  (YILDIZ_TILT)")
    if fark <= a.tolerans:
        print(f"tilt_uyum: UYUMLU (fark {fark:.2f} deg)")
        return 0
    print(f"tilt_uyum: *** UYUMSUZ -- fark {fark:.2f} deg ***", file=sys.stderr)
    print("tilt_uyum: kamera bir aciya bakarken yasa BASKA bir aciya baktigini "
          "varsayar; kadraj referansi (ey_ref) ve dikey standoff ayrisir.",
          file=sys.stderr)
    print("tilt_uyum: cozum: senaryoyu yiginla AYNI DOWN ile kosun "
          "(or. DOWN=0), ya da yigini istenen DOWN ile yeniden baslatin.",
          file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
