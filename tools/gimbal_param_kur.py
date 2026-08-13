#!/usr/bin/env python3
"""
gimbal_param_kur.py -- servo gimbal icin ArduPilot parametrelerini KUR/DOGRULA
=============================================================================
NEDEN BU ARAC: gimbal komutu (MAV_CMD_DO_MOUNT_CONTROL / GIMBAL_MANAGER) ancak
otopilotta mount SERVO surucusu tanimliysa servoya doner. O tanim bir avuc
parametreden ibaret ve ELLE girilince en sik iki hata oluyor:
  (1) servo baska bir kanala lehimlenmis ama SERVOx_FUNCTION baska kanalda,
  (2) MNT1_TYPE degistikten sonra REBOOT edilmemis (parametre yazilir ama
      surucu yuklenmez; disaridan "komut gidiyor, servo kimildamiyor" gorunur).
Bu arac ikisini de kapatir: yazar, GERI OKUR, dogrular ve reboot gerekliligini
soyler.

NEDEN VARSAYILAN 'KURU': parametre yazmak araci kalici olarak degistirir.
Once --kanal ile ne yazilacagini GOSTERIR; gercekten yazmak icin --uygula
gerekir (repodaki tools/tilt_ayarla.py ile ayni disiplin).

KULLANIM:
    # ne yazilacagini gor (hicbir sey yazmaz)
    tools/gimbal_param_kur.py --kanal 5

    # gercekten yaz + geri okuyup dogrula
    tools/gimbal_param_kur.py --kanal 5 --uygula

    # yalniz mevcut degerleri oku (teshis)
    tools/gimbal_param_kur.py --kanal 5 --oku

    # gercek donanim (seri port)
    tools/gimbal_param_kur.py --baglanti /dev/ttyAMA0 --baud 57600 --kanal 5 --uygula

SONRASI: tools/mavlink_tilt.py --kanal 5   (komut ver, servo kimildiyor mu bak)
         tools/gimbal_bench_takip.py        (govdeyi yatir, kamera sabit mi)
"""

import argparse
import sys
import time

from pymavlink import mavutil

# --- PARAMETRE KUMESI ------------------------------------------------
# MNT1_TYPE=1 (Servo): mount1'i servo surucusune baglar. ArduPilot bu
#   parametreyi ACILISTA okur -> degisirse REBOOT SART.
# SERVOx_FUNCTION=7 (Mount1Pitch): tek eksen (pitch) gimbal bizim durumumuz.
# MNT1_MODE=2 (MAVLINK_TARGETING): aci komutunu MAVLink'ten (Pi'den) alir.
#   RC_TARGETING (3) olursa komutlarimiz yok sayilir, gimbal RC'yi dinler.
# MNT1_PITCH_MIN/MAX: yazilim kelepcesi. Fiziksel gimbalin GERCEK aralugindan
#   dar tutulur ki servo mekanik limite dayanmasin. Varsayilanlar
#   TiltTakip'in yazilim kelepcesiyle (tools/gz_gimbal.py: -30..+60) uyumlu
#   secildi; gimbalin daha darsa --pitch-min/--pitch-max ile ver.
MNT_TYPE_SERVO = 1
SERVO_FUNC_MOUNT1_PITCH = 7
MNT_MODE_MAVLINK = 2


def param_kumesi(kanal, pitch_min, pitch_max):
    """(ad, deger, tip, reboot_ister) listesi -- yazim SIRASI onemli degil."""
    return [
        ('MNT1_TYPE', MNT_TYPE_SERVO,
         mavutil.mavlink.MAV_PARAM_TYPE_INT8, True),
        (f'SERVO{kanal}_FUNCTION', SERVO_FUNC_MOUNT1_PITCH,
         mavutil.mavlink.MAV_PARAM_TYPE_INT16, True),
        ('MNT1_MODE', MNT_MODE_MAVLINK,
         mavutil.mavlink.MAV_PARAM_TYPE_INT8, False),
        ('MNT1_PITCH_MIN', pitch_min,
         mavutil.mavlink.MAV_PARAM_TYPE_REAL32, False),
        ('MNT1_PITCH_MAX', pitch_max,
         mavutil.mavlink.MAV_PARAM_TYPE_REAL32, False),
    ]


def oku(mav, ad, zaman_asimi=3.0):
    """Tek parametreyi oku; yoksa None. (Firmware'de olmayan ad sessiz kalir.)"""
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component, ad.encode(), -1)
    bitis = time.time() + zaman_asimi
    while time.time() < bitis:
        m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == ad:
            return m.param_value
    return None


def yaz_ve_dogrula(mav, ad, deger, tip, deneme=3):
    """Yaz -> geri oku -> karsilastir. Otopilot ACK vermez, dogrulama SART."""
    for i in range(deneme):
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               ad.encode(), float(deger), tip)
        time.sleep(0.3)
        okunan = oku(mav, ad)
        if okunan is not None and abs(okunan - float(deger)) < 1e-4:
            return True, okunan
        if i < deneme - 1:
            time.sleep(0.5)
    return False, okunan


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--baglanti', default='tcp:127.0.0.1:5760',
                   help='SITL: tcp:127.0.0.1:5760 | donanim: /dev/ttyAMA0')
    p.add_argument('--baud', type=int, default=57600,
                   help='yalniz seri baglantida kullanilir')
    p.add_argument('--kanal', type=int, default=5,
                   help='mount pitch servosunun LEHIMLENDIGI cikis '
                        '(SERVOx_FUNCTION=7 buraya yazilir)')
    p.add_argument('--pitch-min', type=float, default=-30.0,
                   help='MNT1_PITCH_MIN [deg] (asagi bakis siniri)')
    p.add_argument('--pitch-max', type=float, default=60.0,
                   help='MNT1_PITCH_MAX [deg] (yukari bakis siniri)')
    p.add_argument('--uygula', action='store_true',
                   help='GERCEKTEN yaz (yoksa yalniz ne yazilacagini gosterir)')
    p.add_argument('--oku', action='store_true',
                   help='yalniz mevcut degerleri oku ve bas (teshis)')
    a = p.parse_args()

    kume = param_kumesi(a.kanal, a.pitch_min, a.pitch_max)

    if not (a.uygula or a.oku):
        print("KURU KOSU -- hicbir sey yazilmadi. Yazmak icin --uygula ekle.\n")
        print(f"  kanal {a.kanal} icin yazilacaklar:")
        for ad, deger, _, reboot in kume:
            print(f"    {ad:18s} = {deger:g}" + ("   [REBOOT ister]" if reboot else ""))
        print("\n  UYARI: servo HANGI cikisa lehimliyse --kanal O olmali.")
        return 0

    print(f"baglaniliyor: {a.baglanti}")
    if a.baglanti.startswith('/dev/') or a.baglanti.startswith('COM'):
        mav = mavutil.mavlink_connection(a.baglanti, baud=a.baud, source_system=254)
    else:
        mav = mavutil.mavlink_connection(a.baglanti, source_system=254)
    mav.wait_heartbeat(timeout=15)
    if mav.target_system == 0:
        print("HATA: HEARTBEAT yok -- baglanti/baud dogru mu?", file=sys.stderr)
        return 1
    print(f"baglandi: sysid={mav.target_system} comp={mav.target_component}\n")

    if a.oku:
        print("MEVCUT DEGERLER:")
        for ad, beklenen, _, _ in kume:
            v = oku(mav, ad)
            if v is None:
                durum = "YOK (firmware'de tanimsiz?)"
            elif abs(v - float(beklenen)) < 1e-4:
                durum = "beklenen deger"
            else:
                durum = f"FARKLI (beklenen {beklenen:g})"
            print(f"  {ad:18s} = {'--' if v is None else f'{v:g}':>8s}   {durum}")
        return 0

    reboot_gerek = False
    hata = False
    print("YAZILIYOR (her biri geri okunarak dogrulanir):")
    for ad, deger, tip, reboot in kume:
        onceki = oku(mav, ad)
        tamam, okunan = yaz_ve_dogrula(mav, ad, deger, tip)
        if tamam:
            degisti = onceki is None or abs(onceki - float(deger)) > 1e-4
            print(f"  {ad:18s} = {deger:g}  OK" +
                  (f"   (onceki {onceki:g})" if degisti and onceki is not None else ""))
            if reboot and degisti:
                reboot_gerek = True
        else:
            hata = True
            print(f"  {ad:18s} = {deger:g}  BASARISIZ "
                  f"(okunan: {'yok' if okunan is None else f'{okunan:g}'})")

    print()
    if hata:
        print("BAZI PARAMETRELER YAZILAMADI. Sik sebepler: parametre adi bu")
        print("firmware'de yok (surum farki), ya da arac ARMED (bazi param'lar")
        print("armed iken kilitli).")
        return 1

    if reboot_gerek:
        print("*** REBOOT SART ***  MNT1_TYPE / SERVOx_FUNCTION acilista okunur.")
        print("Otopilotu yeniden baslat, sonra dogrula:")
        print(f"    tools/gimbal_param_kur.py --baglanti {a.baglanti} "
              f"--kanal {a.kanal} --oku")
    else:
        print("Reboot gerekmiyor (tip/fonksiyon zaten dogruydu).")

    print("\nSIRADAKI ADIM -- servo gercekten kimildiyor mu:")
    print(f"    tools/mavlink_tilt.py --baglanti {a.baglanti} --kanal {a.kanal}")
    print("Sonra govde yatirma testi: tools/gimbal_bench_takip.py "
          "(bkz. donanim/GIMBAL_TAKIP_TESTI.md)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
