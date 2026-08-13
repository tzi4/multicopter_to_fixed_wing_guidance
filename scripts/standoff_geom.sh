#!/usr/bin/env bash
# =====================================================================
# standoff_geom.sh - STANDOFF DIKEY GEOMETRISININ TEK KAYNAGI
# =====================================================================
# CALISTIRILMAZ, KAYNAKLANIR:  source scripts/standoff_geom.sh
#
# GIMBAL DALI (2026-08-05) - BAGIMLILIK YONU TERSINE DONDU:
# Kamera artik govdeye sabit degil; KENDINI STABILIZE EDEN fiziksel tek
# eksen (tilt) gimbalde. Eski dunyada standoff DOWN, sabit kamera acisina
# uymak ZORUNDAYDI (down = back*tan(mount+trim)); yeni dunyada DOWN serbest
# bir GOREV TASARIMI dugmesi, kamera ACISI ondan turetilir:
#
#     YILDIZ_TILT_DEG = atan(DOWN / BACK)      [kamera dunya elevasyonu]
#
# Tilt komutunu bbox_to_redis.py verir (gz topic, POZITIF = yukari);
# gövde pitch'i plugin tarafindan telafi edilir, PITCH_TRIM'in dikey
# kanaldaki isi OLDU (yaw/eski dondurulmus kollar icin export suruyor).
#
# UCUS KANITI (eski rejim, tarihce): back=25/down=13 (+30 montajla) tespit
# %98.4; down=3'te hedef eksenden 22 deg kacip tespit %5'e dusmustu. Ayni
# ders yeni rejimde de gecerli: TILT, atan(DOWN/BACK)'ten koparsa hedef
# eksenden kacar - o yuzden turetim hala TEK YERDE, burada.
#
# TUKETICILER: yildizlar_gudum.sh (bbox_to_redis.py --back/--down [--tilt];
# tilt verilmezse bbox kendisi atan(down/back) turetir - ayni formul) ve
# tools/senaryo.sh (simple_guided_follow.py --back/--down).
#
# EZME KAPILARI: YILDIZ_BACK, YILDIZ_DOWN, YILDIZ_TILT, YILDIZ_MOUNT,
# YILDIZ_PITCH_TRIM disaridan verilirse onlar gecerlidir.
# =====================================================================

YILDIZ_MOUNT="${YILDIZ_MOUNT:-0}"
YILDIZ_PITCH_TRIM="${YILDIZ_PITCH_TRIM:--2.5}"
YILDIZ_BACK="${YILDIZ_BACK:-25}"

# bash'te tan() yok: once python3, o yoksa awk (awk'ta da tan yok -> sin/cos).
# Her iki dal da `set -e` altinda guvenli olsun diye || ile yakalanir.
standoff_down_turet() {
  local d=''
  d=$(python3 -c 'import math,sys; b,m,t=(float(x) for x in sys.argv[1:4]); print(int(round(b*math.tan(math.radians(m+t)))))' \
        "$1" "$2" "$3" 2>/dev/null) || d=''
  if [[ -z "$d" ]]; then
    d=$(awk -v b="$1" -v m="$2" -v t="$3" \
          'BEGIN{r=(m+t)*atan2(0,-1)/180; printf "%.0f", b*sin(r)/cos(r)}' 2>/dev/null) || d=''
  fi
  if [[ -z "$d" ]]; then
    # Ne python3 ne awk: bos --down vermektense ucusla dogrulanmis ikiliye
    # dus, ama SESSIZ kalma (BACK 25'ten farkliysa bu deger yanlistir).
    echo "UYARI: standoff DOWN turetilemedi (python3/awk yok) -> 13 kullaniliyor" >&2
    d=13
  fi
  printf '%s' "$d"
}

# GIMBALLI / KUCUK MONTAJ KURULUMU (2026-08-04): montaj 0 dereceye cekilince
# turetim  down = back*tan(mount+trim)  NEGATIF cikiyor (0 + (-2.5) < 0), yani
# kopteri hedefin ALTINA degil USTUNE koyuyor -- arkadan carpma geometrisinin
# tam tersi. Kok neden: turetim, kamera acisinin GOVDEYE sabit oldugu
# varsayimina dayanir; gimbal (ya da montaj~0) o bagi KOPARIR ve standoff
# derinligi artik kameradan degil GOREV tasariminden gelir.
# Bu yuzden: turetim <= 0 verirse TASARIM DEGERINE dus ve SESSIZ KALMA.
# Tasarim degeri asagida YILDIZ_DOWN_TASARIM olarak TEK YERDE durur;
# tools/montaj_ayarla.py --down onu gunceller (uyari metni de ondan okur).
YILDIZ_DOWN_TASARIM=4      # gimballi kurulumun standoff derinligi [m]
# GIMBAL DALI: DOWN artik kameradan TURETILMEZ; gorev tasarim degeri tek
# kaynak. Eski mount-tabanli turetim yalnizca dondurulmus govdeye-sabit
# kollar icin YILDIZ_ESKI_TURETIM=1 ile ayakta.
if [[ "${YILDIZ_ESKI_TURETIM:-0}" == 1 ]]; then
  YILDIZ_DOWN="${YILDIZ_DOWN:-$(standoff_down_turet "$YILDIZ_BACK" "$YILDIZ_MOUNT" "$YILDIZ_PITCH_TRIM")}"
  if [[ "${YILDIZ_DOWN%%.*}" -le 0 ]] 2>/dev/null; then
    echo "UYARI: eski turetim DOWN=$YILDIZ_DOWN (<=0); tasarim degeri ${YILDIZ_DOWN_TASARIM} m." >&2
    YILDIZ_DOWN="$YILDIZ_DOWN_TASARIM"
  fi
else
  YILDIZ_DOWN="${YILDIZ_DOWN:-$YILDIZ_DOWN_TASARIM}"
fi

# Kamera tilt'i (dunya elevasyonu, + = yukari) standoff geometrisinden:
yildiz_tilt_turet() {
  python3 -c 'import math,sys; d,b=(float(x) for x in sys.argv[1:3]); print(f"{math.degrees(math.atan2(d,max(b,1e-6))):.2f}")' \
    "$1" "$2" 2>/dev/null || \
  awk -v d="$1" -v b="$2" 'BEGIN{printf "%.2f", atan2(d,b)*180/atan2(0,-1)}'
}
YILDIZ_TILT="${YILDIZ_TILT:-$(yildiz_tilt_turet "$YILDIZ_DOWN" "$YILDIZ_BACK")}"

export YILDIZ_MOUNT YILDIZ_PITCH_TRIM YILDIZ_BACK YILDIZ_DOWN YILDIZ_TILT
