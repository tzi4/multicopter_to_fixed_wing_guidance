#!/usr/bin/env bash
# UC ROTA A/B -- her algoritma degisikligi BU UC ROTADA kosulur (kullanici
# kurali, 2026-08-06): duz (GERCEKTEN duz), elips, wanderer.
#
# NEDEN UCU BIRDEN: tek rota yanlis hukum verdiriyor. Olculdu (TUR-5):
#   duz  'ta kazanc  ISKA zaman asimini bitirmek (2->0)
#   elips'te kazanc  CPA ortancasi 16.19 -> 6.52 m
# Ayni duzeltme, iki rotada FARKLI kanaldan kazandi. Wanderer (zikzak)
# ucuncu rejim: hedef surekli yon degistirdigi icin bozucu kestirimini
# ve yaw kanalini zorlar.
#
# KULLANIM:
#   tools/uc_rota_ab.sh "ETIKET" "ENV1=deger ENV1=deger ..."
# ORNEK (kapanma duzeltmesi A/B):
#   tools/uc_rota_ab.sh baseline ""
#   tools/uc_rota_ab.sh duzeltmeli "YILDIZ_Q_ALAN_CARPANI=4 YILDIZ_UFUK_MENZIL_REF=60"
#
# NOT: bu bir BETIK dosyasidir; komut satirinda "*_gudum.py" deseni GECMEZ,
# boylece senaryo.sh temizle()'nin pkill'i sarmalayiciyi vurmaz (DEVAM.md #4).
set -u
cd "$(dirname "$0")/.."

ETIKET="${1:?kullanim: uc_rota_ab.sh ETIKET \"ENV=deger ...\"}"
ENVLER="${2:-}"
SURE="${SURE:-360}"
GORSEL="${GORUNTULU:-mpc_gudum.py}"

ROTALAR="duz elips wanderer"

durdur() { ./yildizlar_gudum.sh --stop >/dev/null 2>&1 || true; sleep 8; }

echo "=========================================================="
echo "UC ROTA A/B  etiket=$ETIKET  sure=$SURE s"
echo "  env: ${ENVLER:-<yok, baseline>}"
echo "=========================================================="

for rota in $ROTALAR; do
  plan="missions/hedef_${rota}.plan"
  if [[ ! -f "$plan" ]]; then
    echo "!!! plan yok, atlaniyor: $plan"; continue
  fi
  echo
  echo ">>> ROTA=$rota  ($(date +%H:%M))"
  # shellcheck disable=SC2086
  env $ENVLER METOT="${ETIKET}_${rota}" SURE="$SURE" \
      GORUNTULU="$GORSEL" PLAN="$plan" tools/senaryo.sh
  echo ">>> $rota BITTI ($(date +%H:%M)) -- temizlik"
  durdur
done

echo
echo "=========================================================="
echo "UC ROTA BITTI: $ETIKET  ($(date +%H:%M))"
echo "Karsilastirma: python3 tools/karsilastir.py"
echo "=========================================================="
