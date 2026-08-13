#!/usr/bin/env bash
# =====================================================================
# senaryo.sh - TEK KOMUTLA TAM DENEME
# =====================================================================
# Sifirdan kalkar: yigin -> hedef ucak elips rotada -> avci kopter havada
# -> guidance_allstar konumlu gudumu (standoff, carpma YOK) -> bbox olcumu.
#
# Her deneme kendi klasorunu acar: run/denemeler/<tarih>/
#   guidance.log   gudum ciktisi
#   bbox.log       dedektor ciktisi (o denemeye ait)
#   ozet.txt       kapanan mesafe, tespit sayisi, bbox boyutlari
#   video          videos/gudum_<tarih>.mp4 (YILDIZ_VIDEO=1 ile)
#
# KULLANIM:
#   tools/senaryo.sh                    # varsayilan: 300 s takip
#   SURE=600 tools/senaryo.sh           # daha uzun
#   BACK=35 tools/senaryo.sh            # DOWN montaj acisindan yeniden turer
#   BACK=25 DOWN=3 tools/senaryo.sh     # turetimi ez (kamera ekseni bozulur!)
#   YILDIZ_MOUNT=20 tools/senaryo.sh    # model.sdf montaji degistiyse
#   YENIDEN_BASLAT=0 tools/senaryo.sh   # yigin zaten ayakta, sadece ucur
# =====================================================================
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

SURE="${SURE:-300}"
# STANDOFF: BACK/DOWN artik burada SABIT DEGIL, kamera montaj acisindan
# turetilir (scripts/standoff_geom.sh: down = round(back*tan(mount+trim))).
# Gudumun ucurdugu ikili ile bbox_to_redis.py'ye giden ikili AYNI olmak
# zorundadir; ayrisirsa hedef kamera ekseninin disinda kalir
# (bkz. yildizlar_gimbal.analitik_aim = -atan(down/back)).
# BACK=/DOWN= (ya da YILDIZ_BACK=/YILDIZ_DOWN=) verilirse turetim EZILIR.
[[ -n "${BACK:-}" ]] && export YILDIZ_BACK="$BACK"
[[ -n "${DOWN:-}" ]] && export YILDIZ_DOWN="$DOWN"
# shellcheck source=../scripts/standoff_geom.sh
source "$SCRIPT_DIR/scripts/standoff_geom.sh"
# Turetim sonrasi kesinlesen ikili; asagidaki yigin yeniden baslatmasi da
# ayni degerleri (export edilmis YILDIZ_BACK/YILDIZ_DOWN) devralir.
BACK="$YILDIZ_BACK"
DOWN="$YILDIZ_DOWN"
SIDE="${SIDE:-0}"
DRONE_ALT="${DRONE_ALT:-60}"
YENIDEN_BASLAT="${YENIDEN_BASLAT:-1}"
# Taze yiginlarda hedef irtifa IMM'i ve avci EKF'i kalkistan hemen sonra
# gecici sicrayabiliyor. 0 eski davranistir; sorunlu baslangic A/B'lerinde
# KONTROL_BEKLE_S=20 gibi bir degerle gudumler baslamadan once oturtulur.
KONTROL_BEKLE_S="${KONTROL_BEKLE_S:-0}"

# --- ASILI HEDEF (HEDEF_ARAC=drone2) ---------------------------------
# Sabit kanat hedef gercekten DURAMAZ; "havada asili hedef" senaryosu ancak
# kopterle kurulabilir. HEDEF_ARAC=drone2 ile hedef rolu drone_2'ye gecer:
#   - ucak KALDIRILMAZ (yerde kalir; AUTO'ya sokulmaz),
#   - drone_2 GUIDED'de kalkip HEDEF_MESAFE m kuzeye gider ve orada asili kalir,
#   - gudum tarafinin hedef portlari guidance_config.py'de ayni env ile
#     14662/14664'e (drone_2) doner -- bkz. oradaki HEDEF_ARAC blogu.
# Bos birakilirsa (varsayilan) hicbir sey degismez.
HEDEF_ARAC="${HEDEF_ARAC:-}"
HEDEF_ALT="${HEDEF_ALT:-70}"
HEDEF_MESAFE="${HEDEF_MESAFE:-1200}"
HEDEF_GIDIS_S="${HEDEF_GIDIS_S:-90}"
HEDEF_BEKLE_S="${HEDEF_BEKLE_S:-90}"
if [[ "$HEDEF_ARAC" == drone2 ]]; then
  export HEDEF_ARAC          # gudum sureclerine (guidance_config.py) tasinsin
  # drone_2 ancak suru dunyasinda ve DRONE_COUNT>=2 iken var olur.
  export YILDIZ_DRONES="${YILDIZ_DRONES:-2}"
  export YILDIZ_WORLD="${YILDIZ_WORLD:-$SCRIPT_DIR/worlds/suru.world}"
fi

# --- Deney kimligi -----------------------------------------------------
# PLAN: hedef rotasi (duz / elips / wanderer). Varsayilan elips (orta seviye).
# GORUNTULU: guidance_allstar altinda calisacak goruntulu gudum komutu
#   (or. GORUNTULU="los_gudum.py"). Bos ise yalniz konumlu kosulur.
# METOT: video/klasor etiketi; verilmezse GORUNTULU dosya adindan turetilir.
PLAN="${PLAN:-$SCRIPT_DIR/missions/hedef_elips.plan}"
PLAN_AD="$(basename "$PLAN" .plan)"; PLAN_AD="${PLAN_AD#hedef_}"
if [[ -z "${METOT:-}" ]]; then
  if [[ -n "${GORUNTULU:-}" ]]; then
    METOT="$(basename "${GORUNTULU%% *}" .py)"; METOT="${METOT%_gudum}"
  else
    METOT="konumlu"
  fi
fi
ETIKET="${METOT}_${PLAN_AD}"
export YILDIZ_VIDEO_ETIKET="$ETIKET"

DAMGA="$(date +%Y%m%d_%H%M%S)"
DENEME_DIR="$SCRIPT_DIR/run/denemeler/${ETIKET}_$DAMGA"
mkdir -p "$DENEME_DIR"
echo ">>> deneme: $DENEME_DIR (video etiketi: $ETIKET)"

temizle() {
  # pgrep/pkill DESENI KENDI KOMUT SATIRIMIZI DA ESLER: koseli parantez
  # numarasi ('[f]ollow') deseni kendi kendine eslemekten kurtarir.
  for pid in $(pgrep -f "simple_guided_[f]ollow" || true); do kill "$pid" 2>/dev/null || true; done
  for pid in $(pgrep -f "suru_komut.py hiz-[k]ilidi" || true); do kill "$pid" 2>/dev/null || true; done
  # DESEN 'python3 ' ONEKI ISTER: cIplak "[a-z]*_gudum\.py" deseni, deneyi
  # BASLATAN sarmalayici kabugun kendi komut satirini da esliyordu
  # (or. "bash kos.sh los_gudum.py elips") ve deneyi baslatan sureci
  # oldurup exit 144 veriyordu -- kosu detached surerken cagiran olmus
  # gorunuyordu. Onek yalnizca gercek python surecine (ve timeout'una) uyar.
  # Yontem adi birden fazla alt-cizgili olabilir (terminal_los_gudum.py).
  # Eski [a-z]* deseni boyle surecleri kacirip iki kontrolcunun ayni MAVLink
  # portundan eszamanli komut vermesine izin veriyordu.
  for pid in $(pgrep -f "python3 goruntulu_[t]emel|python3 [a-z_]*_[g]udum\.py" || true); do kill "$pid" 2>/dev/null || true; done
}
trap temizle EXIT

temizle
sleep 2

if [[ "$YENIDEN_BASLAT" == 1 ]]; then
  echo ">>> yigin yeniden baslatiliyor (video kaydi acik)"
  ./yildizlar_gudum.sh --stop >/dev/null 2>&1 || true
  sleep 3
  YILDIZ_VIDEO=1 YILDIZ_TARGET_PLAN="$PLAN" \
    YILDIZ_AIM="${AIM:-0}" ./yildizlar_gudum.sh --headless
fi

if [[ "$HEDEF_ARAC" == drone2 ]]; then
  echo ">>> ASILI HEDEF: drone_2 kopteri, ${HEDEF_ALT} m, ${HEDEF_MESAFE} m kuzey"
  echo "    (hedef ucak KALDIRILMIYOR; $PLAN_AD plani yuklu ama AUTO'ya girmiyor)"
  # EKF POZISYON KESTIRIMI ICIN BEKLEME: yigin "hazir" dedikten hemen sonra
  # drone_2'nin ARM'i "Arm: Need Position Estimate" ile REDDEDILIYOR (olculdu
  # 2026-08-05, iki kosu ust uste bu yuzden dustu; ayni kopter ~10 dk sonra
  # elle sorunsuz kalkti). Ucak dalinda gorunmuyordu cunku orada once
  # 'hedef-kalkis' calisiyor ve o adim dakikalar suruyor -- EKF o sirada
  # oturuyordu. Asili hedef dalinda o gecikme yok, elle konuluyor.
  echo ">>> EKF oturmasi icin ${HEDEF_BEKLE_S} s bekleniyor"
  sleep "$HEDEF_BEKLE_S"
  kalkti=0
  for deneme in 1 2 3; do
    if python3 tools/suru_komut.py drone-kalkis --id 2 --alt "$HEDEF_ALT" \
        --timeout 180 | tail -1; then
      kalkti=1; break
    fi
    echo ">>> drone_2 kalkisi basarisiz (deneme $deneme/3), 30 s sonra tekrar"
    sleep 30
  done
  [[ "$kalkti" == 1 ]] || { echo "drone_2 kaldirilamadi -- deney iptal" >&2; exit 1; }
  # NEDEN 'hiz-testi': kopteri uzak bir noktaya gonderip ORADA BIRAKAN tek
  # mevcut komut bu (nokta_git -> GUIDED setpoint kalicidir, komut bitince
  # kopter noktada asili kalir). Hiz olcumu yan urun; amac drone_2'yi avcinin
  # kalkis noktasindan uzaga park etmek -- yoksa iki kopter 2.5 m arayla
  # dogar ve kapanma fazi hic yasanmaz.
  python3 tools/suru_komut.py hiz-testi --id 2 --mesafe "$HEDEF_MESAFE" \
    --hiz 20 --sure "$HEDEF_GIDIS_S" | tail -3
else
  # --- PLAN UYUM KAPISI (2026-08-09) --------------------------------
  # PLAN yalnizca YENIDEN_BASLAT=1 dalinda araca yukleniyor (yukarisi).
  # YENIDEN_BASLAT=0 ile kosunca PLAN SESSIZCE ETKISIZ kalir ve hedef,
  # yigin acilisindaki gorevi (varsayilan elips) ucmaya devam eder.
  # BU ARIZA IKI DENEMEYI YAKTI (tyawaccduz, kpnduz: "duz" etiketiyle
  # ELIPS uctular; biri "duz regresyon GECTI" diye raporlandi).
  # Bu yuzden VARSAYILAN DAVRANIS = DUR. SENARYO_PLAN_KONTROL=0 ile atlanir.
  PLAN_ETIKET="$PLAN_AD"
  if [[ "$YENIDEN_BASLAT" != 1 && "${SENARYO_PLAN_KONTROL:-1}" == 1 ]]; then
    echo ">>> plan uyum kontrolu (araca YUKLU gorev vs $PLAN_AD)"
    set +e
    python3 tools/plan_uyum.py --plan "$PLAN"
    PLAN_KOD=$?
    set -e
    if [[ "$PLAN_KOD" == 1 ]]; then
      echo "HATA: araca yuklu gorev '$PLAN_AD' DEGIL -- deney iptal." >&2
      echo "      Cozum: yigini su sekilde yeniden baslatin:" >&2
      echo "      ./yildizlar_gudum.sh --stop ; sleep 3" >&2
      echo "      YILDIZ_TARGET_PLAN=$PLAN ./yildizlar_gudum.sh" >&2
      echo "      bilerek atlamak icin: SENARYO_PLAN_KONTROL=0" >&2
      exit 1
    elif [[ "$PLAN_KOD" != 0 ]]; then
      echo "UYARI: plan uyumu DOGRULANAMADI (arac/plan okunamadi)." >&2
      PLAN_ETIKET="$PLAN_AD (dogrulanmadi)"
    fi
  elif [[ "$YENIDEN_BASLAT" != 1 ]]; then
    PLAN_ETIKET="$PLAN_AD (dogrulanmadi: SENARYO_PLAN_KONTROL=0)"
  fi
  # Etiket artik DOGRULAMA DURUMUNU tasiyor: eski satir PLAN_AD'den
  # geliyordu ve araca ne yuklendigini SOYLEMIYORDU -- yaniltiyordu.
  echo ">>> hedef ucak: AUTO, $PLAN_ETIKET rota"
  python3 tools/suru_komut.py hedef-kalkis --alt 55 --timeout 240 | tail -2
fi

echo ">>> avci kopter: GUIDED, $DRONE_ALT m"
python3 tools/suru_komut.py drone-kalkis --id 1 --alt "$DRONE_ALT" --timeout 180 | tail -1
if awk -v s="$KONTROL_BEKLE_S" 'BEGIN { exit !(s > 0) }'; then
  echo ">>> estimator/EKF oturma beklemesi: ${KONTROL_BEKLE_S} s"
  sleep "$KONTROL_BEKLE_S"
fi

# bbox log'unun bu denemeye ait kismini ayirmak icin isaret koy
BBOX_BASLANGIC=$(wc -l < logs/bbox.log)

# AIM olcumu: gudumle ES ZAMANLI kos (aim=0 iken ey = -yukselis)
setsid python3 tools/aim_olc.py --sure "$SURE" --etiket "$PLAN_AD" \
  > "$DENEME_DIR/aim.txt" 2>&1 < /dev/null &
disown

# Goruntulu gudum: konumluyla BIRLIKTE ayaga kalkar, Redis 'komut_yetkisi'
# 'goruntulu' olana kadar komut GONDERMEZ (goruntulu_temel.py bekletir).
if [[ -n "${GORUNTULU:-}" ]]; then
  echo ">>> goruntulu gudum yetki bekliyor: $GORUNTULU"
  (
    cd guidance_allstar
    # PYTHONUNBUFFERED: log dosyaya gittigi icin print'ler blok tamponlanir;
    # canli izleme/olum tanisi icin satir satir aksin.
    # shellcheck disable=SC2086
    PYTHONUNBUFFERED=1 setsid timeout $((SURE + 60)) python3 $GORUNTULU \
      > "$DENEME_DIR/goruntulu.log" 2>&1 < /dev/null &
  )
fi

# --- YAW KILIDI ------------------------------------------------------
# GERI GELDI (2026-08-03), YAW_KILIT=0 ile kapatilir. NEDEN GERI:
# gorunturlu devir icin hedefin konumlu YAKLASMADA kadrajda kalmasi sart;
# govdeye sabit kamera (yatay FOV +-33 derece) ancak burun hedefe donukse
# hedefi gorur. pid_duz kosusunda OLCULDU: yaw komutlanmayinca (otopilotta)
# duz/kesisen geometride kopter ~-65 derecede asili kaliyor, yakin fazda
# (menzil<60 m) hedef kerterizi burna gore ORTANCA 95 derece sapiyor ve
# hedef karelerin yalniz %30'unda kadrajda -- kararli 1.5 s pencere
# olusmadigi icin devir HIC tetiklenmiyor (elipste hedef onde kaldigi icin
# calisiyordu). NEDEN KAPATILMISTI: yavas dongu (2 Hz) + sabit-dt varsayimi
# yaw'i 344 deg/s ile kovalatip titretiyordu; ikisi de a5a28eb'de duzeldi
# (olculen 19.9 Hz). Guvenlik onlemleri guidance_config.py'de duruyor:
# YAW_LOCK_MODE="los" (burun hedefe), 90 deg/s slew, 38 derece tilt ustunde
# dondurma, 10 m altinda son yaw'i tut. Titreme geri gelirse YAW_KILIT=0.
YAW_KILIT="${YAW_KILIT:-1}"
YAW_BAYRAK=(); [[ "$YAW_KILIT" == 1 ]] && YAW_BAYRAK=(--yaw-lock)

# --- YAKLASIM YASASI (devir ONCESI konumlu faz) ----------------------
# YAKLASIM=slot|kesisme -> simple_guided_follow.py --yaklasim'a gecer.
# BOS BIRAKILIRSA HICBIR BAYRAK EKLENMEZ, yani bugunku davranis BIT-AYNI
# (betigin kendi varsayilani zaten 'slot'; burada varsayilan YAZMIYORUZ ki
# betikteki varsayilan tek kaynak kalsin).
# UYARI: 'kesisme' kolu konumlu fazda hedefe NISAN ALIR (carpisma rotasi);
# temas GORUNTULUYE DEVIRDEN ONCE de olabilir. Bkz. simple_guided_follow.py
# KesismeGuidance ve memory: "yaklasim yasasi iki secenek".
YAKLASIM="${YAKLASIM:-}"
YAKLASIM_BAYRAK=()
if [[ -n "$YAKLASIM" ]]; then
  YAKLASIM_BAYRAK=(--yaklasim "$YAKLASIM")
  echo ">>> YAKLASIM YASASI = ${YAKLASIM} (konumlu faz)"
fi
echo ">>> konumlu gudum: guidance_allstar (kill mode KAPALI, yaw kilidi=${YAW_KILIT})"
echo "    standoff: back=${BACK} m side=${SIDE} m down=${DOWN} m, sure=${SURE} s"
# --- TILT UYUM KAPISI (2026-08-09) ---------------------------------
# Standoff dikey geometrisi UC tuketiciye gidiyor: bbox (YIGIN acilisinda),
# simple_guided_follow ve mpc_gudum (KOSU basinda, buradan). Ilk ikisi ayri
# ortamlardan beslendigi icin SESSIZCE AYRISABILIYOR -- olculdu: yigin
# DOWN=0 ile kalkmisken senaryo DOWN vermeyince tasarim degerine (4) donuyor
# ve MPC "kamera ekseni +9.09" saniyordu; kamera ise 0.00'a bakiyordu.
# plan_uyum.py ile ayni sinif kontrol: SENARYO_TILT_KONTROL=0 ile atlanir.
if [[ "${SENARYO_TILT_KONTROL:-1}" == 1 ]]; then
  set +e
  python3 tools/tilt_uyum.py --beklenen "$YILDIZ_TILT"
  TILT_KOD=$?
  set -e
  if [[ "$TILT_KOD" == 1 ]]; then
    echo "HATA: yiginin komutladigi kamera tilt'i senaryonunkiyle UYUSMUYOR" >&2
    echo "      -- deney iptal (kadraj referansi ayrisir)." >&2
    echo "      Cozum: senaryoyu yiginla ayni DOWN ile kosun (or. DOWN=0)," >&2
    echo "      ya da yigini istenen DOWN ile yeniden baslatin." >&2
    echo "      bilerek atlamak icin: SENARYO_TILT_KONTROL=0" >&2
    exit 1
  elif [[ "$TILT_KOD" != 0 ]]; then
    echo "UYARI: tilt uyumu DOGRULANAMADI (bbox.log okunamadi)." >&2
  fi
fi
# YILDIZ_NO_GUI=1: estimator penceresi headless testte aniden acilmasin.
# GUI istenirse YILDIZ_NO_GUI=0 verilir.
(
  cd guidance_allstar
  YILDIZ_NO_GUI="${YILDIZ_NO_GUI:-1}" timeout "$SURE" python3 simple_guided_follow.py \
    --no-kill-mode "${YAW_BAYRAK[@]}" "${YAKLASIM_BAYRAK[@]}" \
    --back "$BACK" --side "$SIDE" --down "$DOWN" \
    > "$DENEME_DIR/guidance.log" 2>&1 || true
)

echo ">>> deneme bitti, ozet cikariliyor"
tail -n +"$BBOX_BASLANGIC" logs/bbox.log > "$DENEME_DIR/bbox.log" || true
python3 tools/deneme_ozeti.py "$DENEME_DIR" | tee "$DENEME_DIR/ozet.txt"
echo; cat "$DENEME_DIR/aim.txt" 2>/dev/null | tail -8
