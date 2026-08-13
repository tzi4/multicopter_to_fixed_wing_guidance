# NOTLAR_MPC — MPC yöntemi: testleri kendin koşmak (2026-08-11 güncel)

> **GÜNCEL ANA ADAY (2026-08-11):** Konumlu yaklaşımın ardından 5 ardışık
> büyük bbox ile devralan **tek LOS/PN yasasıdır**. Günlük kullanım ve gerçek
> donanım sırası: **`NOTLAR_LOS_DONANIM.md`**. Bu dosya saf MPC geçmişini,
> `NOTLAR_HIBRIT.md` ise MPC→LOS karşılaştırma kolunu korur.
>
> Kod ilişkisi: `hibrit_gudum.py`, 18 m dışında `mpc_gudum.py`yi; 18 m
> içinde `terminal_los_gudum.py`yi kullanır. Dolayısıyla `mpc_gudum.py`
> silinmedi, hibritin orta-menzil alt bileşeni oldu.

> Bu dosya **MPC kolunun** (`guidance_allstar/mpc_gudum.py`) notudur.
> ArduPilot FOLLOW kolu için: **`NOTLAR_TAKIP.md`** (dal: `ardupilot-takip`).
> Ortak yığın/ortam bölümleri iki dosyada da aynıdır.
>
> **ÖNCE OKU — 2026-08-05 arızası:** Görüntülü güdüm sürecini BAŞLATMAYI
> unutursan, karar verici yetkiyi "görüntülü"ye çevirir ve aracı KİMSE
> komutlamaz. Belirti aldatıcıdır: "MPC titriyor ve hedefi hiç takip etmiyor"
> gibi görünür ama MPC hiç koşmuyordur. Artık ölü-adam anahtarı var
> (`goruntulu_hayatta`), geçiş engellenir ve bbox.log'a şu düşer:
> `[KARAR] goruntulu kontrolcu YOK ... Goruntulu gudumu baslatmayi unuttun mu?`

```bash
# ======================= NEREDE ÇALIŞIR ========================
# TÜM komutlar depo kökünden koşar (içinde kendi `cd`si olanlar hariç):
cd /path/to/multicopter_to_fixed_wing_guidance
# `cd guidance_allstar && ...` yazan satırlar kökten girildiğini varsayar.

# ============================ YIĞIN ============================
YILDIZ_VIDEO=1 ./yildizlar_gudum.sh --robofly --headless    # güncel test aracı + bbox videosu
# GUI istersen --headless'sız (QGC dahil)
./yildizlar_gudum.sh --stop

# ==================== ARAÇLARI GÖREVE SOK ======================
# plan varsayılanı hedef_elips; diğerleri: hedef_duz / hedef_tur / hedef_wanderer
# hedef_duz  = GERÇEKTEN DÜZ, 200 km kuzey, DO_JUMP/RTL YOK (2026-08-05'te
#   düzeltildi: eskiden dikdörtgen turdu, adıyla çelişiyordu — tur için
#   hedef_tur zaten var). 20 m/s'te 2.8 saat; 360 s'lik koşuda planın %3.6'sı
#   kullanılır, yani hedef ASLA dönmez / RTL'ye girmez.
# hedef_sonsuz = aynı fikir, 40 km (daha kısa düz rota)
# DIKKAT: asagidaki --plan SATIRI OPSIYONEL. Vermezsen hedef_elips kosar.
# KOSE PARANTEZ YAZMA -- o "opsiyonel" demek, komutun parcasi DEGIL.
python3 tools/gorev_baslat.py --drones 1 --drone-alt 60
# ...ya da plan sec (parantezsiz):
python3 tools/gorev_baslat.py --drones 1 --drone-alt 60 --plan missions/hedef_duz.plan

# ############################################################
# # KOŞU SIRASI — İKİ AYRI TERMİNAL, İKİSİ DE GEREKLİ         #
# ############################################################
# [1] KONUMLU (yaklaşım):
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
# [2] GÖRÜNTÜLÜ (devralan) — BUNU UNUTMA:
cd guidance_allstar && python3 hibrit_gudum.py
#
# [2] BAŞLATILMAZSA GÖRÜNTÜLÜYE GEÇİLMEZ. Ölü-adam anahtarı
# ('goruntulu_hayatta') geçişi engeller; bbox.log'a şu düşer:
#   [KARAR] goruntulu kontrolcu YOK ... baslatmayi unuttun mu?
# TEŞHİS: guidance_allstar/logs/goruntulu_hibrit_*.csv YOKSA süreç hiç
# başlamamıştır (o dosya yetki beklemeden ÖNCE açılır).
# Bu hata 2026-08-05'te ÜÇ KEZ tekrarlandı; belirtisi aldatıcı:
# "MPC titriyor ve hedefi hiç takip etmiyor" gibi görünür.

# ============ YAKLAŞIM YASASI (devirden ÖNCEKİ faz) ============
# İKİ SEÇENEK VAR (2026-08-05). Fark TEMELDİR:
#   --yaklasim slot     (VARSAYILAN, kanıtlı): hedefin ARKASINDA standoff
#     slotu; hedefe nişan ALMAZ. KUYRUK geometrisi kurar -> devir oradan.
#   --yaklasim kesisme  (formation_KILLER ATTACK yasası): hedefin GİDECEĞİ
#     yere nişan alır, kesişmenin 50 m ÖTESİNE uzatır (fren yok).
#     Devir genelde ÇAPRAZ/KAFA-KAFAYA geometride olur.
#     UYARI: çapraz devir MPC'nin tarihsel EN KÖTÜ hali (kerteriz-rota ≥45°
#     olan 11 segmentin 11'i ıska). q_alan/ufuk düzeltmeleri offline çözdü,
#     sim'de DOĞRULANMADI. Bu yüzden varsayılan değil.
#   Düğmeler: --kesisme-hiz 24 --kesisme-terminal 45 --kesisme-overshoot 50
# NOT: formation_KILLER.py'nin KENDİSİ koşmaz (config.py yok, sürü
# kontrolcüsü + klavye/waypoints ister); yalnız YASA taşındı.

# ======================= KONUMLU GÜDÜM =========================
# DEĞİŞTİ: --yaw-lock artık VARSAYILAN (senaryo.sh veriyor; YAW_KILIT=0 kapatır).
# Kapalıyken düz rotada hedef kadraja girmiyor, tespit %2.7'ye düşüyordu.
# estimator penceresi varsayılan kapalı — log için --gui, uçarken açma (GIL boğuyor)
#
# İKİ YAKLAŞIM SEÇENEĞİ (devirden önceki faz):
python3 simple_guided_follow.py --yaklasim slot      # VARSAYILAN, arkadan (mevcut)
python3 simple_guided_follow.py --yaklasim kesisme   # formation_KILLER ATTACK yasası
#   düğmeler: --kesisme-hiz 24 --kesisme-terminal 45 --kesisme-overshoot 50
#
# tam hali (varsayılan yaklaşımla):
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6
# DEĞİŞTİ: 25/6. GİMBALLİ/0° kurulumda down TÜRETİLMEZ, elle verilir
# (türetim down=round(back*tan(mount+trim)) mount=0'da negatif çıkar → kopteri hedefin ÜSTÜNE koyar)
# [GİMBAL DALI GÜNCELLEMESİ: bu artık geçici bir çare değil, KURAL. down/back
#  görev tasarımı; kamera açısı ONDAN türetilir (YILDIZ_TILT=atan(down/back)).
#  Eski mount-tabanlı türetim yalnız YILDIZ_ESKI_TURETIM=1 ile ayakta.]

# ======================= GÖRÜNTÜLÜ GÜDÜM =======================
# Konumluyla BİRLİKTE ayağa kalkar, Redis 'komut_yetkisi'='goruntulu' olana kadar komut GÖNDERMEZ.
cd guidance_allstar && python3 hibrit_gudum.py     # güncel: dışarıda MPC, 18 m içinde PN/LOS
# Saf-MPC karşılaştırma kolu: python3 mpc_gudum.py
# Portlar: konumlu avcı 14652 / hedef 14603 ; GÖRÜNTÜLÜ avcı 14654 / hedef 14604 (aynı port iki kez bağlanamaz)

# ===================== TAM DENEME (tek komut) ==================
# kalkış → konumlu → HİBRİT görüntülü devir → video → aim → özet. AIM VERME: aim=0 kasıtlı
YILDIZ_DRONE_MODEL=robofly SURE=360 KONTROL_BEKLE_S=20 GORUNTULU="hibrit_gudum.py" PLAN=missions/hedef_elips.plan METOT=hibrit tools/senaryo.sh
#   GORUNTULU= boş → yalnız konumlu | METOT=xxx → video/klasör etiketi
#   BACK=25 DOWN=6 → standoff'u ez (montaj_ayarla.py ile UYUMLU tut)
#   YAW_KILIT=0 → konumluda yaw-lock'u kapat
# çıktı: run/denemeler/<metot>_<rota>_<damga>/ + videos/<metot>_<rota>_<damga>.mp4

# 2026-08-10 CANLI A/B SONUCU: güncel aday orta-faz MPC + terminal PN/LOS hibriti.
# Taze yığında 20 s bekleme, kalkış/kestirim irtifa sıçramasını önler.
# Saf terminal kontrol kolu: GORUNTULU="terminal_los_gudum.py"
# Tarama hücresi örneği:
# GORUNTULU="hibrit_gudum.py --n-pn 5 --gecis-menzil 20 --tirmanma-hiz-max 2.0"
# Sonuç/karar ve sıradaki tarama: TO_TEST.md → CANLI T2 HÜKMÜ

# Koşu sağlığı: "DONGU YAVAS" = CPU sıkışık; "SIMULASYON GERIDE" = koşu GEÇERSİZ
# canlı: loop=..Hz sim=..   CSV: loop_dt_meas_s, simtime_ratio (1.00 iyi, 0.74 çöp)

# =================== MONTAJ AÇISI (0° — YENİ) ==================
# [GİMBAL DALI GÜNCELLEMESİ 2026-08-05: bu blok TARİHSEL. Kamera artık
#  kendini stabilize eden FİZİKSEL tek eksen (tilt) gimbalde; "montaj açısı"
#  diye bir düğme KALMADI (SDF cam pose pitch'i 0 kalmalı — oraya açı yazmak
#  gimbalin komutladığı elevasyonun üstüne sessiz ofset bindirir).
#  Bağımlılık TERSİNE döndü: down/back serbest görev tasarımı, kamera açısı
#  ondan türetilir → YILDIZ_TILT = atan(down/back) (dünya elevasyonu, + yukarı).
#  YENİ ARAÇ: python3 tools/tilt_ayarla.py --down 6 --back 25 [--uygula]
#  montaj_ayarla.py'nin YAZMA yolu kapatıldı; yalnız --goster çalışır.
#  Ayrıntı: NOTLAR_GIMBAL.md]
# Montaj TEK düğme DEĞİL: SDF + sanal gimbal + standoff ÜÇÜ birlikte değişmeli.
python3 tools/montaj_ayarla.py --goster                       # mevcut durum
python3 tools/montaj_ayarla.py --mount 0 --back 25 --down 6   # kuru koşu (YAZMAZ)
python3 tools/montaj_ayarla.py --mount 0 --back 25 --down 6 --uygula   # + statik testi koşar
python3 tools/montaj_ayarla.py --geri-al                      # git checkout ile geri
# KURAL: montaj ≈ ÇARPMA anındaki hedef LOS yükselişi. Arkadan eş-irtifa çarpmada LOS→0 → montaj 0.
# Gerçek donanımda gövde dash'te dik burun-aşağı (18 m/s → −34°) → orada GİMBAL şart.

# ============ GİMBAL STATİK TEST / AIM / KALİBRASYON ===========
python3 yildizlar_gimbal.py --test [--aim -27]    # ADI DEĞİŞTİ (eski: sanal_gimbal.py)
python3 tools/aim_olc.py --sure 300 --etiket deneme   # senaryo aim=0 koşarken; ey ortancası = aim
python3 tools/kamera_kalibrasyon.py --port 14551 --kuzey 700 --dogu 250 --alt 60 --tarama 8,130,150 --sure 380

# ==================== ELLE KOMUT / ÖZET ========================
python3 tools/suru_komut.py durum
python3 tools/suru_komut.py hiz-testi --id 1 --mesafe 3000 --hiz 35
python3 tools/suru_komut.py hedef-kalkis | drone-kalkis --id 1 --alt 60 | pusu | takip | hiz-kilidi
python3 tools/deneme_ozeti.py run/denemeler/<klasor>
python3 tools/karsilastir.py [--metot mpc los pid] [--csv rapor.csv]   # YENİ: yöntem kıyası
#   alan_px2 = bbox alanı tepe [px², LİNEER, BİRİNCİL ödül] | a_hiz90 = alan büyüme hızı (İKİNCİL)
#   ex/ey_rms = merkezleme | ÇARPMA delili: min_m<3 + vibe sıçraması
#   DİKKAT: vibe tepesi hedef uzakta + pos_z~0 ise YER TEMASIDIR (vibe_menzil_m/vibe_pos_z ayırır)
# DİKKAT: deneme_ozeti "dongu ornegi" gerçeğin ~1/10'u; GEOMETRI bölümü verilen klasörü değil EN YENİ csv'yi okur

# ================= OFFLINE TESTLER (sim gerekmez) ==============
cd guidance_allstar && python3 mpc_test.py && python3 los_test.py && python3 pid_test.py

# ===================== ENV DÜĞMELERİ ===========================
#   YILDIZ_MOUNT=0 (model.sdf ile AYNI olmalı — montaj_ayarla.py garanti eder)
#   YILDIZ_PITCH_TRIM=-2.5   YILDIZ_BACK=25   YILDIZ_DOWN=6 (0° montajda ELLE)
#   [GİMBAL DALI GÜNCELLEMESİ: YENİ DÜĞME **YILDIZ_TILT** — kamera dünya
#    elevasyonu [deg, + = yukarı], boş = atan(DOWN/BACK) türetilir. Dikey
#    ekseni ARTIK BU belirler. YILDIZ_MOUNT ve YILDIZ_PITCH_TRIM'in dikey
#    kanaldaki işi bitti (yalnız dondurulmuş gövdeye-sabit kollar için export
#    ediliyor). Tek kaynak: scripts/standoff_geom.sh]
#   YILDIZ_AIM: boş bırak = türetilir (aim=-atan(down/back), AimTrim ±6°); senaryo.sh 0 sabitler
#   YILDIZ_GUI=1 estimator penceresi | YILDIZ_VIDEO=1 | YILDIZ_VIDEO_ETIKET=mpc_duz
#   YILDIZ_DRONES=1..5 (+YILDIZ_WORLD=worlds/suru.world) | YILDIZ_TARGET_PLAN
#   YILDIZ_CV_THREADS=1 | YILDIZ_STREAMRATE=20 | YILDIZ_TARGET_STREAMRATE=15 | YILDIZ_BBOX=0 (dedektörsüz)
#   --- devir kapıları (bbox_to_redis.py) — YENİ ---
#   YILDIZ_COV_GECIS=2.0     geçiş için min yatay kapsama [%]  (~40 m)
#   YILDIZ_COV_KAL=0.3       görüntülüde kalma eşiği [%]
#   YILDIZ_GECIS_MENZIL=60   geçiş için azami estimator menzili [m]
#   YILDIZ_GECIS_ALAN_PCT=0  ALAN ölçütü (0=kapalı, eski davranış). p verilirse
#     geçiş eşiği kadrajın (p% x p%) dikdörtgeninin alanı. ÖLÇÜLEN menzil
#     karşılığı (bbox_alan ~ 4.65e5/r^2, n=1945 gerçek kare):
#       p=2 -> 369 px^2 -> 35 m | p=3 -> 829 -> 24 m | p=4 -> 1475 -> 18 m
#       p=5 -> 2304 px^2 -> 14 m | p=6 -> 3318 -> 12 m | p=8 -> 5898 -> 9 m
#     Menzil karşılığı karşılaşma tipinden BAĞIMSIZ (kuyruk 14.3/çapraz 13.8/
#     kafa-kafaya 14.1 m). Alan iki ekseni birden görür; yatay kapsama tek
#     eksen ölçer ve hedefin en/boy oranı (ortanca 2.33) ile kayar.
```

## Montaj/geometri değişince DOĞRULAMA listesi

1. `python3 yildizlar_gimbal.py --test` → "TÜM STATİK TESTLER GEÇTİ".
2. Bir elips koşusu → `ozet.txt` GEOMETRI: "hedef - eksen" küçük olmalı; tespit %80+.
3. `<15 m` bandında tespit %0'dan yükseldi mi (eski 30°'nin endgame körlüğü) — görüntülü CSV.
4. Çakılma/yer teması 0 (min irtifa + vibe bağlamı).
5. `bbox.log`'da "MOD DEGISTI" var mı (konumlu→görüntülü devir tetikleniyor mu).

## Güncel durum

- **Montaj 0°**, standoff **back 25 / down 6** (LOS +13.5° → çarpmada 0°, ikisi de kadraj içinde).
  Gerçekte pitch-servo **gimbal** kullanılacak; sim'de kopter eğilmediği için 0° ≈ ideal gimbal.
  > **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05]** "Sim'de kopter eğilmediği için
  > 0° ≈ ideal gimbal" cümlesi ÇÜRÜDÜ — iki kere. (a) Kopter eğiliyor: bu
  > notların kendi ölçümleri gövde pitch'ini %5 −42.5° / %95 +39.4° gösteriyor,
  > yani 0° sabit kamera ideal gimbal DEĞİLDİ; "0 derece sabit kamera hedefi
  > göremiyor, MPC kör kalır" tespiti (bkz. DEVAM.md) bunun sonucuydu.
  > (b) Artık **gerçek** bir gimbal var: sim'deki 5 drone'un kamerası kendini
  > stabilize eden fiziksel tek eksen tilt gimbalinde. Uçuşta ölçüldü: gövde
  > −35.4…+35.2° savrulurken kamera dünya pitch'i **max |0.65°|**. Yani artık
  > emülasyon değil, gerçeğin kendisi. Dikey eksen komut edilebilir:
  > `YILDIZ_TILT = atan(down/back)`. Ayrıntı: `NOTLAR_GIMBAL.md`.
- **Kazanan yöntem MPC**; LOS ve PID donduruldu (kod kıyas için duruyor).
- Konumluda **yaw-lock açık**; devir üç kapılı: ~1.5 s kadraj + kapsama ≥%2 + estimator menzili ≤60 m.
- back **25 KANITLANMIŞ** — back 40 denemesi kapanışı öldürdü (alan 12513→1800, min menzil 7→19 m).
