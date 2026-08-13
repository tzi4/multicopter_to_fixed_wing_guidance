# AUTORESEARCH DURUM — tek LOS/PN donanım adayı (2026-08-11 kapanışı)

> **Bu dosya devir belgesidir.** Yeni bir ajan/oturum buradan devralabilir.
> Deney backlog'u ve tek tek madde kayıtları: `TO_TEST.md` (madde 19-27 bu
> kampanyanın). Koşu prosedürü: `NOTLAR_MPC.md`. Bu dosya ONLARIN ÜSTÜNDE
> bir özet + öncelik + tuzak listesidir.

## 0. 2026-08-11 GÜNCEL MİMARİ KARARI

Güncel ana yön **konumlu arka-slot yaklaşımı + yalnız görüntüyle devir + tek
LOS/PN görüntülü yasa**dır. MPC silinmedi, fakat artık varsayılan donanım
yolunda veya görsel fazda çalışmaz; yalnız tarihsel A/B/fallback olarak
`--gudum mpc` ile kalır.

Devir kuralı: art arda **5 taze bbox**, her biri 1280×720 kadrajda en az
`%3×%3 = 829 px²`. Merkez ve hedef telemetrisi geçiş kapısında aranmaz.
YOLO kullanıldığında `--yolo-conf` filtresi kutuyu Redis'e yayınlamadan önce
çalıştığı için sayaç yalnız detectorün kabul ettiği kareleri görür.

RoboFly/elips canlı sonuçları:

| kol | angajman | gerçek CPA medyan / p90 | `<5 m` | `vurus_basarili` |
|---|---:|---:|---:|---:|
| LOS `N=4`, alan `%3`, 5 büyük kare | 6 | **1.09 / 2.03 m** | **6/6** | **2/6** |
| LOS `N=5`, aynı kapı | 12 | 1.65 / 3.76 m | 11/12 | 0/12 |
| önceki saf LOS, kapısız/erken devir | 19 | 1.95 / 39.13 m | 15/19 | 0/19 |

N=4 iki bağımsız başlangıçta temas üretip aracı çökertti (`vibe=369.5,
r=0.87 m` ve `vibe=95.2, r=0.52 m`). Temas sonrası başlayan satırlar
performans havuzundan çıkarıldı. N=5 kolundaki tek ağır ıska `8.94 m`dir.
Dolayısıyla varsayılanlar `N=4`, `vur_ivme=4 m/s²`, `buyuk_kare=5`,
`alan_pct=3` olarak donduruldu.

Donanımda çalışmış `yarışma/gimbal_bench_takip.py`, `mavlink_tilt.py` ve
`mpc_komut_izle.py` referans alındı. IMX500 1280×720/180° dönüş, gerçek HSV
bandı, moment merkezi, servo rampası ve `DO_MOUNT_CONTROL` ana araçlara
taşındı. Salt gözlemci artık `--yasa los` ile gerçek LOS çıktısını aracı
sürmeden çalıştırabiliyor. Güncel tek-satır komutlar:
`NOTLAR_LOS_DONANIM.md`.

Önemli sınır: **geçiş** telemetri menzilinden bağımsızdır; mevcut LOS/PN ise
PN kazancı, `t_go`, dikey çözüm ve ıska bırakma için hâlâ `menzil_m` kullanır.
Tam telemetrisiz terminal uçuş için bbox ölçeği/optik büyüme veya bağımsız
mesafe sensörü füzyonu sıradaki araştırmadır. Yanal hız isteğini ArduPilot
zaten roll/pitch'e çevirir; yaw-rate yalnız yatay görüşü tutar ve yeni
`--no-yaw` koluyla ayrı sınanabilir, fakat temas üreten varsayılanda açıktır.

## 0b. 2026-08-10 TARİHSEL HİBRİT KARARI

Canlı elips A/B'si sonucunda ana yön **konumlu yaklaşım + orta-faz MPC +
terminal PN/LOS hibriti** olarak donduruldu:

- Konumlu `simple_guided_follow.py` hedefin arkasındaki slotu kurar ve görsel
  devir şartlarını oluşturur.
- Görüntülü devirden sonra `hibrit_gudum.py`, `R>18 m` iken MPC'yi FOV ve
  kısıt planlayıcısı olarak tutar; `R<=18 m` iken `terminal_los_gudum.py`
  PN/LOS terminal yasasına geçer.
- Her iki çıkış gerçek hızdan ivme/yön bakımından ulaşılabilir komuta
  izdüşürülür. Dikey hız ayrıca jerk ve mutlak hız sınırındadır.

Saf MPC canlı tabanda CPA ortanca 11.20 m ve 59 geçişin yalnız 3'ünde `<5 m`
üretirken güvenli hibrit üç geçişte 3.22/3.42/4.00 m üretti. `R<8 m` komut
terslemesi 171.6° ortancadan 5.35°'ye, `>90°` oranı %91.4'ten 0/103'e indi.
Fakat henüz fiziksel temas yoktur; güvenli hibritte üç `ALTITUDE ABORT`
kaldı. Bu nedenle sıradaki amaç "daha yakın tahmin" değil, emniyetli ve
tekrarlanabilir gerçek temastır. Ayrıntı: `TO_TEST.md`, **CANLI T1 HÜKMÜ**.

## 1. TARİHSEL SAF-MPC ŞAMPİYONU

```bash
SURE=360 YENIDEN_BASLAT=0 GORUNTULU="mpc_gudum.py" \
PLAN=missions/hedef_elips.plan METOT=<etiket> YILDIZ_VIDEO=1 \
YILDIZ_TUT_YAW=1 YILDIZ_EYLEYICI=1 YILDIZ_DIKEY_HATA=1 YILDIZ_DIKEY_TGO=2 \
YILDIZ_KOR_PN=1 tools/senaryo.sh
```

2026-08-10'dan beri yukarıdaki beş kanıtlı kol (`TUT_YAW`, `EYLEYICI`,
`DIKEY_HATA=1`, `DIKEY_TGO=2`, `KOR_PN`) **çıplak çalıştırmada varsayılan
açık**; komutta açıkça yazılmaları yalnız deney kaydını okunur tutar. Her biri
ilgili `YILDIZ_*=0` override'ıyla ayrı ayrı kapatılabilir.

| bayrak | ne yapar | kanıt |
|---|---|---|
| `YILDIZ_TUT_YAW=1` | tespit `tut`tayken yaw sönümlü sürer | dönüşte kapanma +2.66→+3.59, açılan %40→%18.5 |
| `YILDIZ_EYLEYICI=1` | yatay eyleyici doyum modeli (tau_lin 1.7 s, a_max 4) | ulaşılamaz komut %40→%11-15, maliyet +%3.5 |
| `YILDIZ_DIKEY_HATA=1` + `YILDIZ_DIKEY_TGO=2` | P + t_go-şekilli dikey kol (**dikkat: env değeri k DEĞİL, k'ye ÇARPAN** → 2 verirsen k=4) | k=4 kolu günün en iyi temas koşusunu verdi (minR p50 5.34) |
| `YILDIZ_KOR_PN=1` | kör karelerde komutu değil YASAYI sürdür (ölü hesap + çözücü koşar) | kör karelerin %100'ünde çözücü koşuyor; ortak katmanda CPA p50 5.99→3.50 |

Varsayılan yapılanlar (bayrak gerekmez): tablodaki beş kanıtlı kol; kör-PN
kelepçesi 26°, menzil kapısı 12 m, HOLD irtifa kapısı ve
`RECOVERY_HOLD_MAX_S`=8 s.

## 2. BU KAMPANYADA DÜZELTİLENLER

| # | düzeltme | kanıt |
|---|---|---|
| 1 | **tut-yaw sürdürme** — dönüşteki ana kaçırma döngüsü kırıldı | dönüş karelerinin %40'ında yaw komutu hiç gitmiyordu |
| 2 | **Doyumlu eyleyici (yatay)** — model gerçeğe uyduruldu | ölçüldü: yatay tau 1.7 s / plato 4 m/s²; model 1.0 s + sınırsız sanıyordu |
| 3 | **KOR_PN + R3′ kelepçe (26°) + menzil kapısı** | ham sürüm dönüşte regresyon yapıyordu (kadraj dışına hayali hedef); kelepçe kapattı |
| 4 | **HOLD irtifa kapısı** (takla patolojisi) | takla 3 koşu → 0; alt_err p95 56→23, >15 m oranı %86-91→%15 |
| 5 | **Üç sessiz-ayrışma kapısı** | `plan_uyum` (iki "düz" koşu aslında elips uçmuştu), `tilt_uyum` (9° zincir ayrışması), prearm sağlık biti |
| 6 | **Operasyonel** | BRAKE shutdown (POSHOLD fırıldak-çakılması), EEPROM temizliği (gyro-cal), ölçülmüş-agresif parametre profili |

**Reddedilenler (hepsi uçuşla):** APN türev terimi (gürültü), dikey doyum
(abort ↑), dikey rampa (no-op — devir ~25 m'de rampa zaten doymuş), ilerleme
saati (kazanç 0.00 m, iki senaryoda), DOWN=0 (dz<0 kırılmadı), kesişme
yaklaşımı (görüntülü faz aşımı toparlayamıyor).
**Engellenen zarar:** `MOT_THST_HOVER=0.68` assert'i (gerçek askı gazı
0.28-0.35; ileri beslemeyi 2× şişirirdi).

## 3. AÇIK PROBLEMLER — ÖNCELİK SIRASI

> **2026-08-10 canlı T0 bu sırayı değiştirdi.** Ayrıntılı kanıt ve deney
> tanımları `TO_TEST.md` madde 28–33. Aşağıdaki sıra önceki P0-dikey hükmünün
> yerini alır.

### P0 — Güvenli, tekrarlanabilir fiziksel temas
Terminal hız terslemesi hibrit kolun ortak ulaşılabilirlik katmanıyla
kapatıldı. Yeni darboğaz, 3.22–4.00 m gerçek CPA'yı titreşim/temasla
doğrulanmış çarpmaya çevirmek. `N`, devir menzili ve VUR hız farkı kontrollü
faktöriyel taranacak; hakem `ref_menzil_gercek_m + vibe/vurus_basarili`.
İlk hibritin `cmd_vz=-9 m/s` ve 78.9 m irtifa sapması düzeltildi, ancak
güvenli koşuda üç `ALTITUDE ABORT` kaldığı için temas kazancı emniyet
pahasına alınmayacak.

### P1 — Arc/constant-turn hedef modeli
Sert dönüş CPA ortancası **13.36 m**, düz faz **9.96 m**; sert dönüşte `<5 m`
yok. APN ivmesi tüm canlı koşuda 0. Raw türev yeniden açılmayacak: MINRECT
işareti iki yönde doğrulanıp güven-kapılı CT/IMM yayılımı ve PN feed-forward
denenecek.

### P2 — Arka-koni kapısı ve hız programı
Sistem R<20'de zaten arka çeyrekte (yaklaşım açısı medyan 160°), fakat R<8'de
149°'ye bozularak setpoint tersliyor. `YERLEŞ` (LOS hızı/yanal göreli hız
sıfırla) ve yalnız kararlı `>=165°` kuyruk konisinde `VUR` hızlanması ayrı
fazlar olacak.

### P3 — Dikey dal/kalk sönümlemesi
P+TGO CPA dikey artığını küçülttü; artık birincil ıska değil. Ancak 23 CPA'nın
20'sinde önceki 5 s içinde vel_z en az iki kez işaret değiştirdi; 8–15 m
dikey ray doluluğu %28.7. dz/dz_dot kritik sönüm + histerezis + jerk/slew A/B.

### P4 — Kalan işler
- GUI bütçe-kesme ~%90: her kabul headless tekrar ister.
- Alt-metre kuyruğu (TO_TEST.26), kurtarmada kontrolsüz yaw (TO_TEST.27) ve
  resmi temas ölçütü açık kalır.
- Doğrudan roll/attitude komutu ilk seçenek değildir; istenen yanal ivme
  ArduPilot hız/ivme katmanından geçirilerek roll üretilir.

## 4. ÖLÇÜM METODOLOJİSİ (bunlara uymayan sonuç geçersizdir)

1. **minR tek koşuda hakem DEĞİL.** Koşu-içi SD 0.91 (log) vs koşular-arası
   0.23; ICC 0.06. Kol farkı için 6-8 koşu/kol gerekir.
   **Güvenilir bitiş noktaları (1-2 koşu yeter): düz-faz taze%, zaman aşımı%,
   roll RMS.**
2. **Analiz birimi ANGAJMAN**, koşu bloktur. Kol kıyasında devir geometrisi
   dengesi (kapanma0/R0/aspekt0 medyanları) mutlaka raporlanır; dengesizse
   kıyas geçersiz.
3. **Katmanla:** `kapanma0>=3 m/s` ve `<3` ayrı okunur. Devir anı kapanma
   hızı minR'nin en güçlü öngörücüsü (Spearman −0.56) — MPC var olan
   kapanmayı korur, yaratmaz.
4. **GUI şerhi:** GUI'li koşu çözücüyü boğuyor (p95 21.8 vs headless 13.7,
   `butce_kesti` %88 vs %75). GUI'li koşuyu GUI'li tabana karşı oku.
5. **Ölçü tuzakları:** `los_hiz_az` PN artığı DEĞİL (komutlanan u2'yi
   kullanır — gerçek için `ref_kerteriz_deg` türevi); `|u3|` dikey talebi
   ölçmez (LOS üçayağı, eps ile yatay pay büyür — `cmd_vz` kullan);
   `ozet.txt` "kümülatif tespit %" koşular arası kıyaslanamaz (bbox.log
   birikir — CSV `durum` kolonunu kullan); `ref_hedef_ax/ay` BOZUK.

## 5. OPERASYONEL TUZAKLAR (hepsi bu kampanyada yandı)

| tuzak | belirti | çözüm |
|---|---|---|
| **PLAN yalnız restart'ta yüklenir** | `YENIDEN_BASLAT=0` iken PLAN sessizce etkisiz; hedef eski planı uçar, ekrana yine "düz rota" yazar | `plan_uyum` kapısı artık durduruyor; düz koşu için yığını `YILDIZ_TARGET_PLAN=... ` ile kaldır |
| **VIDEO yalnız restart'ta açılır** | `senaryo.sh` `YILDIZ_VIDEO=1`'i yalnız kendi restart kolunda export eder; elle kaldırılan yığında **video kaydedilmez** (2026-08-09 koşularının hiçbiri kaydedilmedi) | Yığını **`YILDIZ_VIDEO=1 ./yildizlar_gudum.sh --headless`** ile kaldır |
| **Parametre otoritesi** | dosya/EEPROM/MAVLink yazımı geçersiz | `guidance_config.GUIDED_STARTUP_PARAM_ASSERTS` her koşuda ezer — **gerçek otorite orası** |
| **EEPROM bozulması** | taze yığında "Gyros not calibrated", EKF3 DCM'e düşer | her tazelemede `rm run/sitl0/eeprom.bin` |
| **Arm hazırlığı** | PreArm mesaj sessizliği yanıltıcı (ArduPilot kısıtlıyor) | `SYS_STATUS` `MAV_SYS_STATUS_PREARM_CHECK` sağlık biti |
| **Tilt zinciri** | DOWN'u yalnız senaryoya vermek kamera/MPC referansını 9° ayırır | `tilt_uyum` kapısı; DOWN değişikliği yığın + senaryo ikisinde birden |
| **BIN↔CSV saat ofseti** | dataflash unix zamanı duvar saatinden 0.65-3.2 s ileride | koşu başına ofset kestir; mümkünse aynı-satır CSV kolonlarını kullan |

## 6. STANDART KOŞU PROSEDÜRÜ

```bash
# 1) yığın (headless + video + taze EEPROM)
./yildizlar_gudum.sh --stop ; sleep 3 ; rm -f run/sitl0/eeprom.bin
YILDIZ_VIDEO=1 ./yildizlar_gudum.sh --headless
#    düz rota için:  + YILDIZ_TARGET_PLAN=missions/hedef_duz.plan
#    S rotası için:  + YILDIZ_TARGET_PLAN=missions/hedef_s.plan
#    arc algısı için:+ YILDIZ_MINRECT=1

# 2) kapılar (koşu öncesi): prearm sağlık biti · tools/plan_uyum.py · tools/tilt_uyum.py

# 3) deneme (şampiyon konfigürasyon — bölüm 1)
```

**Veri yerleri:** koşu klasörleri `run/denemeler/<metot>_<rota>_<damga>/`
(ozet.txt, guidance.log, goruntulu.log, bbox.log) · CSV'ler
`guidance_allstar/logs/goruntulu_mpc_<damga>.csv` + `mpc_tani_<damga>.csv` +
`guided_follow_<damga>.csv` · videolar `videos/<etiket>_<damga>.mp4` ·
dataflash `run/sitl0/logs/*.BIN` (avcı), `run/sitl5/logs/*.BIN` (hedef).

## 7. YENİ AJAN İÇİN İLK ÜÇ İŞ

1. **Faktöriyel terminal tarama:** `N={4,5,6}` × devir
   `{18,20 m}` × tırmanma sınırı `{1.5,2.0 m/s}`. Her hücrede en az altı
   angajman; CPA, `vibe/vurus_basarili`, `ALTITUDE ABORT`, tazelik ve devir
   geometrisi birlikte raporlanır.
2. **VUR/DON son metre ayarı:** yalnız fiziksel temas artıyorsa VUR kapanma
   farkını ve DON/t_go bırakma eşiğini tara. İlk geçiş sonrası kafa kafaya
   ikinci saldırı veya dikey taşma regresyon sayılır.
3. **Constant-turn hedef modeli:** hibrit terminal tabanı sabitlendikten sonra
   MINRECT sağ/sol işaretini doğrula; güven-kapılı CT/IMM yayılımını önce
   `R>18 m` MPC orta fazında dene. Terminal PN'e ham bbox türevi verme.
