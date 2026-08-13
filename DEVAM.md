# DEVAM — bekleyen işler (güncelleme 2026-08-05 gece, HEAD a2a82a1 sonrası)

> **YAN DAL: `ardupilot-takip`** (2026-08-05 öğlen). MPC yerine ArduPilot'un
> kendi takip yasası (`mode_follow.cpp` + `AP_Follow` + `AC_PrecLand`
> kestirimi) denendi. Ayrıntı ve gerekçeler: **`ARDUPILOT_TAKIP.md`**.
> Sim sonucu (aynı ortam, aynı iskelet, aynı ıska ölçütü):
> asılı **0.5 m** (MPC 0.5 ile eşit) / sonsuz **1.2 m** (MPC tur-4 1.3) /
> elips **0.2 m** (tablodaki en iyi) / wanderer 5.5 m (MPC 4.9-6.7).
> Döngü maliyeti 43 µs (MPC çözücüsü p95 13 ms). Zayıf noktası: kadrajı
> koruyan hiçbir terimi yok — hızlı geometride `ex_rms` 11-19, yaw geriden
> geliyor. Merge kararı verilmedi; önce §7'deki PrecLand EKF'i denenmeli.

Bu dosya devir notudur: hangi iş neden bekliyor, hangi sırayla yapılmalı,
ve her birinin somut kabul ölçütü. Bağlam için önce `NOTLAR_MPC.md` (ortamı
sürme) ve `guidance_allstar/LOG_SOZLUGU.md` (logları okuma) okunmalı.

## Durum özeti

- **Kazanan yöntem MPC.** LOS ve PID dondurulmuş kıyas artifaktları.
- Testler: `cd guidance_allstar && python3 mpc_test.py && python3 los_test.py
  && python3 pid_test.py` → **67/67, 66/66, 51/51** (mpc 48 → 67:
  35 m/s turu + VURUŞ fazı testleri eklendi, bkz. madde 1).
- Sim geometrisi: **montaj 0°**, standoff **back 25 / down 4**.
  > **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05]** `gimbal` dalında "montaj" artık
  > bir ayar değil: kamera kendini stabilize eden **fiziksel tek eksen (tilt)
  > gimbalde**, SDF montaj pitch'i 0 kalır. Dikey ekseni **TILT** belirler:
  > `YILDIZ_TILT = atan(down/back)` (dünya elevasyonu, + = yukarı,
  > `scripts/standoff_geom.sh`). Uçuşta ölçüldü: gövde −35.4…+35.2°
  > savrulurken kamera dünya pitch'i max |0.65°|. Araç: `tools/tilt_ayarla.py`.
  > Ayrıntı: `NOTLAR_GIMBAL.md`.
- Hız tavanı iskelet 35 m/s; **MPC artık aynı kaynaktan türetiyor**
  (2026-08-05, madde 1 — sim koşusu bekliyor).
- Rekorlar: statik yer hedefi **0.92 m**; asılı hedef (drone_2) **0.47 m**
  (`videos/mpcasili2_elips_20260805_031421.mp4`, ama bkz. asılı hedef notu).

## TAMAMLANANLAR (2026-08-05 gece oturumu)

### ISKA iskelet bağlantısı — BİTTİ (commit 43bbef4)
Eski madde 1'in üç parçası aynen bağlandı + `birak_bekliyor` kilidi eklendi
(ıska ilanı ile `bbox_to_redis`'in `komut_yetkisi`ni çevirmesi arasındaki
1-37 ms'lik pencerede sahte yeniden devralma oluyordu; 9 gerçek devire 17
`devir_alindi`). Kabul koşusu (elips, 360 s): 9 ISKA, CPA→bırakma ortanca
**7.0 s** (eski 40-77 s), yetki segmenti **4 → 9**, 9/9 geçiş
`goruntulu_birak` anahtarından (redis monitor kanıtı). Kilit doğrulaması
hedef_sonsuz koşusunda: 8 devir = 8 segment, sıfır sahte devralma.

### hedef_sonsuz koşusu — BİTTİ, kritik bulgu
`run/denemeler/mpc_sonsuz_20260805_022808/`. Yakın bantta (<30 m)
karşılaşmaların **%100'ü kuyruk takibi** (hedef_duz'da %30.6 kafa kafayaydı
→ eski "düz rota" sonuçları saf kuyruk DEĞİLDİ, doğrulandı).
**KRİTİK:** saf kuyrukta MPC'nin kapanması SIFIR — hedef 21.05 m/s,
`MpcAyar.hiz_tavani_mps=18` → kapanma −3 m/s, 7/7 ıska "menzil açılıyor",
en yakın geçiş 21 m duvarında. hedef_duz'daki 4.66 m'lik meşhur geçişin
kafa kafaya geometriden geldiği de görüldü. **Madde 2'nin gerekçesi artık
ölçülmüş ve tek nedenli.** Ek: FOV kısıtı döngülerin %96.8'inde bağlayıcı;
çözücü p95 13.3 ms (bütçe 12.7 aşılıyor) — 35 m/s turunda birlikte bakılmalı.

### Asılı hedef kablolaması — BİTTİ (commit a2a82a1), koşu yapıldı
`HEDEF_ARAC=drone2` env anahtarı (`guidance_config.py` + `senaryo.sh`);
boşken davranış birebir eski. drone_2 portları 14662/14664, sysid 2.
Koşu: 6 geçiş, min **0.47 m**, ortanca 1.25 m; mor gövde tespiti %81.9.
**UYARILAR (videoyu izleyen bilsin):** (1) mor hedef kalkıştan önce kameranın
dibinde YERDE park ediyor → videonun ilk kareleri ekranı kaplayan mor blok
(cov %72.8) — senaryo artefaktı, arıza değil; (2) çift SITL aynı makinede
`simtime_ratio ~0.44` → video 13 fps, kamera akıcı DEĞİL; sayılar yön
gösterir, kesin metrik sayılmaz. İyileştirme fikirleri: hedefi uzağa spawn
et / kaydı kalkıştan sonra başlat / `HEDEF_ARAC=drone2` iken uçak SITL'ini
hiç başlatma (yildizlar_gudum.sh'ye anahtar ister). Ayrıca durgun hedefte
ıska çoğunlukla 15 s zaman aşımıyla ilan ediliyor; `iska_acilma_m` asılı
hedef için yeniden ölçülmeli.

### Formasyon A/B — KAPANDI (sim gerekmedi, kod okumasıyla)
Cevap: **karşılaştırılacak iki tür yok.** `formation_KILLER.py`'nin
FORMASYON fazı hedefe güdüm içermiyor (girdisi lider drone); o mimaride
hedefi kovalayan `simple_guided_follow_shaykh.py` ve güdüm yasası bizim
`simple_guided_follow.py` ile AST düzeyinde birebir aynı (26/45 sembol
özdeş, `BehindSlotGuidance` dahil). Fark yalnız taşıma katmanı
(MAVLink↔Redis) + shaykh'in `a5a28eb` dönüş-kelepçesi yamasını ALMAMIŞ
olması (yani B kolu = A'nın 678 m daire hatalı eski sürümü). Ayrıca
formation_KILLER SALDIRI yasası hedef HIZ VEKTÖRÜ türetiyor → "yalnız
menzil" kuralının ihlali; makalede atıf verilecekse not düşülmeli.
`formation_KILLER.py` çalıştırılamaz durumda da (7 eksik bağımlılık:
config.py/controller.json/waypoints.json vb. yok).

---

## 1. MPC'yi 35 m/s'e taşıma turu + VURUŞ FAZI — KOD/TEST BİTTİ, SİM BEKLİYOR

Yapıldı (yalnız güdüm tarafı; ortam/plan/param dosyalarına dokunulmadı):
- `MpcAyar.hiz_tavani_mps` artık `guidance_config.GORUNTULU_MAX_SPEED_MPS`'ten
  **türetiliyor** (`mpc_gudum.cevre_hiz_tavani`) → 35 m/s, bir daha
  ayrışamaz. Offline: saf kuyruk (hedef 21.05 m/s) **30 m → 1.6 m
  ÇARPIŞMA**, tavana değme %73-98. Aynı senaryo 18 m/s'te 30.00 m
  (hiç kapanmıyor) — hedef_sonsuz bulgusunun birebir karşılığı.
- **VURUŞ FAZI**: `KAPANMA → TERMINAL(45 m) → VURUS(22 m) → ISKA`.
  22 m'de 0, 8 m'de 1 olan sürekli karışım katsayısı FOV bantlarını
  fiziksel kenara açar (14/17.5 → 19/19; ex 26 → 31), fren/hızlanma
  bant daraltmasını söndürür, alan ödülünü 2x büyütür, `q_ivme`'yi
  0.35x kısar. Ölçülen: aynı durumda kapanma komutu **u1 27.3 → 32.3 m/s**.
  Kadraj/PN ağırlıkları bilinçli olarak 1.0 (büyütmek saf takibe itiyor,
  ölçüldü — gerekçe `MpcAyar` vurus_* bloğunda).
- `adim_s` 0.18 → 0.12 (ufuk 3.6 → 2.4 s): 35 m/s'te 3.6 s ufuk 126 m,
  angajman zarfı 60 m. Ölçüldü: 0.18'de kadraj kaybı %37, 0.12'de %27.
- `fov_tirmanma_talep_tavani_mps` 5 → 9 ve `q_ey` 0.35 → 0.60: yüksek
  tavanda kadraj kayıplarının baskın kanalı DİKEY (saf kuyrukta kayıp
  anında |ex| max 1.1 deg). İkisi birlikte kayıp %26.9 → %23.4,
  min menzil ortancası 9.63 → 8.51 m.
- ISKA eşikleri yeniden temellendirildi: `iska_gecis_arm_m` 20 → **12**
  (ayırıcı artık hız değil GEOMETRİ: gerçek geçişlerin CPA'sı ≤ 11 m,
  salınım vakasınınki 18.1 m), `gecis_kapanma_esigi_mps` 15 → **10**
  (kuyrukta ulaşılabilir kapanma 35−21.05 = 13.9 m/s, eski eşik
  matematiksel olarak ulaşılamazdı → ölçülen 0/16 ateş),
  `iska_zaman_asimi_s` 15 → **8**.
- **Tasarım kararı verildi:** ıskada "yetkiyi bırak" KORUNDU (gecişten
  sonra 0 deg sabit kamera hedefi göremiyor, MPC kör kalır — **[GİMBAL DALI
  GÜNCELLEMESİ: bu gerekçe dikeyde çürüdü; kamera artık tilt gimbalinde ve
  yukarı bakabiliyor. Geçişten sonra hedefi kaybettiren asıl şey YAW
  (gimbal tek eksen) — karar ayakta, gerekçesi yaw'a kaydı. Gimballi tur
  koşulunca "yetkiyi bırak" yeniden ölçülmeli.]**) ama
  süzülme artık **FRENLİ**: yön korunarak 3 m/s² ile 12 m/s'e iner
  (dönüş yarıçapı 245 → 29 m). "Yavaşla ve dön"ün faydası, ortak dosya
  değiştirmeden bu şekilde alındı.

### SİM TURU-1 SONUCU (hedef_sonsuz, commit bcd74f2) ve TUR-2 DÜZELTMESİ

Tur-1 kazandı: hız paritesi tam çalıştı (u1 max 35.00, cmd p95 34.78,
kapanma −18.4 m/s), VURUŞ fazı %93.8 döngüde aktif, **21 m duvarı
kırıldı** (CPA min 20.48→2.36, ortanca 21.65→5.84, 10/12 geçiş ≤10 m).
Çözücü p95 13.26 ms (değişmedi).

Tur-1 kusuru: **metre-altı vuruş 0/12** ve terminal fazın son
0.2-1.3 s'si HER geçişte KÖR. Kök neden ölçüldü: ileri ivmelenme burnu
aşağı eğiyor (5.84°/(m/s²)), SABİT 0° kamera onunla iniyor, mount 0'da
zaten eksenin ÜSTÜNDE olan hedef üst kenardan çıkıyor.
**[GİMBAL DALI GÜNCELLEMESİ 2026-08-05: bu kök neden ZİNCİRİ KIRILDI.
"Kamera burunla birlikte iniyor" halkası fiziksel tilt gimbaliyle koptu
(gövde ±35° iken kamera max 0.65°). Aşağıdaki tur-1/tur-2 sayıları
GÖVDEYE-SABİT dönemde ölçüldü; `pitch hızı` sütununun kadraja çevrimi
(°/s → px/s) gimballi kurulumda GEÇERSİZ. Üst-kenar kaybının gimballi
karşılığı tilt hatasıdır ve YENİDEN ÖLÇÜLMELİ. Ayrıca bu kök nedene karşı
alınan tur-2 önlemleri (`vurus_ivme_carpani`, devir ivme rampası, ileri
ivme kelepçesi) artık BAŞKA bir sorunu çözüyor olabilir — gimballi turda
kaldırılıp kaldırılamayacakları ölçülmeli.]**
Kayıpların
%18.5'i üst / %6.0'ı alt (3:1), bunun **%10.9'u fiziksel FOV dışında**
(beta < −20.07) — bant açmak o kısmı kurtaramaz. Yan etki: |pitch hızı|
ortanca 3.6→13.3 °/s (kadrajda ~950 px/s = kullanıcının şikâyet ettiği
titreme), kadraj kaybı %8.6→%48.3, devir transiyenti ilk 3 s'de
pitch −20.5°, taze-değil döngü 2→261.

Tur-2 düzeltmesi (yalnız MPC tarafı, offline ölçüldü — düz rota,
hedef 21.05 m/s, 3 devir × 2 tohum):

| kol | pitch hızı | üst kenar % | fiziksel FOV dışı % | min menzil | cmd p95 |
|---|---|---|---|---|---|
| tur-1 (`vurus_ivme` 0.35) | 3.81 | 12.0 | 14.0 | 1.93 | 35.0 |
| **tur-2 (1.0 + devir rampası)** | **1.83** | **10.5** | **12.4** | **2.13** | **35.0** |

- `vurus_ivme_carpani` **0.35 → 1.0**: VURUŞ artık ivme cezasını
  kısmıyor. Hız paritesi tek başına tavana değiyordu; ekstra ivme
  yetkisi bedava değil, kamera ekseniyle ödeniyordu. (0.7 ara değeri
  de ölçüldü: 2.74 °/s.)
- **Devir ivme rampası** (yeni): `q_ivme` devirde ×3.0, τ=2 s ile
  1.0'e sönümlenir. 5.0/2.5 s denendi → min menzil 12.27 m (kapanmayı
  öldürüyor); 2.0/1.5 s → kazanç yok.
- **Hız-artış kelepçesi ölçümle ELENDİ** (`ileri_ivme_tavani_mps2`
  varsayılan 0.0): pitch'i düzeltmedi, kapanmayı öldürdü
  (2 m/s² → min menzil 1.93→11.17 m). Sebep: ArduPilot hız döngüsü
  setpoint farkını ~0.25 s'de kapatmaya çalışır, 1.25 m/s'lik fark
  bile ivmeyi 5 m/s²'ye doyurur — komutun BÜYÜKLÜĞÜNÜ kelepçelemek
  o farkı küçültmez, küçülten şey (u−w) üzerindeki CEZAdır.
- `vurus_ey_carpani` 2.0 denendi → nötr (1.83→1.88 °/s, min menzil
  2.13→2.09). 1.0'da bırakıldı.

**Kalan açık:** üst-kenar kaybının fiziksel FOV dışı kısmı (~%12)
GEOMETRİKTİR — standoff hedefin ALTINDA (down 4-6 m), menzil
kapandıkça eps = asin(down/r) büyür. Tur-3'te SAF GÜDÜM çözümü
denendi (aşağı).

### SİM TUR-2 SONUCU ve TUR-3 DÜZELTMESİ

Tur-2 sim'de (commit 68cee27) **İLK metre-altı vuruş geldi**: CPA min
2.36 → 0.84 m, ≤1m 0/12 → 2/16 — tamamen `vurus_ivme` 0.35→1.0'dan
(derin terminal). Parite korundu (u1 max 34.96, simtime 0.995).

Tur-2 kusurları ve tur-3 kararları:
- **Devir ivme rampası GERİ ALINDI**: sim'de net zararlı — kapanmayı
  yarıya düşürdü (menzil_hızı ort −3.11 → −1.40 m/s, güçlü kapanma
  ≤−10 m/s %7.0 → %2.9), devir transiyentini de düzeltmedi
  (taze-değil 261 → 269). `devir_ivme_carpani`/`_ivme_carpani`/
  `_yetki_t0` rampası kaldırıldı.
- **Titreme (b) ile hedeflendi, (c) korundu**: pitch hızı sim'de
  13.32 → 14.12 (offline −%52 iddiası çürüdü — ivme-kenetli, `vurus_ivme`
  1.0 gövde ivmesini yükseltiyor). Kullanıcı "titremeyi salla, çarpış
  önce" dedi; `vurus_ivme` 1.0 KALDI.
- **TERMINAL DİKEY HİZALAMA** (yeni, asıl fikir — `vurus_hiza_*`):
  VURUŞ'ta hedef görünen yükselişi (eps) RAHAT bandını (10°) aşınca
  dikey LOS-hızı referansına tırmanma biası eklenir → avcı hedef
  hattına tırmanıp standoff'u eritir, kamera düzleşir, hedef üst
  kenardan çıkmaz. Saf geometri (yalnız eps=f(ey), telemetri yok),
  tek yanlı (alçalma zorlanmaz), deadzone'lu (eps≤10° hiçbir şey
  yapmaz → kapanma bedeli yok).

**ÖNEMLİ — OFFLINE/SİM GAP**: bu mekanizma OFFLINE KAPALI DÖNGÜDE
ÖLÇÜLEMEZ. Ölçtüm: benzetim terminale **eps<0 ile giriyor** (hedef
zaten düzleşmiş: −1.6…−5.0°), çünkü offline motorun dikey dinamiği
sim'inkiyle aynı değil — sim'de avcı standoff'ta ~4m altta KALIYOR
(üst-kenar kaybı oradan), offline'da terminale kadar hizaya tırmanıyor.
Yani mekanizma tüm offline senaryolarda deadzone içinde kalıyor
(bias 0, regresyon riski YOK) ve **çözücü seviyesinde deterministik**
test edildi (5p): eps=19°'de KAPALI kol descend ederken (u3 +0.14,
üst kenarı kötüleştirir) AÇIK kol tırmanıyor (u3 −0.46). **Kapalı
döngü doğrulaması SİM'e ait.** Bu, tur-2'de senin işaret ettiğin
offline→sim ayrışmasının ta kendisi.

### SİM TUR-3 SONUCU ve TUR-4 DÜZELTMESİ

**GERÇEK ÇARPMA OLDU.** CPA 0.62 m (senaryo rekoru; tur-2 0.84, tur-1
2.36). Kanıt fiziksel: 2.32 m'de vibe 2.0→17.4, 1.04 m'de 25.5, ardından
takla (roll −106°, pitch −51.8°) — o anda güdüm kör süzülmedeydi
(u1=u3=0), yani takla komut değil TEMAS. Tur-2'de 0.85 m geçişte vibe
yalnızca 3.3 idi (değmemiştik).

Dikey hizalama kapalı döngüde doğrulandı: `hiza_ref` 39 kare aktif,
hepsi pozitif (tek-yanlılık tuttu), u3 +1.53 → −3.72; **CPA'da dikey
ayrım 0.81 m → 0.05 m** (standoff gerçekten eridi). Kadrajı geç bıraktı
(son-taze menzil 22.3 → 8.1 m, kör süre 2.72 → 0.85 s), titreme düştü
(pitch hızı p50 14.12 → 10.93), alt-kenar BOZULMADI (39 aktif karenin
0'ı), kapanma bedeli YOK (aktif u1 20.31 vs pasif 17.92).

Vurulmayan hedef: üst-kenar fiziksel-dışı %16.26 (tur-2 %16.24) — hiç
düşmedi, çünkü mekanizma terminal karelerin yalnız **%8.1**'inde
açılıyordu (VURUŞ'ta eps p50 4.6°, p95 13.6°; 10° deadzone karelerin
~%90'ını dışarıda bırakıyor).

Tur-4 kararları:
- **`vurus_hiza_rahat_deg` 10 → 5** (TEK knob). Aktif pencere ~%8 →
  ~%45. `tavan`/`tau` bilerek DEĞİŞMEDİ: solver taraması bu bantta
  tavanın bağlayıcı olmadığını gösterdi (ref = (eps−5)/1.5 ≤ 5.7 dps,
  yani 6/8/10 aynı çözüm) ve tau 2.0'ın u1 tasarrufuna sim'de ihtiyaç
  yok (bedel zaten sıfır ölçüldü). Tek knob = sonraki koşu yorumlanabilir.
- **VURUŞ BAŞARISI TESPİTİ** (ölçüm sorunu): `vibe > 15` VE **ölçülen**
  menzil `< 3 m` → `vurus_basarili` olayı + `vuruldu` latch kolonu.
  Artık vuruş sayısı koşu özetinden doğrudan okunur; CPA tahminine ve
  "her vuruş koşuyu bitirdiği için n=5" kısıtına gerek yok.

**SİM KOŞUSUNDA BAKILACAKLAR** (kolon → hipotez):
1. `cmd_hiz_mps` / `kelepce_hiz` → tavana gerçekten değiyor muyuz
   (hedef_sonsuz'da %0 idi). Değmiyorsa `u1`, `vz_*_cbf` ve `cmd_vz`
   ile dikey tavanın mı bağladığına bak.
2. `mpc_tani.durum` + yeni `vurus` kolonu → VURUŞ fazına giriliyor mu,
   karışım 1'e ulaşıyor mu (yani 8 m'nin içine giriliyor mu).
3. `beta` vs yeni `bant_ust` → kayıp hangi kenardan. Hipotez: üst
   kenar (ileri ivmelenme burnu aşağı eğiyor).
4. `sure_ms=0` satırları → kör süzülme; VURUŞ'ta ve < 1 s ise tasarım.
5. `_olay.csv` iska satırları + `menzil_hizi` → yeni 12 m / 10 m/s
   geçiş kolu kuyruk takibinde gerçekten ateşliyor mu (eskiden 0/16).
6. **`pitch_deg` türevi** (ardışık satır farkı / `dt`): tur-1'de
   ortanca 13.3 °/s idi. Hedef < 7 °/s. Bu doğrudan görsel titreme
   ölçüsü (1 °/s ≈ 17 px/s kadraj kayması).
7. **`mpc_tani.ivme_carp`** (yeni kolon): devirde 3.0 olmalı, ~4 s'de
   1.0'a inmeli. İnmiyorsa rampa takılmış; hep 1.0 ise rampa hiç
   çalışmamış (`_yetki_t0` tohumlanmamış olabilir).
8. Devir anındaki ilk 3 s: `kadraj_kenar_px`, `pitch_deg` min,
   `durum=taze` oranı. Tur-1'de taze-değil döngü 2 → 261, pitch min
   −20.5°. Rampa çalışıyorsa pitch min ~−12° civarına çıkmalı.
9. Üst/alt kenar kayıp oranı ve `beta < −20.07` yüzdesi: tur-2'de
   %15.8/... ve %10.9 (6:1). Terminal dikey hizalama çalışıyorsa üst
   payı düşmeli VE alt-kenar kaybı artmamalı (deadzone + tek-yanlı
   onu engellemeli — ama SİM'de doğrula, offline test edemedim).
10. **`mpc_tani.hiza_ref`** (yeni, tur-3): VURUŞ terminalinde `eps>10°`
    iken pozitif olmalı (0-8 dps). **Hep 0 kalıyorsa** avcı zaten
    hizada (offline'daki gibi) demektir — o zaman üst-kenar kaybı
    hizalama dışı bir sebepten (saf pitch titremesi) ve mekanizma
    boşa çalışıyor; `pitch_deg` min ile birlikte oku.
11. Terminal dikey hizalamanın kapanma bedeli: `hiza_ref` yükselen
    karelerde `u1`/`cmd_hiz_mps` düşüyor mu? Tur-3'te bedel SIFIR
    ölçüldü (aktif 20.31 vs pasif 17.92). Bozulursa
    `vurus_hiza_rahat_deg`'i yükselt ya da `vurus_hiza_tavan_dps`'i düşür.
12. **TUR-4 ANA KABUL ÖLÇÜTÜ**: `hiza_ref > 0` kare oranı %8 → **~%45**
    olmalı (deadzone 10→5). Buna bağlı: üst-kenar fiziksel-dışı
    %16.3 → **<%12** VE alt-kenar fiziksel-dışı **<%4.5** kalmalı.
    Alt-kenar artarsa deadzone çok daraldı → `rahat`ı 6-7'ye çek.
13. **`vurus_basarili` olayı** (`_olay.csv`) + `mpc_tani.vuruldu`
    kolonu: vuruş sayısı = `vuruldu` 0→1 geçiş sayısı = olay satırı
    sayısı. İkisi tutmuyorsa latch/reset hatası var. `vibe` kolonuyla
    birlikte oku: temas 17-26 bandında olmalı; **>150 ve irtifa~0 ise
    YER teması**, vuruş değil (menzil kapısı elemiş olmalı).

## 2. Küçük açık maddeler

- `bbox.log` zaman damgasız — `HEDEF merkez=` satırına unix damga
  (ortak dosya, **kullanıcı onayı bekliyor**).
- `mpc_tani` CSV'sine `t_unix`: bir kez eklendi, test edildi (48/48),
  kullanıcı isteğiyle GERİ ALINDI (2026-08-05). Tekrar istenirse yapılacak
  değişiklik biliniyor: `_tani_yaz`'da başlık+satır sonuna `time.time()`.
- `mpc_test`'te `YILDIZ_DOWN` varsayılanı 6.0, `standoff_geom.sh` 4
  türetiyor. `YILDIZ_DOWN=4`te 5c ve 5i düşüyor (temiz tabanda da) →
  standoff değişikliğine ait, yeniden temellendirilmeli. 5c anlamlı:
  down=4'te sert FOV kısıtı yumuşağa üstünlüğünü kaybediyor.

---

## Çalışma kuralları (ajanlara verilirken tekrarlanmalı)

1. **Hedef telemetrisinden yalnız MENZİL kullanılır.** Hedef hızı/yönü/
   ivmesi türetmek YASAK. Loglamak serbest ama `ref_` önekiyle ve komut
   gönderildikten SONRA.
2. **Sim koşarken ortak dosyalara dokunulmaz.**
3. **Aynı anda tek simülasyon**; koşuları orkestratör sıralar.
4. `senaryo.sh` `temizle()` `python3 *_gudum.py` desenli süreçleri öldürür;
   sarmalayıcı kabuğun komut satırında bu desen GEÇMEMELİ (exit 144).
5. Montaj açısı tek düğme değil: `tools/montaj_ayarla.py` kullan.
   **[GİMBAL DALI GÜNCELLEMESİ: "montaj açısı" diye bir düğme kalmadı.
   Dikey geometri için `tools/tilt_ayarla.py --down .. --back .. --uygula`
   kullan; tek dosyaya (`scripts/standoff_geom.sh`) yazar. SDF'teki cam
   sensor pose pitch'i 0 KALMALI — oraya açı yazmak gimbalin komutladığı
   elevasyonun üstüne SESSİZ ofset bindirir. `montaj_ayarla.py`'nin yazma
   yolu kapalı, yalnız `--goster` çalışır.]**
6. **Koşu sırasında MAVLink portlarına ikinci istemci BAĞLANMAZ** —
   `suru_komut.py durum` bir koşuyu öldürdü (14561'e ikinci bind ARM
   ACK'ini çaldı). Durum sorgusu ancak koşu bittikten sonra.
7. Ajan seçimi: yalnız Opus ya da orkestratörün kendisi (kullanıcı
   tercihi, 2026-08-05).
