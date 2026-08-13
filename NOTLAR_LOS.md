# NOTLAR_LOS — güncel LOS/PN karar ve kullanım belgesi

Son güncelleme: 2026-08-11

Bu belge güncel otoritedir. `NOTLAR_MPC.md` MPC kampanyasının tarihini,
`NOTLAR_LOS_DONANIM.md` ise ilk donanım taslağını korur; aralarında çelişki
olursa bu dosya ve `LAST_TO_DO.md` esas alınır.

## 1. Kısa karar

Güncel araştırma tabanı:

`konumlu arka-slot yaklaşımı → 5 büyük ve taze bbox → tek TerminalLosKontrolcu`

- Görüntülü fazın varsayılan yasası `guidance_allstar/terminal_los_gudum.py`
  içindeki LOS/PN'dir; `N=4`, ileri VUR ivmesi `4 m/s²`.
- Simülasyonda kullanılacak doğrudan giriş `guidance_allstar/terminal_los_gudum.py`,
  gerçek donanımdaki sarmalayıcı `donanim/gudum_tek_dugum.py --gudum los`tur.
- MPC silinmedi. `--gudum mpc` ile A/B ve çevrimdışı araştırma kolu olarak
  durur; bugünkü terminal uçuş varsayılanı değildir.
- Araç doğrudan roll komutu almıyor. LOS yanal hız/ivme isteği üretir;
  ArduPilot bunu roll/pitch'e çevirir. Yaw-rate yalnız hedefi yatay FOV'da
  tutan yardımcı kanaldır.
- Mimari yön kesinleşmiştir, fakat gerçek uçuş stacki henüz hazır değildir.
  Kamera yönü, tek-komut-yazarı interlock'u, zaman eşleme ve hareketli gimbal
  açısı `LAST_TO_DO.md` P0 maddeleri bitmeden pervaneli terminal deneme
  yapılmamalıdır.

### 2026-08-11 gerçek donanım sonucu

- Pi üretim girişi mevcut donanım uyarlamaları korunmuş
  `donanim/goruntulu_gudum.py --gudum los` dosyasıdır; `MPC` yalnız açıkça
  `--gudum mpc` seçilirse yüklenir.
- MAVProxy `--streamrate=20` zorunludur. Varsayılan 4 Hz, dinamik sanal
  gimbal için 125–237 ms bayat ATTITUDE üretmiştir.
- ArduPilot servo-mount `GIMBAL_DEVICE_ATTITUDE_STATUS` pitch'i bu kurulumda
  gövdeye göre eklem açısıdır; DO_MOUNT_CONTROL hedefi ise dünya
  elevasyonudur. Üretim köprüsü taze status'u doğrudan eklem açısı olarak,
  yalnız status bayatsa komuttan türetilen açı olarak kullanır.
- Balon denemesinde `--menzil-kaynak telemetri` hedef GLOBAL_POSITION_INT'ten
  yalnız skaler 3B menzil çıkarır. Yarışma kolu `--menzil-kaynak estimator`
  olarak kalır; iki kolda da hedef yönü görüntüden gelir.
- Canlıda sabit menzil reddedilir; taze LOCAL_POSITION, GUIDED heartbeat ve
  gerçek menzil olmadan angajman/setpoint yoktur. Komut örnekleri ve ağ
  topolojisi `donanim/BALON_TESTI.md` içindedir.

## 2. LOS'a sıfırdan mı başladık?

Hayır. Yeni olan bölüm, terminal optimizasyonu kaldırıp doğrudan analitik
LOS/PN ivmesi üretmemizdir. MPC kampanyasında öğrenilen fiziksel ve emniyet
katmanlarının çoğu aynen korunmuştur.

| Korunan fikir | Güncel LOS'taki karşılığı |
|---|---|
| Konumlu güdümden sıcak başlangıç | Devir anındaki hız ve menzil ile `tohumla()` |
| Ataletsel LOS-rate | `q_az = d(ex) + yaw_rate` üzerinden PN |
| Ulaşılabilir komut | Mevcut hız çevresinde ivme zarfı, ±45° yön konisi ve hız tavanı |
| Terminalde ters komutu önleme | `R≤3 m` veya `t_go≤0,25 s` iken `DON` latch'i |
| Dikey emniyet | Histerezis, düşey hız tavanı ve düşey jerk sınırı |
| İlk geçişten sonra tekrar kafa-kafaya saldırmama | `ISKA/BIRAK`, sonra arka-slotu yeniden konumlu katman kurar |
| Ortak çalışma emniyeti | Taze/tut/süz/bırak merdiveni, komut LPF'si, yaw slew ve irtifa tabanı |

Korunmayanlar; çok terimli MPC maliyeti, 2,4 saniyelik sabit optimizasyon
ufku, çevrimiçi çözücü ve ufuk boyunca hatalı sabit-hedef yayılımıdır. Bu
nedenle bugünkü yapı “MPC'den vazgeçip eski LOS'u geri getirmek” değil,
MPC'nin işe yarayan kısıtlarını analitik bir nominal yasaya indirmektir.

## 3. MPC neden terminal varsayılanı olmaktan çıktı?

Karar estetik değil, ölçüme dayanıyor:

- Uzun saf-MPC tabanında gerçek CPA medyanı yaklaşık `11,20 m`; 59
  angajmanın yalnız 3'ü `<5 m` oldu.
- `R<8 m` kapanışlarında gerçek `t_go` medyanı `1,54 s` iken MPC ufku
  `20×0,12=2,4 s` idi. Maliyet çarpışmadan sonraki fiziksel olmayan kısmı da
  optimize etti ve hız komutu mevcut hıza medyanda `171,6°` ters döndü.
- MPC zaten görüntü hatası, LOS üçayağı, LOS-rate ve menzil kullanıyordu.
  Dolayısıyla yalnızca “MPC'ye LOS modeli verelim” demek temel problemi
  çözmüyor; kötü ufuk, yanlış eyleyici cevabı ve bilinmeyen hedef manevrası
  yine kalıyor.
- Ölçülen yatay araç cevabı yaklaşık `4 m/s²` plato ve `1,7 s` gecikme
  rejimindeydi; eski MPC bunu belirgin biçimde iyimser modelliyordu.
- Hedef ivmesini türevle besleyen APN denemesi gürültü çıktı: düz fazda bile
  `%70–73` yanlış aktivasyon, doğru işaret yalnız `%38–43`.
- GUI'li canlı çalışmada çözücü bütçesinin kesildiği kareler yaklaşık `%90`a
  ulaştı. Bu tek başına kök neden değil, fakat donanım hata yüzeyini büyütüyor.

MPC'den tamamen vazgeçilmedi. En değerli gelecek kolu, LOS'un her döngüde
nominal komutu üretmeye devam ettiği ve küçük bir constraint/reference
governor'ın aynı anda yalnız ivme, jerk, bank/FOV ve hız sınırına izdüşüm
yaptığı yapıdır. Bu, “önce MPC sonra LOS” şeklinde iki yasa geçişi değildir.

## 4. Güncel algoritma

`TerminalLosKontrolcu` üç fazlıdır:

1. `YERLES`: LOS-rate ve yatay görüntü hatasını azaltır; ileri ivme düşüktür.
2. `VUR`: LOS yeterince kararlı ve menzil kapanıyorsa arkadan hızlanır.
3. `DON`: kalan süre eyleyici cevabından kısa olduğunda son ulaşılabilir
   çarpışma komutunu dondurur; post-CPA terslemesi yapmaz.

Güncel ana değerler:

| Parametre | Değer |
|---|---:|
| PN katsayısı `N` | 4 |
| PN kapanma tabanı | 10 m/s |
| Kod içi yatay ivme tavanı | 5 m/s² |
| VUR ileri ivmesi | 4 m/s² |
| Komut ufku | 0,70 s |
| Komut yön konisi | ±45° |
| Görüntülü hız tavanı | 35 m/s |
| VUR'dan çıkış LOS-rate | 11°/s |
| DON | `R≤3 m` veya `t_go≤0,25 s` |
| Yaw dead-zone | 0,7° |
| Görüntülü devir | 5 ardışık taze bbox, her biri ≥829 px² |

İki önemli ayrım:

- `35 m/s` bir talep değil, tavan; fakat `IRL_Tests` içindeki mevcut gerçek
  uçuşlarda görülen tepe hızlar yaklaşık `24–27,3 m/s`dir. İlk gerçek LOS
  uçuşu bu kanıtlı zarfın altında başlamalıdır.
- Geçiş telemetri menzilinden bağımsızdır; mevcut terminal yasa ise PN
  ölçeği, `t_go`, düşey çözüm ve ıska bırakma için hâlâ `menzil_m` kullanır.
  Dolayısıyla geçiş robustlaşmış olsa da terminal henüz tamamen
  telemetrisiz değildir.

## 5. Görüntülüye geçiş

`--buyuk-kare 5 --alan-pct 3` şu an korunan kuraldır:

- 1280×720 görüntüde eşik `(1280×0,03) × (720×0,03) ≈ 829 px²`dir.
- Aynı kare iki kez sayılmaz; beş farklı ve taze tespit gerekir.
- Hedefin merkezde olması aranmaz. Devraldıktan sonra LOS yanal hatayı
  kapatır.
- Hedef telemetrisi ve yerden bildirilen menzil geçiş kapısında aranmaz.
- YOLO confidence filtresi kamera köprüsünde bbox yayınından önce çalışır;
  sayaç yalnız dedektörün kabul ettiği kutuları görür. Confidence değeri
  Redis mesajına ayrıca taşınmıyor.
- Yaklaşık süre 20 Hz kabul edilmiş dedektörde `0,25 s`, 30 FPS'te `0,17 s`dir.

Bu kural erken/uzak saf-LOS devirlerindeki ağır kuyruğu büyük ölçüde
kaldırdı. Şimdilik merkez veya LOS-rate kapısı eklenmemelidir; önce mevcut
kapının gerçek `.rpk` confidence ve koordinat davranışı ölçülmelidir.

## 6. Düz rota testi — 2026-08-11

RoboFly, `hedef_duz.plan`, N=4 ve 5×%3 kapı ile tek kampanyada sekiz ayrı
görsel devir oluştu.

Gerçek CPA'lar:

`1,85 · 0,44 · 1,09 · 0,17 · 1,52 · 0,61 · 1,50 · 0,64 m`

| Ölçüt | Sonuç |
|---|---:|
| Angajman | 8 |
| CPA medyan / p90 / maks | **0,87 / 1,62 / 1,85 m** |
| `<5 m` | **8/8** |
| `<3 m` | **8/8** |
| `vurus_basarili` | **2/8** |
| Son temas | `vibe≈117`, ardından crash/LOITER ve altitude abort |

Düz hedefleri geometrik olarak yakalayabildiği doğrulandı. Buna rağmen
`0,17 m` CPA'lı geçişte fiziksel temas olayı oluşmadı; bu yüzden yalnız CPA
“vurdu” demek değildir. Son fiziksel temas aracı düşürdü; emniyet sonucu da
başarı metriğine dahil edilmelidir.

Düz hedefte bile faz değişimi sayıları angajman başına
`8, 2, 7, 3, 12, 2, 14, 2` oldu. Normal tek geçişte yaklaşık iki faz olayı
beklenir. Bu veri, kullanıcının gördüğü titreşim için gerçek bir
`YERLES↔VUR` chatter kanıtıdır.

Veriler:

- Deney: `run/denemeler/los_big5a3_n4_duz_20260811_145455/`
- LOS CSV: `guidance_allstar/logs/goruntulu_terminal_los_20260811_145628.csv`
- Video: `videos/los_big5a3_n4_duz_20260811_145517.mp4`
- Video doğrulaması: 1280×720, yaklaşık 27,28 FPS, 482,64 s, 126 MB.

Bu sekiz angajman tek yığın başlangıcından geldiği için iyi bir ön kanıttır,
bağımsız koşu tekrarı yerine geçmez.

## 7. Elips ve önceki yöntem kıyası

| Yöntem/kapı | n | CPA medyan / p90 | `<5 m` | fiziksel temas | not |
|---|---:|---:|---:|---:|---|
| Saf MPC uzun taban | 59 | 11,20 / — | 3/59 | ölçülmedi | terminal tersleme |
| Erken/kapısız saf LOS | 19 | 1,95 / 39,13 m | 15/19 | 0 | 13,74–48,74 m ağır kuyruk |
| LOS N=4, 5×%3, elips | 6 | **1,09 / 2,03 m** | **6/6** | **2** | güncel elips tabanı |
| LOS N=5, 5×%3, elips | 12 | 1,65 / 3,76 m | 11/12 | 0 | bir 8,94 m ıska |
| LOS N=4, 5×%3, düz | 8 | **0,87 / 1,62 m** | **8/8** | **2** | son temas aracı düşürdü |

N=5 titreşimi çözmedi ve N=4'ten daha iyi çıkmadı; yalnız `N` yükseltmek
dönüş sorununa cevap değildir. Güncel N=4 elips havuzu da sert dönüş kanıtı
değildir: altı geçişte CPA öncesi hedef dönüşü yaklaşık `0,06–0,35°/s`
bandındaydı. N=5 logunda sürdürülen `1,6–6,22°/s` dönüşlü dört geçişin CPA
medyanı yaklaşık `2,66 m` oldu. Sert `±8/±12°/s` hücreleri henüz sistematik
ölçülmedi.

## 8. Titreşim ve dönüş kaçırma için güncel teşhis

Öncelik sırası şöyledir:

1. **Kamera–yaw zaman uyuşmazlığı.** Terminal `q_az=d_ex+yaw_rate_now`
   hesaplıyor. `d_ex` gecikmiş görüntüden, yaw-rate ise taze ATTITUDE'dan
   geliyor. `pid_gudum.py` aynı hatayı daha önce “yapay geri besleme” olarak
   tanımlamış ve yaw'ı görüntünün capture zamanına taşıyarak çözmüş.
2. **Faz chatter'ı.** VUR için minimum dwell yok; `q_az` 11°/s çevresinde
   oynadığında ileri ivme `0/0,5 ↔ 4+ m/s²` arasında sıçrayabiliyor.
3. **Yanal doygunluk ile ileri gazın uyumsuzluğu.** N=4 ve kapanma 10 m/s
   iken yanal kanal yaklaşık `7,2°/s`de, kapanma 20 m/s iken `3,6°/s`de
   5 m/s² tavana vuruyor; ileri gaz ise 11°/s'ye kadar devam edebiliyor.
4. **Yatay gerçek jerk sınırı yok.** 0,70 s ulaşılabilirlik zarfı ardışık
   döngülerde `+5↔−5 m/s²` ham yanal istek sıçramasını tek başına engellemiyor.
5. **Eyleyici modeli iyimser.** Terminal 5 m/s² kullanırken MPC kampanyası
   yaklaşık 4 m/s² plato ve 1,7 s cevap ölçtü.
6. **Hareketli gimbal açı vekili ve görüntü zinciri.** Açık çevrim servo
   gecikmesi, ters kamera, bbox merkez titremesi ve tutum gecikmesi doğrudan
   sahte LOS-rate üretebilir.
7. **Klasik PN reaktiftir.** Hedef dönüşünü ancak LOS dönmeye başladıktan
   sonra görür; güvenilir hedef-ivme gözlemcisi yoktur.

İlk çözüm geniş bir dead-zone eklemek değildir. LOS-rate üzerindeki gerçek
küçük manevrayı da öldürüp CPA'yı büyütebilir. Önce zaman eşleme, ölçüm
kalitesi, gerçek per-step jerk ve faz histerezisi ölçülmelidir. Dead-zone
ancak düz-uçuş gürültü dağılımından türetilmiş küçük bir eşik olarak A/B
edilmelidir.

Roll komutu doğrudan verilmemelidir. Önce aynı zaman tabanında
`lambda_dot → a_yan → velocity setpoint → desired roll/rate → actual
roll/rate → gyro/motor` zinciri ayrılmalıdır. Komut düzgünken actual roll
titrerse sorun FC/mekanik/filtre; komut da titrerse sorun dış LOS döngüsüdür.

## 9. Gerçek donanım stacki: kaç süreç, kaç dosya?

### Çalışan süreçler

Tüm görev üç mantıksal kontrol rolünden oluşur:

1. **Konumlu yaklaşım:** hedefin arkasındaki slotu kurar ve görsel faz
   devralınca aynı araca setpoint göndermeyi kesin olarak bırakır.
2. **Kamera/gimbal ölçümü:** `donanim/kamera_kopru.py`.
3. **Terminal LOS:** `donanim/gudum_tek_dugum.py --gudum los --buyuk-kare 5 --alan-pct 3`.

Pi üzerindeki asgari görsel runtime iki Python sürecidir: kamera köprüsü ve
tek-düğüm LOS. Konumlu rol başka bilgisayarda olabilir, fakat görev boyunca
canlı kalır. Bunlara `redis-server` ve her tüketici için ayrı çıkış veren bir
MAVLink router/MAVProxy eklenir.

### Kritik otorite durumu

`simple_guided_follow.py`, Redis `komut_yetkisi=goruntulu` olduğunda setpoint
ve mod komutlarını gerçekten kesiyor. `formation_KILLER.py` içindeki güncel
`_attacker_intercept_thread` ise `komut_yetkisi`, `tekdugum_durum` veya
`tekdugum_hayatta` okumuyor ve `goto_shared_ned` göndermeye devam ediyor.

Bu nedenle bugünkü `formation_KILLER + gudum_tek_dugum` birleşimi uçuşa
hazır değildir; iki komut yazarı çatışabilir. Ayrıca repoda
`formation_KILLER.py`nin import ettiği `config.py` yoktur. Donanımda kullanılan
özel `config.py` de dağıtım manifestine alınmalıdır.

### Kopyalanacak bağımlılıklar

Yalnız iki giriş dosyasını kopyalamak yetmez. Repo dizin yapısı korunarak en
az şu dosyalar gerekir:

- Kamera: `donanim/kamera_kopru.py`, `bbox_to_redis.py`,
  `yildizlar_gimbal.py`, `tools/gz_gimbal.py`, `tools/mavlink_tilt.py`, `.rpk`
  model.
- LOS: `donanim/gudum_tek_dugum.py` ve
  `guidance_allstar/{guidance_config.py,goruntulu_temel.py,terminal_los_gudum.py,los_gudum.py,mavlink_utils.py,vector_math.py,numeric_differentiation.py,filterwndr.py}`.
- Konumlu: seçilen gerçek yaklaşım giriş dosyası ve bütün bağımlılıkları;
  `formation_KILLER.py` seçilirse harici `config.py` dahil.
- Python: `numpy`, `redis`, `pymavlink`, `opencv`, `filterpy`; `.rpk` için
  Picamera2/IMX500 paketleri.
- Sistem: `redis-server`, `python3-picamera2`, `imx500-all`, MAVLink router.

`requirements.txt` şu an `filterpy` içermiyor; LOS seçilse bile
`goruntulu_temel → filterwndr` import zinciri nedeniyle temiz Pi kurulumunda
bu paket gerekir. Bu P0 dağıtım hatasıdır.

## 10. Donanımda çalışmış üç `yarışma/` dosyası neyi kanıtlıyor?

| Dosya | Gerçek rol | Uçuş runtime'ı mı? |
|---|---|---|
| `yarışma/gimbal_bench_takip.py` | Sabit/düz araç varsayımıyla kamera+servo masa takibi | Hayır |
| `yarışma/mavlink_tilt.py` | Servo mount sürücüsü ve 0/+30/−30/0 sweep teşhisi | Modül olarak evet, sweep olarak hayır |
| `yarışma/mpc_komut_izle.py` | Gerçek LOS veya MPC çıktısını hesaplayan salt gözlemci; hız/arm/mod göndermez | Hayır |

Bu dosyaların donanımda çalışmış olması gimbal açısının okunduğunu veya LOS'un
aracı uçurduğunu kanıtlamaz. `mpc_komut_izle.py` kamera/gimbali yönetmez;
Redis'te önceden üretilmiş `tracker_bbox_stab` bekler.

## 11. Gimbal açısı gerekli mi?

Kısa cevap:

- Bbox/confidence/alan ile görüntülüye **geçiş** için gerekli değil.
- Servoya açık çevrim **komut göndermek** için gerekli değil.
- Sabit ve kalibre edilmiş kamera–gövde dönüşümünde ayrı açı sensörü gerekli
  değil; sabit mount ve FC roll/pitch yeterlidir.
- Hareketli tilt gimbaliyle sert bank altında doğru 3B/stabilize LOS için
  gerçek gimbal açısı veya doğrulanmış bir durum gözlemcisi gereklidir.

Üretim köprüsü bugün gerçek mekanik açıyı okumuyor.
`kamera_kopru._tilt_eps()`, son yayımlanan servo komutunu kamera dünya
elevasyonu sayıyor. Servo lag, backlash, yük, stall veya doygunluk varsa bu
vekil yanlıştır. Aynı geometriyle 10° tilt vekil hatası yaklaşık olarak:

| Araç roll | Sahte yatay LOS |
|---:|---:|
| 15° | 2,7° |
| 30° | 5,8° |
| 45° | 10,2° |

`SERVO_OUTPUT_RAW` yalnız PWM çıkışıdır, mekanik açı değildir. Raspberry Pi
AI Camera/IMX500 resmi özelliklerinde de gimbal açısı sağlayan bir IMU yoktur.
MAVLink `GIMBAL_DEVICE_ATTITUDE_STATUS` quaternion yolu vardır, fakat servo
mount backend'inin gerçekten ölçülmüş açı mı yoksa komut durumu mu yayımladığı
tezgâhta açıölçerle doğrulanmalıdır.

İlk gerçek uçuş için en düşük riskli yol, gimbali sabit ve kalibre bir açıya
kilitleyip FC ATTITUDE ile LOS işaretlerini kanıtlamaktır. Hareketli mod;
encoder/pot, doğrulanmış gimbal attitude veya ölçülmüş komut→açı gecikme ve
backlash modeli sonrasında açılmalıdır.

## 12. Bench kodunda olup üretim köprüsünde olmayan yararlı ayrıntılar

| Bench'te kanıtlanan | Üretim köprüsünün bugünkü hali | Karar |
|---|---|---|
| `hflip+vflip` ile 180° dönüş | Picamera2 konfigürasyonunda transform yok | Fiziksel montaj tersse P0 |
| HSV OPEN+CLOSE | Yalnız iki dilation | Detector A/B |
| Kontur moment merkezi | Axis-aligned bbox midpoint | Titreşim A/B |
| 0,4° görüntü dead-zone'u | Tilt takip girişinde yok | Gürültüden ölçerek A/B |
| 50 Hz, 120°/s, 0,1° servo deadband | Üretim varsayılanı yaklaşık 10 Hz, 150°/s, 0,2° | Gerçek açı/cadence ölçümü |
| 3 s kayıpta tut, yavaş merkeze dön | Üretimde `TiltTakip` ile korunuyor | Koru |

En kritik sessiz fark 180° dönüşüdür. Aynı fiziksel ters montaj sürüyorsa hem
`ex` hem `ey` işareti ters kalabilir.

## 13. Başka yasalardan korunmaya değer fikirler

Önce taşınabilecekler:

- `pid_gudum.py`: görüntü capture zamanında yaw eşleme; bbox alanı+yaşından
  ölçüm kalitesi; düşük kalitede daha ağır LOS-rate filtresi; kayıptan sonra
  türev durumunu sıfırlama.
- `los_gudum.py`: yalnız ölçülmüş gecikme kadar, kelepçeli LOS öngörüsü;
  gürültülü ham ivme yerine sönümlenen lead state.
- `mpc_gudum.py`: ölçülmüş eyleyici zarfı, komut yumuşaklığı ve
  proximal/constraint governor fikri.
- `takip_gudum.py`: bbox alanının mutlak menzil değil göreli büyüme/ıska
  sinyali olarak kullanılması; hız artışını yönü bozmadan sınırlama.
- `formation_KILLER.py`: tek MAVLink receiver/dispatcher, kaynak timestamp ve
  bayatlık kapısı, hız sıçraması reddi, analitik range-rate, etki sonrası
  çoklu health kontrolü ve saldırı dinamiklerini geri yükleme.
- Bench: 180° dönüş, morphology, moment merkezi, servo rampası ve kayıp
  politikası.

Şimdilik taşınmaması gerekenler:

- Ham hedef ivmesi türeviyle APN.
- `minAreaRect rot_aci_deg` değerini doğrudan dönüş feed-forward'u yapmak;
  çevrimdışı işaret/korelasyon zayıf ve menzile göre değişiyor.
- Eski uzun-ufuk kesişme tahmini. IRL logunda `t_int=1707,7 s` ve yaklaşık
  40 km ötede aim gibi kötü koşullanmış örnekler var.
- Sabit gövde kamerasına göre yazılmış eski dikey LOS/FOV varsayımları.

## 14. IRL_Tests'ten çıkan zarf ve uyarılar

Bu kayıtlar güncel görüntülü LOS uçuş kanıtı değil; eski konum/kesişme
stackinin ve gerçek araç zarfının kanıtıdır.

- Toplam 138 eski atakta kayıtlı başarılı vuruş yok: 70 range-opening miss,
  63 timeout, 3 target-ran-away ve 2 shutdown.
- İlk kampanyada en iyi menzil medyanı yaklaşık 32 m; `16/100 ≤5 m`.
- Son kampanyada medyan yaklaşık 49,5 m; `2/38 ≤5 m`.
- Son 38 atak penceresinde tepe hız medyan/p90/maks yaklaşık
  `18,86/23,99/24,16 m/s`.
- Tepe `|roll|` yaklaşık `32,03/44,04/45,76°`; tepe `|pitch|`
  `39,41/45,38/46,60°`; tepe açısal hız `36,2/49,7/70,1°/s`.
- İstenen ATTITUDE 25 Hz olmasına rağmen tlogda araç başına fiilî yaklaşık
  5 Hz görüldü. 20 Hz guidance titreşimini ayırmaya yetmez.
- Atak penceresi VIBRATION tepe medyan/p90/maks yaklaşık
  `18,5/25,3/41,6`. Güncel `vibe>10 && R<3 m` sim vuruş eşiği gerçek uçuşa
  doğrudan taşınamaz.

## 15. Literatürün bu stack için söylediği

- Gerçek uçuşta PN tabanlı image-based visual servoing; roll/pitch'i ana
  yakalama kanalı, küçük yaw-PD'yi yalnız FOV tutma kanalı olarak başarıyla
  kullanıyor. Bu, “yaw roll'ün yerine geçmesin ama tamamen de kaybolmasın”
  kararını destekliyor: [Yan vd.](https://arxiv.org/html/2409.17497v2)
- Yaklaşık 100 ms kamera/işleme gecikmesi 20 m/s'de 2 m konum farkına denk
  gelebiliyor; IMU ile gecikmiş görüntüyü komut anına taşımak gerçek dairesel
  hedef testlerinde fayda sağlamış: [Yang vd.](https://arxiv.org/html/2404.08296)
- Klasik PN manevrasız hedef varsayar; APN ancak güvenilir hedef ivmesi varsa
  avantajlıdır: [Johns Hopkins APL, Modern Homing Guidance](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Homing.pdf)
- Türev/ivme kestiriminin ölçüm gürültüsü hassasiyeti: [Johns Hopkins APL,
  Guidance Filter Fundamentals](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Guidance.pdf)
- İyi çalışan nominal yasayı koruyup yalnız kısıt ihlalinde referansı
  değiştiren yapı: [Reference Governor incelemesi](https://shadow.merl.com/publications/docs/TR2016-102.pdf)
- Tam perception-aware NMPC mümkündür, fakat model/çözücü/ayar maliyeti daha
  yüksektir ve burada son araştırma basamağı olmalıdır: [Model Predictive
  Spherical IBVS](https://arxiv.org/abs/2212.09613)
- Gerçek gimbal attitude için standart mesaj:
  [MAVLink GIMBAL_DEVICE_ATTITUDE_STATUS](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS)
- IMX500 ürün özetinde kamera içi gimbal/attitude IMU'su yoktur:
  [Raspberry Pi AI Camera product brief](https://datasheets.raspberrypi.com/camera/ai-camera-product-brief.pdf)

## 16. Simülasyon komutları — sırayla, her biri tek satır

Ortamı RoboFly, LOS kapısı ve video kaydıyla başlat:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && YILDIZ_VIDEO=1 YILDIZ_VIDEO_ETIKET=los_robofly YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 YILDIZ_DEVIR_SOGUMA_S=3 ./yildizlar_gudum.sh --robofly
```

Hedefe elips görevi verip avcıyı kaldır:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 tools/gorev_baslat.py --drones 1 --drone-alt 60 --plan missions/hedef_elips.plan
```

Konumlu arka-slot yaklaşımını çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
```

Ayrı terminalde görüntülü LOS'u çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum/guidance_allstar && python3 terminal_los_gudum.py
```

Ortamı düzgün kapatıp MP4 başlığını tamamla:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && ./yildizlar_gudum.sh --stop
```

Tam otomatik elips kampanyası:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && YILDIZ_DRONE_MODEL=robofly YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 YILDIZ_DEVIR_SOGUMA_S=3 SURE=600 KONTROL_BEKLE_S=20 GORUNTULU="terminal_los_gudum.py" PLAN=missions/hedef_elips.plan METOT=los_big5a3_n4 tools/senaryo.sh
```

Tam otomatik düz kampanya:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && YILDIZ_DRONE_MODEL=robofly YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 YILDIZ_DEVIR_SOGUMA_S=3 SURE=360 KONTROL_BEKLE_S=20 GORUNTULU="terminal_los_gudum.py" PLAN=missions/hedef_duz.plan METOT=los_big5a3_n4 tools/senaryo.sh
```

Birim/kapalı-döngü testleri:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 guidance_allstar/terminal_los_test.py
```

2026-08-11'de bu test `15/15 OK` verdi.

## 17. Donanımda şimdilik yalnız güvenli test komutları

Bench kamera+servo, araç sabit ve pervaneler güvenliyken:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 yarışma/gimbal_bench_takip.py --kaynak picam --kuru --goster
```

Servo mount sweep, araç sabit ve pervaneler güvenliyken:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 yarışma/mavlink_tilt.py --baglanti udp:127.0.0.1:14554 --kanal 9
```

Gerçek LOS yasasını aracı sürmeden izle:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 yarışma/mpc_komut_izle.py --yasa los --menzil-m 20 --n-pn 4 --vur-ivme 4
```

Tüm tek-düğüm zincirini MAVLink'e hız komutu yazmadan çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum && python3 donanim/gudum_tek_dugum.py --gudum los --buyuk-kare 5 --alan-pct 3 --dry-run --sure 60
```

Gerçek komutlu `kamera_kopru + gudum_tek_dugum + formation_KILLER` satırı
bilerek verilmedi. `LAST_TO_DO.md` P0 tamamlanmadan bugünkü kaynaklarla o
kombinasyon iki yazarlı ve kamera yönü belirsizdir.
