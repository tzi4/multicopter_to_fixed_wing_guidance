# NOTLAR_HIBRIT — tarihsel MPC→LOS kolu

> 2026-08-11 güncel sade donanım adayı saf görüntülü LOS/PN'dir. Kısa
> komutlar, 5 büyük bbox geçişi ve gerçek donanım sırası için
> `NOTLAR_LOS_DONANIM.md` dosyasına bak. Bu dosya karşılaştırmada kullanılan
> MPC→LOS hibritinin tarihsel kaydı olarak korunuyor.

Güncel ana aday: `guidance_allstar/hibrit_gudum.py`.

## Kod haritası

| Dosya | Görevi |
|---|---|
| `simple_guided_follow.py` | Konumlu yaklaşım; hedefin arkasındaki slotu kurar |
| `hibrit_gudum.py` | Görüntülü fazın ana çalıştırıcısı ve MPC→LOS geçişi |
| `mpc_gudum.py` | Hibritin 18 m dışındaki orta-menzil planlayıcısı |
| `terminal_los_gudum.py` | 18 m içindeki PN/LOS terminal yasası |
| `goruntulu_temel.py` | Kamera ölçümü, MAVLink komutu, Redis yetkisi ve ortak LPF |
| `bbox_to_redis.py` | Konumlu→görüntülü yetki kararını verir |

Ana geliştirme yaparken önce `hibrit_gudum.py` ve
`terminal_los_gudum.py`ye bak. Saf MPC davranışını değiştireceksen
`mpc_gudum.py`yi düzenle; bu değişiklik hibritin dış bandını da etkiler.

## Manuel çalışma sırası

Her satırı ayrı terminalde çalıştır. İlk iki komut tamamlandıktan sonra
konumlu ve görüntülü terminaller açık kalır.

### 1. Ortamı seçilen araç modeliyle başlat

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_VIDEO=1 YILDIZ_VIDEO_ETIKET=hibrit_robofly ./yildizlar_gudum.sh --robofly
```

Güncel test aracı RoboFly'dır. Karşılaştırma gerektiğinde araç seçeneğini
`--iris` veya `--hummingbird` ile değiştirebilirsin. GUI istemiyorsan aynı
satırın sonuna `--headless` ekle. Video artık varsayılan açık olsa da
`YILDIZ_VIDEO=1` komutta niyeti görünür tutar; yalnız `YILDIZ_VIDEO=0`
kaydı kapatır. Bu seçenek **ortam başlarken** okunur; sonradan konumlu veya
hibrit komutuna eklemek kayıt başlatmaz.

Başlangıçtan birkaç saniye sonra video ve model doğrulaması:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && printf 'model=' && cat run/selected_model && rg "Video kaydi:" logs/bbox.log | tail -1 && ls -lh videos/*.mp4 | tail -1
```

### 2. Hedefi göreve sok ve avcıyı havaya kaldır

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 tools/gorev_baslat.py --drones 1 --drone-alt 60 --plan missions/hedef_elips.plan
```

### 3. Konumlu güdümü çalıştır

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6
```

### 4. Görüntülü hibrit güdümü çalıştır

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 hibrit_gudum.py
```

### Ortamı kapat

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && ./yildizlar_gudum.sh --stop
```

## Otomatik elips deneyi

Yukarıdaki sırayı tek komutla çalıştırmak ve log/özet toplamak için:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly YILDIZ_VIDEO=1 SURE=360 KONTROL_BEKLE_S=20 GORUNTULU="hibrit_gudum.py" PLAN=missions/hedef_elips.plan METOT=hibrit tools/senaryo.sh
```

`tools/senaryo.sh`, `YENIDEN_BASLAT=1` iken videoyu ayrıca zorunlu olarak
açar; komuttaki `YILDIZ_VIDEO=1` niyeti görünür kılmak için yazılmıştır.

Yığın zaten sağlıklı biçimde açıksa:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly SURE=360 YENIDEN_BASLAT=0 KONTROL_BEKLE_S=0 GORUNTULU="hibrit_gudum.py" PLAN=missions/hedef_elips.plan METOT=hibrit tools/senaryo.sh
```

Bu ikinci komut yalnız yığın daha önce `YILDIZ_VIDEO=1` ile başlatılmışsa
video alır. Çalışan bbox sürecine sonradan video özelliği eklenemez; gerekirse
yığını durdurup 1. adımdaki komutla yeniden başlat.

Yeni modelin ilk otomatik koşusunda `YENIDEN_BASLAT=1` kullan. Model, kamera
konusu, MAVLink portları ve ArduPilot parametreleri doğrulandıktan sonra
`YENIDEN_BASLAT=0` ile daha hızlı tekrar yapılabilir.

## İki farklı devir

### 1. Konumlu → görüntülü hibrit

Konumlu ve hibrit süreç aynı anda çalışır. Hibrit başlangıçta yalnız
`goruntulu_hayatta` kalp atışı gönderir, komut göndermez.

Varsayılan karar kuralı:

- Son 25 karenin en az 20'sinde geçerli tespit,
- bbox alan eşiği: kadrajın `%2 × %2` dikdörtgen alanı,
- konumlu kestirim menzili en fazla `60 m`,
- çalışan görüntülü kontrolcü kalp atışı.

Koşullar sağlanınca `bbox_to_redis.py`, Redis `komut_yetkisi` değerini
`goruntulu` yapar. Konumlu süreç setpoint göndermeyi keser. Hibrit,
`devir_durumu` içindeki son konumlu hız komutuyla tohumlanır; böylece ilk
komutta hız sıçraması azaltılır.

Geçerli tespit sayısı geri dönüş eşiğinde kalıp 2 saniyelik dwell dolarsa,
görüntülü süreç ölürse veya terminal yasa geçiş sonrası ıska bildirirse yetki
yeniden konumluya döner.

### 2. Hibrit içinde MPC → PN/LOS

Bu bir Redis veya uçuş modu değişimi değildir. Aynı `hibrit_gudum.py`
sürecindeki yazılım durumu değişir:

```text
görüntülü yetki alındı
        |
        v
 R > 18 m: MPC + ortak ulaşılabilirlik sınırı
        |
        | ölçülen menzil <= 18 m
        v
 R <= 18 m: terminal PN/LOS (angajman boyunca latch)
```

Geçişte terminal yasa mevcut araç hızı ve menzille yeniden tohumlanır.
Menzil tekrar 18 m üstüne çıksa bile aynı angajmanda MPC'ye dönmez; bu,
eşik çevresindeki MPC/LOS titreşimini engeller. Iskadan sonra görüntülü yetki
bırakılır, konumlu katman yeniden arka slotu kurar.

Gerçek araçta bu geçişin zorunlu girdisi güvenilir `menzil_m` ölçümüdür.
Simülasyonda bu büyüklük hedefin konum paketlerini besleyen IMM kestiricisinden
üretiliyor; hedef yönü/hızı terminal yasaya verilmiyor ama menzil yine hedef
konum telemetrisine dayanıyor. Gerçek donanımda bunun yerine onboard
stereo/depth/radar veya doğrulanmış görüntü-temelli menzil konmalıdır. Menzil
`None` kalırsa MPC→LOS geçişi gerçekleşmez.

Deneysel yalnız-görüntü geçişi (varsayılan değildir):

```bash
cd /path/to/multicopter_to_fixed_wing_guidance/guidance_allstar && python3 hibrit_gudum.py --gecis-kaynagi gorsel
```

Varsayılan eşikler `alan=%3.4`, `|ex|<=6°`, `|ey|<=15°`, `dwell=0.30 s`.
İstersen sırasıyla `--gorsel-alan-pct`, `--gorsel-ex`, `--gorsel-ey` ve
`--gorsel-dwell` ile değiştir. 2026-08-11 uzun RoboFly/elips testinde bu kol
LOS'a geçtiği 15 angajmanda güçlüydü (`CPA medyan 1.50 m`, 13/15 `<5 m`),
ama 29 devrin yalnız 15'inde kapıyı açabildi. Bu nedenle varsayılan menzil
geçişi korunuyor; ayrıntılı tablo `TO_TEST.md` içindeki **CANLI T2 HÜKMÜ**nde.

Önemli: geçişi görsel yapmak terminal yasayı tamamen telemetrisiz yapmaz.
Mevcut PN/LOS hâlâ kazanç, `t_go`, dikey çözüm ve ıska bırakma için
`menzil_m` kullanır. Gerçek araç için stereo/depth/radar veya doğrulanmış
monoküler menzil gerekir. İlerideki YOLO dedektörünün confidence değeri de
Redis ölçümüne/loga eklenmeli; mevcut HSV dedektörü confidence üretmiyor.

## Kamera eşleştirmesi

RoboFly simülasyon kamerası:

| Özellik | Değer |
|---|---:|
| Görüntü | 1280 × 720, BGR8 |
| Kamera hızı | 30 FPS |
| Yatay görüş açısı | 66.0° |
| Dikey görüş açısı | yaklaşık 40.1° |
| İç parametre | `fx=fy=985.5`, `cx=640`, `cy=360` |
| ROS konusu | `/drone_1/webcam/image_raw` |

Gerçek kamerada yalnız çözünürlüğü eşlemek yetmez. Aynı yatay FOV/crop ve
kalibre edilmiş iç parametreler kullanılmalı; aksi halde pikselden açı ve
bbox-alanından yakınlık eşikleri değişir.

Gerçek kamera için ayrıca otomatik pozlama/beyaz dengesi davranışını, gerçek
FPS'yi, uçtan uca gecikmeyi ve timestamp kaynağını ölç. Dijital zoom,
stabilizasyon ve değişken crop kapalı veya kalibrasyonda kullanılan hâliyle
sabit olmalı. Rolling-shutter süresi hızlı yaw/roll altında ayrıca sınanmalı.

## Gerçek gimbal uçuş-öncesi doğrulaması

Gimbal açısını okuyamıyorsan komut edilen açıyı gerçek açı kabul etmek ancak
servo gecikmesi, backlash ve doyum ihmal edilebiliyorsa çalışır; saldırı
manevrasında bu güvenli bir varsayım değildir. Tercih sırası:

1. Servo encoder/potansiyometre veya gimbal telemetrisinden gerçek açı,
2. Kamera taşıyıcısına IMU ve gövde IMU'su ile göreli açı,
3. Kalibre edilmiş servo dinamiğiyle komut+açı kestirimi,
4. Gimbal geri bildirimi yoksa sabit kamera ve `--gimbalsiz` kontrollü sınama.

Güdüm kapalıyken şu testleri sırayla geçir:

1. Yedi statik görüntüde (merkez, dört köşe, üst-orta, alt-orta) bbox merkezi,
   `ex/ey` işareti, büyüklük simetrisi ve FOV/intrinsics doğrulaması.
2. Drone sabitken gimbale `0, ±5, ±10, ±15°` adım ver; piksel/° kazancı,
   yerleşme süresi, overshoot, deadband, backlash ve doyumu ölç.
3. Hedef sabitken gövdeyi güvenli düzende roll/pitch ile oynat; stabilize
   edilmiş hedef hatası sabit kalmalı. Tek eksenli tilt gimbal roll'u
   düzeltemez; roll kaynaklı görüntü dönüşünü kod telafi etmeli.
4. Pervaneler sökülü veya güvenli bağlı testte gimbal komutu, gerçek açı,
   kamera timestamp'i ve bbox timestamp'i arasındaki gecikmeyi ölç.
5. Hover'da yatay güdüm kapalı, yalnız gimbal takibi açık: merkez RMS/p95,
   kayıp kare oranı, saturation süresi ve yeniden yakalama süresi kaydedilir.
6. Son olarak düşük hız, geofence, irtifa tabanı ve manuel override ile
   kademeli görüntülü güdüm testi yapılır.

Yedi fotoğraf geldiğinde ilk madde için aynı dedektör ve gerçek kamera
kalibrasyonuyla sayısal bir kabul tablosu çıkarılmalı; yalnız gözle “ortada”
değerlendirmesi yeterli değildir.

## Güncel varsayılanlar

| Parametre | Değer |
|---|---:|
| MPC→LOS menzili | 18 m |
| PN katsayısı `N` | 4 |
| VUR ileri ivmesi | 4 m/s² |
| Tırmanma hız sınırı | 2.5 m/s |
| Alçalma hız sınırı | 2.0 m/s |
| DON menzili | 3 m |
| DON `t_go` | 0.25 s |

Tek değişkenli örnek:

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && YILDIZ_DRONE_MODEL=robofly SURE=360 KONTROL_BEKLE_S=20 GORUNTULU="hibrit_gudum.py --n-pn 5 --gecis-menzil 20 --tirmanma-hiz-max 2.0" PLAN=missions/hedef_elips.plan METOT=h_n5_r20_vz2 tools/senaryo.sh
```

Her değişik kombinasyona benzersiz `METOT` adı ver. Aynı koşuda birden fazla
parametreyi değiştirirsen sonucun nedenini ayıramazsın.

## Kontrol ve loglar

```bash
cd /path/to/multicopter_to_fixed_wing_guidance && python3 guidance_allstar/terminal_los_test.py
cd /path/to/multicopter_to_fixed_wing_guidance && python3 -m py_compile guidance_allstar/hibrit_gudum.py guidance_allstar/terminal_los_gudum.py
```

Beklenen olaylar:

- `devir_alindi`: konumlu→görüntülü,
- `hibrit_los_gecisi`: MPC→LOS,
- `faz_vur` / `faz_don`: terminal durumları,
- `iska_birak`: terminal geçiş sonrası yetkiyi konumluya bırakmış.

Başlıca çıktılar:

- Manuel kayıtta `videos/gudum_<tarih>.mp4`,
- otomatik deneyde `videos/<METOT>_<rota>_<tarih>.mp4`,
- `run/denemeler/<METOT>_.../goruntulu.log`
- `run/denemeler/<METOT>_.../ozet.txt`
- `guidance_allstar/logs/goruntulu_hibrit_*.csv`
- `guidance_allstar/logs/hibrit_mpc_tani_*.csv`

Video ortamla birlikte kayda başlar. MP4 dosyasının düzgün kapanması için
deney sonunda `./yildizlar_gudum.sh --stop` komutunu kullan.

Başarı yalnız minimum menzil değildir. Birlikte kontrol et:

- `ref_menzil_gercek_m`,
- `vibe_max` veya `vurus_basarili`,
- `ALTITUDE ABORT`,
- hedef tazelik oranı,
- MPC/LOS geçişindeki komut–gerçek hız açısı.

Mevcut canlı karar ve sıradaki deney matrisi: `TO_TEST.md`, **CANLI T1
HÜKMÜ**. Ayrıntılı saf-MPC geçmişi: `NOTLAR_MPC.md`.
