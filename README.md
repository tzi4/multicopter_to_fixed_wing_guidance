# savasan_iha_yildizlar — Görsel Güdüm ve Otonom Önleme Araştırma Platformu

Bu depo; hareketli bir hava hedefinin çoklu araç simülasyonunda bulunması, arkadan yaklaşma ile emniyetli biçimde devralınması ve son safhada kamera tabanlı bir güdüm yasasıyla önlenmesi için geliştirdiğimiz uçtan uca araştırma platformudur. Çalışma yalnızca bir kontrol algoritmasından ibaret değildir: ArduPilot SITL, Gazebo, ROS, Redis, MAVLink, fiziksel gimbal modeli, Raspberry Pi 5 ve Raspberry Pi AI Camera arasında çalışan bütün zinciri içerir.

Proje **MIT lisansı ile açık kaynak** olarak yayımlanmaktadır. Kaynak kod; lisans/atıf kayıtları, sabitlenmiş doğrulama ortamı, katkı ve güvenlik belgeleri ile otomatik test hattını içerir. Ham gerçek uçuş telemetrisi, kişisel saha verileri ve özel geliştirme geçmişi açık kaynak dağıtımının dışında tutulur.

> **Güncel durum:** Simülasyonda en başarılı yapı, konum tabanlı yaklaşmadan sonra büyük görüntü kutusu ile devreye giren tek aşamalı **Terminal LOS/PN, `N=4`** yasasıdır. Altı bağımsız denemenin tamamında gerçek en yakın geçiş mesafesi 5 metrenin altında kalmış, iki denemede fiziksel temas imzası oluşmuştur. Gerçek araç yazılım zinciri Raspberry Pi 5 üzerinde hazırlanmıştır; buna rağmen gerçek uçuş öncesinde aşağıdaki kademeli doğrulama ve emniyet adımları zorunludur.

## Sistem ne yapıyor?

Simülasyon ortamı en fazla beş takipçi multikopter ile sabit kanatlı bir hedefi birlikte çalıştırabilir. Her takipçi önce hedefin telemetri konumuna göre hedefin arkasındaki bir slota yerleşir. Kamera hedefi yeterince büyük ve kararlı gördüğünde kontrol görsel terminal safhaya aktarılır. Son safhada görüntü hatası, yaklaşma süresi ve görüş hattı dinamiği kullanılarak hız komutları üretilir.

```text
Hedef görevi / telemetri
          │
          ▼
Konum tabanlı arkadan yaklaşma
          │  5 ardışık büyük ve taze bbox
          ▼
Terminal LOS/PN (N=4)
          │
          ▼
MAVLink hız komutu → ArduPilot → araç

Kamera → algılayıcı → Redis bbox ───────┘
             │
             └── gimbal açı komutu
```

Varsayılan geçiş koşulu 1280×720 görüntüde kutunun hem genişlik hem yükseklik olarak görüntünün en az `%3`'üne ulaşması ve bunun **5 ardışık taze kare** boyunca korunmasıdır. Bu geçiş görsel ve menzilden bağımsızdır. Terminal yasası ise PN kazancı, tahmini varış süresi, düşey kontrol ve kaçırma kararı için hâlen `menzil_m` bilgisini kullanır. Gerçek sistemde bu bilgi telemetriden veya bir kestiriciden sağlanabilir; telemetri bağımlılığını kaldırmak araştırmanın sıradaki ana hedeflerinden biridir.

## Sonuç: yarışma için seçilen güdüm yasası

11 Ağustos 2026 tarihli RoboFly/elips deneylerinde, büyük-kutu devri ile çalışan Terminal LOS/PN için elde edilen sonuçlar şöyledir:

| Yapı | Deneme | Gerçek CPA medyan / p90 | `< 5 m` | Temas imzası |
|---|---:|---:|---:|---:|
| **LOS/PN `N=4`** | 6 | **1.09 / 2.03 m** | **6/6** | **2/6** |
| LOS/PN `N=5` | 12 | 1.65 / 3.76 m | 11/12 | 0/12 |
| Güvenli hibrit | 3 | 3.42 / 3.88 m | 3/3 | 0/3 |
| Uzun saf MPC | 59 | 11.20 m medyan | 3/59 | — |

`N=4` için altı ayrı gerçek CPA değeri:

```text
0.42 m, 0.79 m, 2.60 m, 1.45 m, 1.39 m, 0.63 m
```

İki bağımsız deneyde `vurus_basarili` olayı tetiklenmiş ve simüle edilen araç temasın ardından düşmüştür. Temastan sonraki ölçümler değerlendirme dışı bırakılmıştır. Buradaki “temas”, Gazebo fizik modeli içinde ölçülen bir sonuçtur; gerçek hava aracında güvenli ve tekrarlanabilir temasın kanıtı değildir.

Gimbalın fiziksel simülasyon doğrulamasında gövde yunuslama açısı yaklaşık `−35.4° … +35.2°` arasında değişirken kameranın dünya eksenindeki mutlak yunuslama açısı en fazla `0.65°`, p95 değeri `0.43°` olmuştur. Böylece görüntü merkezleme ile gövde hareketinin ayrıştırılması doğrulanmıştır.

## Araştırmanın yönünü değiştiren bulgu

İlk güçlü adayımız, görüş alanı ve eyleyici sınırlarını doğrudan optimizasyona alan MPC idi. MPC orta menzilde anlamlı bir yol üretse de son 8 metrede sabit `2.4 s` ufuk, gerçek varış süresinden (`t_go` medyanı yaklaşık `1.54 s`) uzun kaldı. Optimizasyon çarpışma anından sonrasını da planladığı için komut edilen hız gerçek hıza göre neredeyse ters yönde oluştu:

- Saf MPC'de son 8 metrede hız açısı medyanı yaklaşık **171.6°** idi; örneklerin `%91.4`'ünde açı `90°` üzerindeydi.
- Hibrit PN/LOS yapısı ters komutu kaldırdı: medyan/p90 **5.35° / 6.72°**, `90°` üzeri **0/103**.
- Büyük görüntü kutusuna kadar konum yaklaşmasını koruyup terminal safhada doğrudan `N=4` LOS/PN kullanmak, hem ağır kuyruğu azalttı hem ilk fiziksel temas imzalarını üretti.

Bu nedenle gelecekteki çalışma “son birkaç metrede saf MPC'yi daha fazla ayarlamak” üzerine kurulmayacaktır. MPC tarihsel A/B adayı ve geri dönüş seçeneği olarak korunmaktadır; ana yön, seçilen LOS/PN yasasını gerçek donanımda kademeli doğrulamak, telemetrisiz menzil kestirmek ve güvenli/tekrarlanabilir terminal davranış elde etmektir.

## Şu ana kadar geliştirilenler

- Beş multikopter ve bir sabit kanatlı hedef için eşzamanlı ArduPilot SITL başlatma, port sözleşmeleri ve görev yükleme.
- Yaklaşık 20–21 m/s hedefi yakalayabilmek için takipçi hız sınırının 18 m/s'den 35 m/s'ye çıkarılması.
- Gazebo gökyüzü ile karışan kırmızı yerine mor hedef algılama, HSV ve IMX500/YOLO algı yolları.
- Redis üzerinde zaman damgalı bbox, görüntü boyutu, güven, gimbal ve menzil veri sözleşmesi.
- Konum tabanlı arkadan-slot yaklaşması, görsel devralma, dead-man, bırakma ve soğuma mantığı.
- FISTA çözücülü, görüş alanı ve eyleyici kısıtlı MPC; kör PN devamı; ıskalama, CPA ve temas günlükleri.
- Terminal LOS/PN, klasik LOS ve PID taban çizgileri ile hibrit güdüm karşılaştırmaları.
- Iris, Hummingbird ve RoboFly modelleri; Hummingbird/RoboFly köprü eklentisi.
- Tek eksenli fiziksel tilt gimbal simülasyonu, gövde hareketinden arındırma ve kazanç düzeltmeleri.
- Raspberry Pi 5 + Raspberry Pi AI Camera IMX500 için Picamera2, 180° görüntü dönüşü, HSV/moment merkezleme, servo rampası ve `DO_MOUNT_CONTROL` desteği.
- Gerçek araç için plan/tilt uyumluluğu, pre-arm sağlık kontrolü, minimum irtifada HOLD kurtarma, kapanışta BRAKE ve geçici parametrelerin temizlenmesi.
- Tek komutlu tekrarlı deney, özet üretimi, A/B karşılaştırma, video, CSV, DataFlash ve simülasyon-zamanı sağlık kontrolleri.

Ham APN türevi, doygun düşey eyleyici, erken düşey rampa, yalnızca ilerleme saatine dayalı çözümler ve kesişim yaklaşması da denendi. Bunlar gürültüyü, irtifa iptallerini veya ağır kuyruk davranışını artırdığı için güncel varsayılan yapıya alınmadı.

## Depo yapısı

| Yol | İçerik |
|---|---|
| `yarışma/` | Yarışmada/gerçek araçta kullanılacak güncel, sadeleştirilmiş kod paketi |
| `guidance_allstar/` | MPC, hibrit, LOS/PN, PID, takip ve test uygulamaları |
| `donanim/` | Raspberry Pi 5, AI Camera, Redis, gimbal ve tek düğüm gerçek araç kodları |
| `tools/` | Görev başlatma, deney otomasyonu, özet ve karşılaştırma araçları |
| `missions/` | Elips, düz, S ve wanderer hedef rotaları |
| `iris/`, `hummingbird/`, `robofly/` | Gazebo araç modelleri |
| `plugins/hummingbird_bridge/` | Gazebo–ArduPilot köprü eklentisi |
| `gimbal_kurulum/` | Gimbal model/kurulum dosyaları |
| `params/` | ArduPilot parametre profilleri |
| `worlds/` | Gazebo dünyaları |
| `run/`, `logs/`, `videos/` | Çalışma çıktıları, günlükler ve kayıtlar |

## Simülasyon ortamının kurulması

Doğrulanmış geliştirme ortamı Ubuntu 20.04 sınıfı bir sistem, **ROS Noetic** ve **Gazebo Classic** kullanır. Farklı dağıtımlar çalışabilir ancak henüz desteklenen kurulum olarak doğrulanmamıştır.

Gerekli ana bileşenler:

- ArduPilot ve derlenmiş `arducopter` / `arduplane` SITL ikilileri
- ROS Noetic: `/opt/ros/noetic/setup.bash`
- Gazebo Classic: `gzserver` ve isteğe bağlı `gzclient`
- `ardupilot_gazebo` ve derlenmiş `libArduPilotPlugin.so`
- Redis, MAVProxy, Python 3, CMake ve Protobuf
- Görsel arayüz kullanılacaksa isteğe bağlı QGroundControl

Python bağımlılıkları:

```bash
python3 -m pip install -r requirements-lock.txt
```

Yerel dizinleri başlatıcıya tanıtın:

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
export ARDUPILOT_GAZEBO_DIR=/path/to/ardupilot_gazebo
export QGC_BIN=/path/to/QGroundControl.AppImage  # isteğe bağlı
```

Başlatıcı `gzserver`, `redis-cli`, `mavproxy.py`, `setsid`, `flock` ve `ss` araçlarını ön kontrolden geçirir. Hummingbird/RoboFly köprüsünü gerektiğinde yeniden derleyebilmesi için `cmake` ve `protoc` da erişilebilir olmalıdır.

## Simülasyonda hızlı test

En güncel yarışma adayını RoboFly ve elips rota üzerinde tek komutla çalıştırmak için depo kökünde:

```bash
YILDIZ_DRONE_MODEL=robofly \
YILDIZ_GECIS_BUYUK_KARE=5 \
YILDIZ_GECIS_ALAN_PCT=3 \
YILDIZ_DEVIR_SOGUMA_S=3 \
SURE=600 KONTROL_BEKLE_S=20 \
GORUNTULU="terminal_los_gudum.py" \
PLAN=missions/hedef_elips.plan \
METOT=los_big5a3 \
tools/senaryo.sh
```

Bu senaryo simülasyonu headless başlatır, video üretimini açar, hedef görevini yükler, araçları kaldırır, konum yaklaşmasını ve görsel devri çalıştırır, ardından sonuç özetini oluşturur. Süreci durdurmak için:

```bash
./yildizlar_gudum.sh --stop
```

Araç modeli ayrı olarak seçilebilir:

```bash
./yildizlar_gudum.sh --iris
./yildizlar_gudum.sh --hummingbird
./yildizlar_gudum.sh --robofly
```

`--headless` grafik arayüzü kapatır; `--gimbalsiz` yalnızca karşılaştırma amacıyla gimbalsiz modeli seçer. Ölçülebilir deneylerde RoboFly ve headless kip önerilir.

## Simülasyonu adım adım çalıştırma

Akışı gözlemlemek veya bir bileşeni ayrı hata ayıklamak için dört terminal kullanabilirsiniz.

Terminal 1 — simülasyon:

```bash
YILDIZ_VIDEO=1 YILDIZ_VIDEO_ETIKET=los_robofly \
YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 \
YILDIZ_DEVIR_SOGUMA_S=3 \
./yildizlar_gudum.sh --robofly
```

Terminal 2 — hedef görevi ve kalkış:

```bash
python3 tools/gorev_baslat.py \
  --drones 1 \
  --drone-alt 60 \
  --plan missions/hedef_elips.plan
```

Terminal 3 — arkadan konum yaklaşması:

```bash
cd guidance_allstar
python3 simple_guided_follow.py \
  --no-kill-mode \
  --yaw-lock \
  --back 25 \
  --down 4
```

Terminal 4 — terminal görsel güdüm:

```bash
cd guidance_allstar
python3 terminal_los_gudum.py
```

Kullanılabilir örnek hedef rotaları:

- `missions/hedef_elips.plan`
- `missions/hedef_duz.plan`
- `missions/hedef_wanderer.plan`
- `missions/hedef_s.plan`

## A/B karşılaştırmaları

Tek komutlu senaryoda `GORUNTULU` değişkenini değiştirerek korunmuş yöntemler karşılaştırılabilir:

| Değer | Yöntem |
|---|---|
| `terminal_los_gudum.py` | Güncel terminal LOS/PN, varsayılan aday |
| `mpc_gudum.py` | Saf MPC, tarihsel karşılaştırma |
| `hibrit_gudum.py` | MPC + terminal PN/LOS hibriti |
| `los_gudum.py` | Klasik LOS taban çizgisi |
| `pid_gudum.py` | PID taban çizgisi |

Örnek:

```bash
GORUNTULU="hibrit_gudum.py" \
PLAN=missions/hedef_elips.plan \
METOT=hibrit_elips \
SURE=600 \
tools/senaryo.sh
```

Her hücre için en az 6–8 bağımsız angajman ve temiz süreç başlangıcı önerilir. `N={4,5,6}`, görsel devir eşiği ve rota aynı anda değil, kontrollü bir faktöriyel düzenle değiştirilmelidir.

## Çıktılar ve deney geçerliliği

Otomatik deney çıktıları aşağıdaki konumlarda oluşur:

```text
run/denemeler/<metot_rota_zaman>/
├── guidance.log
├── goruntulu.log
├── bbox.log
├── ozet.txt
└── aim.txt

videos/                         görüntü kayıtları
guidance_allstar/logs/          CSV güdüm günlükleri
run/sitl0/logs/                 takipçi DataFlash
run/sitl5/logs/                 hedef DataFlash
```

Bir denemeyi tekrar özetlemek ve yöntemleri karşılaştırmak için:

```bash
python3 tools/deneme_ozeti.py run/denemeler/<metot_rota_zaman>
python3 tools/karsilastir.py --metot mpc los pid --csv rapor.csv
```

Sonuç kabul edilmeden önce:

- `simtime_ratio` değerinin 1'e yakın olduğunu doğrulayın.
- Günlükte `SIMULASYON GERIDE` uyarısı varsa denemeyi geçersiz sayın.
- GUI'nin özellikle MPC'de işlem bütçesi kesintilerini artırabileceğini unutmayın; nihai ölçümü headless tekrarlayın.
- Sadece komut telemetrisine değil gerçek CPA, temas/titreşim imzası ve `ALTITUDE ABORT` olaylarına birlikte bakın.
- Plan, tilt ve pre-arm emniyet kapılarını ölçüm uğruna kapatmayın.

## Çevrimdışı doğrulama testleri

Algoritmaları Gazebo açmadan doğrulamak için depo kökünde:

```bash
python3 donanim/test_balon_menzil.py

(
  cd guidance_allstar
  python3 terminal_los_test.py
  python3 mpc_test.py
  python3 los_test.py
  python3 pid_test.py
)

python3 -m compileall -q \
  bbox_to_redis.py donanim guidance_allstar tools yarışma

bash -n yildizlar_gudum.sh tools/senaryo.sh
```

Son doğrulanan test sonucu:

| Test grubu | Sonuç |
|---|---:|
| Terminal LOS/PN | 15/15 |
| MPC | 88/88 |
| LOS | 66/66 |
| PID | 51/51 |
| Balon/menzil yardımcıları | 4/4 |

## Raspberry Pi 5 ve gerçek araçta kullanım

Gerçek araç kodları hem geliştirme geçmişiyle birlikte `donanim/` altında hem de yarışma için seçilmiş güncel kopyalarıyla `yarışma/` altında bulunur. Raspberry Pi 5 üzerinde ROS gerekmez. Temel kurulum:

```bash
sudo apt update
sudo apt install -y \
  redis-server \
  python3-opencv \
  python3-numpy \
  python3-picamera2 \
  imx500-all

python3 -m pip install -r requirements.txt
sudo systemctl enable --now redis-server
```

Kamera tarafında Raspberry Pi AI Camera/IMX500, 1280×720 çözünürlük ve yaklaşık 66° yatay görüş alanı esas alınmıştır. Bazı saha kodları ve kamera/gimbal kararları doğrudan Raspberry Pi 5 üzerinde yapılan deneylerden gelmektedir. Görüntünün fiziksel yerleşime göre 180° çevrilmesi, HSV/moment merkezi, YOLO/IMX500 yolu ve servo rampası bu çalışmalarda doğrulanmıştır.

Gerçek sistem iki ana süreçten oluşur:

1. `kamera_kopru.py`: kamerayı ve algılayıcıyı çalıştırır, Redis'e taze bbox yazar ve gimbalı yönlendirir.
2. `gudum_tek_dugum.py`: görsel devir kararını verir, Terminal LOS/PN yasasını çalıştırır ve MAVLink komutu üretir.

### Kademeli devreye alma

Pervaneler sökülüyken kamera/gimbal masa testi:

```bash
python3 yarışma/gimbal_bench_takip.py \
  --kaynak picam \
  --kuru \
  --goster
```

Yalnızca hesaplanan LOS komutlarını izleme:

```bash
python3 yarışma/mpc_komut_izle.py \
  --yasa los \
  --menzil-m 20 \
  --n-pn 4 \
  --vur-ivme 4
```

Servo yönü ve sınırlarını kontrol etme — pervaneler sökülü olmalıdır:

```bash
python3 yarışma/mavlink_tilt.py \
  --baglanti udp:127.0.0.1:14554 \
  --kanal 9
```

Kamera köprüsü örneği:

```bash
python3 yarışma/kamera_kopru.py \
  --kaynak picamera2 \
  --genislik 1280 \
  --yukseklik 720 \
  --hfov 66 \
  --dedektor yolo \
  --yolo-model MODEL.rpk \
  --yolo-conf 0.70 \
  --mavlink udp:127.0.0.1:14554 \
  --kaydet
```

`MODEL.rpk` yerine IMX500 için hazırlanmış gerçek model dosyasını verin. MAVLink uç noktası da uçuş bilgisayarı bağlantınıza göre değiştirilmelidir.

Önce komut göndermeyen kuru çalışma:

```bash
python3 yarışma/gudum_tek_dugum.py \
  --gudum los \
  --buyuk-kare 5 \
  --alan-pct 3 \
  --dry-run \
  --sure 60
```

Yalnızca bütün emniyet kontrollerinden sonra canlı çalışma adayı:

```bash
python3 yarışma/gudum_tek_dugum.py \
  --gudum los \
  --buyuk-kare 5 \
  --alan-pct 3 \
  --loop-hz 20 \
  --menzil-kaynak estimator
```

Bu komutlar örnek çalışma sözleşmesidir. Bağlantı adresi, servo kanalı, parametre otoritesi, menzil kaynağı ve uçuş sınırları gerçek araca göre doğrulanmadan canlı uçuş yapılmamalıdır.

## Gerçek uçuş emniyet sınırı

Kod zinciri yarışma aracına aktarılmaya hazırdır; bu ifade aracın uçuşa elverişlilik sertifikasına sahip olduğu anlamına gelmez. İlk canlı doğrulama şu sırayla yapılmalıdır:

1. Salt-okunur gözlemci ve günlükleme.
2. Pervanesiz gimbal yönü, mekanik sınır ve gecikme ölçümü.
3. Sabit/askıda araçta kuru komut üretimi.
4. İnsanlardan uzak, geofence içinde, düşük hızlı ve düşük enerjili uçuş.
5. Manuel devralma, kill switch, minimum irtifa ve BRAKE/HOLD kurtarması doğrulandıktan sonra hareketli hedef.

Canlı denemelerde insan, hayvan veya korunmasız malzeme hedef olarak kullanılmamalıdır. Görüş hattı, yerel mevzuat, saha izni ve bağımsız güvenlik pilotu gereksinimleri korunmalıdır.

## Araştırmanın gelecek yönü

Önümüzdeki çalışma sırası elde edilen bulgular tarafından belirlenmiştir:

1. **Kademeli gerçek donanım doğrulaması:** salt-okunur çalışma, gimbal işareti/gecikmesi, hover dry-run ve geofence içinde düşük hızlı uçuş.
2. **Telemetrisiz menzil:** bbox ölçeği ve optik büyüme ile menzil/tahmini varış süresi kestirimi; gerekirse ayrı bir menzil sensörüyle füzyon.
3. **Kapalı çevrim gimbal durumu:** komut edilen açı yerine enkoder, potansiyometre, kamera IMU'su veya ölçülmüş gecikme modeli kullanılması.
4. **Tekrarlanabilir ve emniyetli terminal davranış:** `N={4,5,6}`, devir mesafesi/eşiği, tırmanış ve temas penceresinin kontrollü faktöriyel deneylerle taranması.
5. **Sabit dönüşlü hedef modeli:** ham bbox türevi yerine işaretli `minAreaRect`, kapılı constant-turn/IMM kestirimi ve sınırlı feed-forward.
6. **Algılayıcı sağlamlığı:** farklı hedef, arka plan, ışık, bulanıklık ve kısmi örtülme koşullarında veri toplama ve yeniden eğitim.

Başarı yalnızca minimum mesafe ile ölçülmeyecektir. Gerçek CPA dağılımı, ağır kuyruk, temas/titreşim imzası, irtifa iptali, devralma kararlılığı, işlem süresi ve güvenli manuel dönüş birlikte raporlanacaktır.

## Ayrıntılı belgeler

- [Yarışma ve gerçek araç paketi](yarışma/README.md)
- [Güncel araştırma durumu](AUTORESEARCH_DURUM.md)
- [LOS ve donanım notları](NOTLAR_LOS_DONANIM.md)
- [Test ve deney kayıtları](TO_TEST.md)
- [Donanım kılavuzu](donanim/README.md)
- [Gimbal notları](NOTLAR_GIMBAL.md)
- [Güdüm günlük sözlüğü](guidance_allstar/LOG_SOZLUGU.md)
- [Açık kaynak yayın prosedürü](PUBLIC_RELEASE.md)
- [Üçüncü taraf lisans ve atıfları](THIRD_PARTY_NOTICES.md)
- [Veri ve mahremiyet politikası](DATA_POLICY.md)

## Açık kaynak yayın durumu

Depo 13 Ağustos 2026 tarihinde açık kaynak olarak yayımlanmıştır. Yayın kapsamında şu parçalar tamamlanmıştır:

- Özgün proje kodu için MIT lisansı ve üçüncü taraf atıf kaydı
- CI ile doğrulanan, sürümü sabitlenmiş Python bağımlılıkları
- Katkı, güvenlik, issue ve pull request şablonları
- Çevrimdışı algoritma testleri ve public-yayın veri denetimi
- Simülasyon ve gerçek donanım için yeniden üretilebilir çalışma tarifleri
- Ham gerçek uçuş telemetrisini dağıtım dışında bırakan veri politikası

Yayın kapısı ve sonraki sürümler için uygulanacak denetimler [`PUBLIC_RELEASE.md`](PUBLIC_RELEASE.md) içinde korunmaktadır. Simülasyon senaryoları, algı modelleri, farklı hava aracı entegrasyonları ve güvenli kontrol üzerine katkılar kabul edilmektedir.

---

Bu çalışma bir araştırma ve yarışma platformudur. Simülasyondaki güçlü sonuçlar gerçek uçuş riskini ortadan kaldırmaz; gerçek sistem her zaman kademeli doğrulama, bağımsız pilot gözetimi ve fiziksel emniyet sınırlarıyla kullanılmalıdır.
