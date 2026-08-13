# Yarışma kodları — uçuş paketi

Bu klasör, yarışmada ve gerçek araç üzerinde kullanılacak son görüntülü güdüm
hattını tek yerde görünür kılar. Güncel varsayılan yasa
`terminal_los_gudum.py` içindeki **Terminal LOS/PN** denetleyicisidir:
`N=4`, vurma ivmesi `4 m/s²`, geçiş için art arda `5` büyük/taze bbox ve
`%3 × %3` alan eşiği. MPC silinmemiştir; yalnız A/B karşılaştırması ve geri
dönüş seçeneği olarak korunur.

## Klasördeki dosyalar

| Dosya | Görevi | Kaynak / durum |
|---|---|---|
| `kamera_kopru.py` | IMX500/USB kamera → tespit → Redis → gerçek gimbal | Güncel `donanim/kamera_kopru.py` uçuş kopyası |
| `gudum_tek_dugum.py` | Angajman kararı, LOS/PN, emniyet ve MAVLink hız komutu | Güncel `donanim/gudum_tek_dugum.py` uçuş kopyası |
| `terminal_los_gudum.py` | Yarışmada kullanılacak son güdüm yasası | Güncel `guidance_allstar/terminal_los_gudum.py` kopyası |
| `gimbal_bench_takip.py` | Kamera ve servo-gimbal masa takibi | Raspberry Pi 5 + IMX500 çalışmalarından gelen donanım sürümü |
| `mavlink_tilt.py` | ArduPilot servo-mount tilt komutu ve kanal taraması | Gerçek ArduPilot 4.6.3 / servo-mount üzerinde doğrulanan sürüm |
| `mpc_komut_izle.py` | LOS veya MPC çıktısını aracı sürmeden izler | Gerçek donanım için salt-okunur kabul aracı |

Uçuş kopyaları bu committeki ana kaynaklarla birlikte dondurulmuştur. Ortak
bağımlılıklar (`guidance_allstar/`, `tools/`, `bbox_to_redis.py` ve
`yildizlar_gimbal.py`) depo kökünden kullanılır; bu yüzden komutları depo
kökünde çalıştırın.

## Raspberry Pi 5 çalışmaları

Bu hattın bazı parçaları Raspberry Pi 5 ve Raspberry Pi AI Camera (IMX500)
üzerinde geliştirildi. Sahada ortaya çıkan şu ayrıntılar koda işlendi:

- Pi 5/libcamera hattında OpenCV `/dev/video0` yerine `Picamera2` kullanımı;
- 1280×720 görüntü, 66° yatay görüş açısı ve ters montaj için 180° dönüş;
- gerçek görüntüde çalışan HSV bandı ve titreşimi azaltan moment merkezi;
- EMAX ES08MD için rampalı servo komutu;
- ArduPilot servo mount'ta çalışan `MAV_CMD_DO_MOUNT_CONTROL` yolu.

Bu nedenle klasörde hem Pi 5 üzerinde kullanılan donanım referansları hem de
onlardan ana hatta taşınan son yarışma kodları bulunur.

## Uçuş sırası

Önce kamera, Redis ve gimbali başlatın:

```bash
python3 yarışma/kamera_kopru.py \
  --kaynak picamera2 --genislik 1280 --yukseklik 720 --hfov 66 \
  --dedektor yolo --yolo-model MODEL.rpk --yolo-conf 0.70 \
  --mavlink udp:127.0.0.1:14554 --kaydet
```

Pervaneler devre dışıyken önce kuru kabul koşusunu yapın:

```bash
python3 yarışma/gudum_tek_dugum.py \
  --gudum los --buyuk-kare 5 --alan-pct 3 --dry-run --sure 60
```

Geofence, irtifa tabanı, manuel override, telemetri, gimbal yönü ve kuru log
doğrulandıktan sonra yarışma hattını etkinleştirin:

```bash
python3 yarışma/gudum_tek_dugum.py \
  --gudum los --buyuk-kare 5 --alan-pct 3 \
  --loop-hz 20 --menzil-kaynak estimator
```

## Hazır olma durumu

Yazılım hattı yarışma konfigürasyonuyla **uçuşa hazırdır**: gerçek kamera ve
gimbal köprüsü, kendi angajman kararını veren tek düğümlü kontrolcü, güncel
LOS/PN yasası, kuru çalışma modu, bırakma/irtifa emniyeti ve loglama birlikte
hazırdır. RoboFly/elips doğrulamasında `N=4` kolu 6/6 angajmanda 5 metrenin
altına inmiş ve iki ayrı başlangıçta fiziksel temas üretmiştir.

Bu ifade yazılım hazırlığını anlatır. Her gerçek uçuş öncesinde mekanik
kontrol, kumanda override'ı, geofence, pervane güvenliği, doğru servo yönü ve
kuru kabul testi tekrar yapılmalıdır. Ayrıntılı saha sırası için
`../NOTLAR_LOS_DONANIM.md`, donanım mimarisi için `../donanim/README.md`
dosyasına bakın.
