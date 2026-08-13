# NOTLAR_LOS_DONANIM — güncel sade uçuş hattı

Güncel karar: konumlu güdüm hedefin arkasındaki slotu kurar; ardından art
arda 5 taze ve yeterince büyük bbox görülünce görüntülü yetki tek
`TerminalLosKontrolcu` yasasına geçer. Görsel fazda MPC yoktur. MPC kodu A/B
ve geri dönüş için korunur, donanım varsayılanı değildir.

## Simülasyon — her komut tek satır

Ortamı RoboFly ve video kaydıyla başlat:

```bash
cd /path/to/savasan_iha_yildizlar && YILDIZ_VIDEO=1 YILDIZ_VIDEO_ETIKET=los_robofly YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 YILDIZ_DEVIR_SOGUMA_S=3 ./yildizlar_gudum.sh --robofly
```

Hedefi elipse sokup avcıyı 60 metreye kaldır:

```bash
cd /path/to/savasan_iha_yildizlar && python3 tools/gorev_baslat.py --drones 1 --drone-alt 60 --plan missions/hedef_elips.plan
```

Konumlu yaklaşımı çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar/guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 4
```

Tek LOS görüntülü güdümü çalıştır; geçiş kararı ortam başlarken verilen env değerlerinden gelir:

```bash
cd /path/to/savasan_iha_yildizlar/guidance_allstar && python3 terminal_los_gudum.py
```

Tam otomatik 600 saniyelik elips testi:

```bash
cd /path/to/savasan_iha_yildizlar && YILDIZ_DRONE_MODEL=robofly YILDIZ_GECIS_BUYUK_KARE=5 YILDIZ_GECIS_ALAN_PCT=3 YILDIZ_DEVIR_SOGUMA_S=3 SURE=600 KONTROL_BEKLE_S=20 GORUNTULU="terminal_los_gudum.py" PLAN=missions/hedef_elips.plan METOT=los_big5a3 tools/senaryo.sh
```

Ortamı düzgün kapatıp MP4 başlığını tamamla:

```bash
cd /path/to/savasan_iha_yildizlar && ./yildizlar_gudum.sh --stop
```

## Gerçek donanım — önce salt izleme

Donanımda çalışmış referanslar değiştirilmedi: `yarışma/gimbal_bench_takip.py`,
`yarışma/mavlink_tilt.py` ve `yarışma/mpc_komut_izle.py`. Bunlardan doğrulanan
IMX500 1280×720/180° dönüş, HSV, moment merkezi, servo rampası ve
`MAV_CMD_DO_MOUNT_CONTROL` davranışları ana donanım araçlarına taşındı.

Gimbal ve kamerayı komut vermeden masa testinde çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar && python3 yarışma/gimbal_bench_takip.py --kaynak picam --kuru --goster
```

Donanımda çalışmış salt gözlemciyle gerçek LOS çıktısını aracı sürmeden izle:

```bash
cd /path/to/savasan_iha_yildizlar && python3 yarışma/mpc_komut_izle.py --yasa los --menzil-m 20 --n-pn 4 --vur-ivme 4
```

Servo mount kanalını araç sabit ve pervaneler güvenliyken tara:

```bash
cd /path/to/savasan_iha_yildizlar && python3 yarışma/mavlink_tilt.py --baglanti udp:127.0.0.1:14554 --kanal 9
```

Gerçek kamera, dedektör, Redis ve gimbali başlat:

```bash
cd /path/to/savasan_iha_yildizlar && python3 donanim/kamera_kopru.py --kaynak picamera2 --genislik 1280 --yukseklik 720 --hfov 66 --dedektor yolo --yolo-model MODEL.rpk --yolo-conf 0.70 --mavlink udp:127.0.0.1:14554 --kaydet
```

LOS hattını araç komutu göndermeden sınama:

```bash
cd /path/to/savasan_iha_yildizlar && python3 donanim/gudum_tek_dugum.py --gudum los --buyuk-kare 5 --alan-pct 3 --dry-run --sure 60
```

Geofence, irtifa tabanı, manuel override ve kuru log doğrulandıktan sonra aynı hattı gerçek komutla çalıştır:

```bash
cd /path/to/savasan_iha_yildizlar && python3 donanim/gudum_tek_dugum.py --gudum los --buyuk-kare 5 --alan-pct 3 --loop-hz 20 --menzil-kaynak estimator
```

## Geçiş kuralı

`--alan-pct 3`, 1280×720 kadrajda bbox alanı en az 829 px² demektir. Beş
ardışık taze kare yaklaşık 20 Hz dedektörde 0,25 s, 30 FPS'te 0,17 s sürer.
Merkez koşulu yoktur; LOS ilk anda yanal hatayı kapatır. Hedef menzili geçiş
kapısında kullanılmaz. YOLO `--yolo-conf` altındaki kutuları hiç yayımlamadığı
için beş kare sayacı yalnız yüksek-confidence tespitleri görür.

Önemli sınır: geçiş yalnız görüntüyle yapılır, fakat mevcut LOS/PN yasası
`menzil_m` değerini PN kazancı, `t_go`, dikey çözüm ve ıska bırakma için hâlâ
kullanır. Gerçekte hedef telemetrisi güvenilmezse sıradaki araştırma işi bbox
ölçeği/optik büyüme ile menzil kestirimi veya ayrı bir mesafe sensörü füzyonudur.

Gimbal servo açısı bağımsız olarak okunamıyorsa kamera köprüsü son komutu açı
vekil değeri sayar. Bu masa takibi için yeterli olabilir, sert uçuş için
kanıt değildir. İlk uçuşlarda gimbal durumunu encoder/pot, kamera IMU'su veya
ölçülmüş servo gecikme modeliyle doğrulamak gerekir.

## Güncel ayarlar ve kabul ölçütü

- Geçiş: 5 ardışık büyük bbox, alan eşiği `%3 × %3`.
- Görüntülü yasa: LOS/PN, `N=4`, VUR ivmesi `4 m/s²`.
- Yanal hız komutu ArduPilot tarafından roll/pitch'e çevrilir; yaw-rate
  varsayılan açık kalır. Yaw deneyi için `terminal_los_gudum.py --no-yaw`
  veya donanımda `gudum_tek_dugum.py ... --no-yaw` kullanılabilir.
- DON: menzil `3 m` veya `t_go≤0,25 s`.
- Bırakma: bbox kaybı/tespit penceresi veya geçiş sonrası açılma.
- Başarı: gerçek CPA, `<5 m` oranı, `vurus_basarili`, hedefe yakın vibe,
  `ALTITUDE ABORT`, kadraj tazeliği ve fiziksel çöküş birlikte değerlendirilir.

2026-08-11 RoboFly/elips kampanyasında N=4 için 6 angajmanın tamamı `<5 m`,
gerçek CPA medyan/p90 `1,09/2,03 m` ve iki ayrı başlangıçta iki fiziksel temas
ölçüldü. N=5 karşı kolu 12 angajmanda `1,65/3,76 m`, `11/12 <5 m`, sıfır
temas verdi. Bu yüzden N=4 tutuldu. Temas sonrası araç çöktüğü için sonraki
satırlar performans havuzuna alınmadı. Ayrıntılı karar `TO_TEST.md` içindeki
**CANLI T3 HÜKMÜ**ndedir.
