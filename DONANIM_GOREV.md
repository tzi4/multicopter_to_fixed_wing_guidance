# DONANIM_GOREV.md — Raspberry Pi 5 + IMX500 + servo gimbal getirme görevi

Bu dosya repoyu yeni alan takım arkadaşı içindir. Amaç: simülasyonda çalışan
görüntülü güdüm zincirini (kamera → tespit → gimbal → MPC) gerçek donanıma
taşımak. **Simülasyonu (Gazebo/ROS) KURMANA GEREK YOK** — aşağıdaki işlerin
hepsi sim olmadan, masada yapılır. Branch: `gimbal`.

## Hangi kod donanıma gidiyor?

Repoyu komple klonla ama Pi'de **çalışacak** olan şunlar:

| Yol | Rol |
|---|---|
| `bbox_to_redis.py` (kök) | Tespit + karar verici + gimbal komutu. IMX500 köprün buna bağlanacak |
| `guidance_allstar/` | Güdümün beyni: `mpc_gudum.py`, `goruntulu_temel.py`, `mavlink_utils.py`, `guidance_config.py` |
| `tools/gz_gimbal.py` | Gimbal takip yasası (`TiltTakip`); Gazebo yayın kısmı entegrasyonda MAVLink'e çevrilecek |
| `scripts/standoff_geom.sh` | Takip geometrisinin tek kaynağı |
| redis-server (apt) | Süreçler arası iletişim — Pi'de kurulu olmalı |

Geri kalanı (`gimbal_kurulum/`, `models/`, `worlds/`, `missions/`,
`tools/senaryo.sh` vb.) **simülasyon tarafı** — donanıma koyulmaz,
kurcalamana gerek yok.

## Hedef mimari (gerçek donanımda)

```
IMX500 AI kamera ──> Redis "tracker_bbox" ──> bbox_to_redis.py
                                                 │  (karar verici + sanal gimbal)
                                                 ├─> Redis "tracker_bbox_stab" ──> mpc_gudum.py ──> MAVLink hız setpoint
                                                 └─> MAVLink DO_MOUNT_CONTROL ──> otopilot ──> servo (pitch gimbal)
```

Sim'de bu zincirin iki ucu farklıydı: tespit ROS kamerasından HSV renk
eşiğiyle geliyordu, gimbal komutu Gazebo topic'ine gidiyordu. **Ortadaki her
şey (kontrol yasası, stabilizasyon matematiği, Redis sözleşmesi, MPC) aynen
kalıyor.** Senin işin iki ucu gerçek donanıma bağlamak.

---

## Görev 1 — Pi 5 kurulumu (yarım saat)

```
sudo apt install redis-server python3-opencv python3-numpy
pip install pymavlink redis
# IMX500 için: Raspberry Pi OS Bookworm + picamera2 + imx500 firmware paketi
sudo apt install python3-picamera2 imx500-all
```

- ROS **kurma** — sim bağımlılığı, gerçekte kullanılmıyor.
- Pi ↔ otopilot bağlantısı: TELEM portundan UART (57600 veya 921600) ya da
  USB. `mavproxy.py --master=/dev/ttyAMA0` ile HEARTBEAT gördüğünü doğrula.

## Görev 2 — IMX500 → Redis köprüsü (asıl iş)

IMX500'ün NN tespit çıktısını alıp Redis'e sim'deki sözleşmeyle yayınlayan
küçük bir script yaz (`donanim/imx500_kopru.py` gibi). Sözleşme
(`bbox_to_redis.py` içinde tanımlı, değiştirme):

- **`tracker_bbox`** — 7 eleman: `[x, y, w, h, kapsama_pct, gecerli, t_capture]`
  - `x,y,w,h`: piksel, 1280×720 çerçevede sol-üst köşe + boyut
  - `kapsama_pct`: bbox alanının kadraja oranı (%)
  - `gecerli`: 0/1
  - `t_capture`: karenin yakalandığı an, `time.monotonic()` tabanında (saat
    farkı olmasın diye aynı Pi'de üretiliyor — kritik)
- Hedef kare hızı ≥ 30 fps; tespit yoksa da `gecerli=0` ile yayınla.

Kamera içselleri kodda zaten IMX500'e göre: 1280×720, HFOV 66°
(`tools/tilt_ayarla.py`). Farklı çözünürlük kullanacaksan önce sor.

Entegrasyon noktası (ikinci aşama, birlikte yapacağız): `bbox_to_redis.py`
içindeki `_detect` (HSV tespiti) ve ROS `Image` aboneliği senin köprünle
değişecek. Şimdilik köprünün tek başına doğru yayın yaptığını göstermen
yeterli: `redis-cli --csv subscribe tracker_bbox` ile izle.

## Görev 3 — Servo + otopilot (lehimleme sonrası)

Servoyu Pi'den DEĞİL, **otopilotun AUX çıkışından** sürüyoruz — ArduPilot'un
mount sürücüsü gövde pitch telafisini (stabilizasyonu) IMU'dan kendisi yapar,
sim'deki Gazebo eklentisinin birebir karşılığı budur.

Otopilot parametreleri (servo hangi çıkışa lehimliyse x onun numarası):

```
MNT1_TYPE        = 1      # Servo gimbal
SERVOx_FUNCTION  = 7      # mount1 pitch
MNT1_MODE        = 2      # MAVLINK_TARGETING (komutu Pi verecek)
SERVOx_MIN/MAX/TRIM       # gimbalin fiziksel limitine göre kalibre et
MNT1_PITCH_MIN/MAX        # 3D baskı gimbalin gerçek açı aralığı (derece)
```

Kalibrasyon: `SERVOx_TRIM` = kamera tam yatay bakarken; MIN/MAX'ı mekanik
limitlere dayanmadan ayarla. Açı→PWM doğrusallığını iki bilinen açıda
(örn. 0° ve 45°) açıölçerle doğrula.

**Tezgah kabul testi (uçuş yok, pervane yok):** Pi'den
`MAV_CMD_DO_MOUNT_CONTROL` ile −20°/0°/+40° komutla; sonra gövdeyi elle
±35° yatır — kameranın dünya elevasyonu sabit kalmalı (sapma < ~2°).
Sim'de bu kriter kanıtlandı (gövde ±35° iken kamera 0.65° oynadı); donanım
aynı çıtayı geçmeli. Telefonun açıölçeriyle ölçmek yeterli.

## Görev 4 — MPC çözücü benchmark'ı (OPSİYONEL, ertelendi)

İşlem süresi endişesini şimdilik erteledik; uçuş sonrası `mpc_tani_*.csv`
loglarındaki `sure_ms` kolonundan bakacağız. Vaktin artarsa
`cd guidance_allstar && python3 mpc_test.py` koşup çıktıyı bize at, yeter.

## Tuzaklar (bilinen, düşme)

1. `--no-tilt` bayrağıyla ÇALIŞTIRMA — o dal eski +30° montaj varsayımını
   sessizce uygular (argparse varsayılanı hâlâ 30).
2. `bbox_to_redis.py` gimbal modunu Gazebo model adına bakan bir kapıyla
   açıyor; gerçek donanımda bu kapı geçilemez ve gimbal **sessizce kapanır**.
   Entegrasyonda bypass edilecek (bilinen iş, koda dokunma, hatırlat yeter).
3. `TiltTakip` yazılım slew limiti 60°/s — servon bundan yavaşsa söyle,
   limiti servoya göre düşüreceğiz.
4. Gimbal komutunun anlamı **dünya elevasyonu** (yatay = 0, yukarı +).
   Gövdeye göre açı DEĞİL — telafiyi otopilot yapıyor. Test ederken şaşırma.

## Ne hazır, ne değil (özet)

| Parça | Durum |
|---|---|
| Kontrol yasası, stabilizasyon matematiği, MPC, Redis sözleşmesi | Hazır, dokunulmayacak |
| Gimbal komutunun MAVLink'e çevrilmesi (`tools/gz_gimbal.py` → `TiltKomutcu._yayinla`) | ~10 satır, entegrasyon günü birlikte |
| IMX500 → Redis köprüsü | **SENDE (Görev 2)** |
| Servo + otopilot param + tezgah testi | **SENDE (Görev 3)** |
| Pi'de MPC benchmark | **SENDE (Görev 4)** |
