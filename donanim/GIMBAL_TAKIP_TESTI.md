# GIMBAL SÜREKLİ HEDEF TAKİBİ — MASA TESTİ (Aziz Başar için)

Bu doküman, gerçek dronedeki servo gimbalin **mor bir hedefi kendiliğinden
takip etmesini** masada test etmek içindir. Bu dosyayı bir Claude/agent'a
verip "bu testi yürüt" diyebilirsiniz — dosyanın sonunda agent için
talimat var.

## Bu test NE DEĞİL (önce bunu bil)

* **MPC/güdüm ÇALIŞTIRILMAZ.** Takip mekanizması görüş tarafındadır
  (`tools/gimbal_bench_takip.py`), `mpc_gudum` bu teste girmez.
* **Arm edilmez, pervane takılmaz.** Drone masada, elle tutulur durumda.
* Tek süreç, tek script; ROS/Redis/sim GEREKMEZ.

## Mimari (30 saniyede)

```
kamera karesi → mor HSV tespiti → dikey hata (ey)
   → TiltTakip yasası (süzgeç + 45°/s hız sınırı + kelepçe + kayıp politikası)
   → MavlinkTiltKomutcu (DO_MOUNT_CONFIGURE + DO_MOUNT_CONTROL)
   → ArduPilot mount → servo
```

Aynı yasa simülasyonda kanıtlandı: hedef dikeyde taşındığında tilt
9°→2°→28°→2° izledi ve hedefi ham pikselde tam merkezde tuttu.
Stabilizasyonu (gövde yatınca kameranın sabit kalması) ArduPilot'un kendi
mount sürücüsü yapar — script yalnız "nereye bak" der.

## Mevcut kalibrasyon durumu (2026-08-07)

* Servo kanalı **reversed** konvansiyonunda (MP'deki tik işareti) —
  BUNU DEĞİŞTİRME, kalibrasyon buna göre yapıldı.
* `MNT1_PITCH_MIN = -38`, `MNT1_PITCH_MAX = +58` (gerçek mekanik uçlar).
* Komut 0° = kamera ufka paralel OLMALI. Teste başlamadan doğrula:
  MP Payload Control'dan 0 gönder → kamera düz bakmalı; +20 gönder →
  ~20° yukarı (telefon açıölçeriyle). Tutmuyorsa ÖNCE bunu düzelt
  (MNT1_PITCH_MIN/MAX'ı "açı = (PWM − düz_PWM)/8.89" ile yeniden türet).

## Adım adım

### 0. Depoyu güncelle
```bash
git pull    # gimbal dalı; tools/mavlink_tilt.py ve gimbal_bench_takip.py gelmiş olmalı
```

### 1. MAVLink bağlantısını çöz (ACM sorunu buraya)

Otopilot USB ile bağlıysa cihaz `/dev/ttyACM0`'dır. Bağlanamama için
sırayla:

1. Cihaz var mı: `ls /dev/ttyACM*` (yoksa kabloyu/portu değiştir,
   `dmesg | tail` ile tak-çıkar sonrası ne göründüğüne bak).
2. **İzin**: `sudo usermod -aG dialout $USER` → oturumu kapat-aç
   (en yaygın sebep budur; hata "Permission denied" ise kesin bu).
3. **Portu başka süreç tutuyor mu**: Mission Planner/QGC açıksa seri portu
   O tutar, ikinci süreç bağlanamaz. Çözüm — mavproxy ile böl:
   ```bash
   mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 \
       --out=udp:127.0.0.1:14601 --daemon
   ```
   ve script'lere `--baglanti udpin:127.0.0.1:14601` ver. (MP'ye de
   ayrı bir `--out` eklersen ikisi aynı anda çalışır.)
4. Ubuntu masaüstünde **ModemManager** ACM'yi kapabilir:
   `sudo systemctl stop ModemManager` (kalıcısı: `sudo apt remove modemmanager`).
5. Doğrudan bağlanacaksan adres olarak `/dev/ttyACM0` da geçerlidir:
   `--baglanti /dev/ttyACM0`.

### 2. Sweep testi (komut zinciri kanıtı, ~30 sn)
```bash
python3 tools/mavlink_tilt.py --baglanti <adres> --kanal 5
```
(`--kanal` = Mount1Pitch atanan servo çıkışı; bizde 5.)
Beklenen: 0/+30/−30 komutlarında servo fiziksel hareket + "HAREKET VAR".
Bu SITL'de doğrulandı (PWM 1500/1765/1233 birebir doğrusal).

### 3. Takip demosu (asıl test)
```bash
python3 tools/gimbal_bench_takip.py \
    --kaynak <kamera> --baglanti <adres> \
    --tilt-alt -35 --tilt-ust 55 --goster
```
* `--kaynak`: Pi üzerinde koşuyorsa `0`; `cv2.VideoCapture(0)` Pi'nin
  libcamera yığınında açılmazsa görüntüyü UDP'ye bas ve URL ver:
  `rpicam-vid -t 0 --inline --width 1280 --height 720 --codec h264 -o udp://127.0.0.1:8554`
  → `--kaynak udp://127.0.0.1:8554`. (Takım görüntüyü zaten başka yolla
  akıtıyorsa o URL'yi kullan.)
* Mor/eflatun bir cisim tut (HSV H 140-160 bandı — sim hedefiyle aynı).
* **Başarı ölçütü**: cismi yukarı/aşağı taşıdığında servo ~yarım saniye
  içinde izler ve cisim görüntüde dikeyde merkeze oturur. Cismi saklarsan
  3 sn bekler, sonra yavaşça 0°'ye döner.
* Çıkış Ctrl-C — script gimbali 0'a bırakıp çıkar.

### 4. (Bonus) Stabilizasyon kontrolü
Takip çalışırken droneu elinle burnundan ±20° yatır: görüntüdeki ufuk
yerinde kalmalı (bunu ArduPilot yapar, script değil). Kalmıyorsa mount
stabilizasyon parametresi kapalıdır — aşağıdaki SORULAR bölümüne yazın,
ArduPilot sürümünüze göre parametreyi söyleyelim.

## Bilinen tuzaklar (simülasyonda/SITL'de ölçüldü)

* `DO_GIMBAL_MANAGER_PITCHYAW` ACK'lense bile servo oynatmayabilir —
  script bu yüzden `DO_MOUNT_CONFIGURE(MAVLINK_TARGETING)` +
  `DO_MOUNT_CONTROL` kullanır. MP'nin payload kontrolüne dokunursanız
  mount modu değişebilir; scripti yeniden başlatmak yeter.
* Seri porta aynı anda İKİ süreç recv yapamaz (pymavlink thread-safe
  değil) — bu yüzden mavproxy bölücüsü önerilir.
* Servo 3M bantla duruyor: sarsıntıda sıfır kayarsa "0 komut = düz bakış"
  kontrolünü tekrarla.

---

## AGENT'A TALİMAT

Sen bu testi yürüten agent'san: yukarıdaki adımları sırayla uygula,
her adımın ÇIKTISINI (komut + gözlem) kısa notlar hâlinde kaydet.
Bir adım beklendiği gibi gitmezse kendi başına mimariyi değiştirme —
özellikle `tools/mavlink_tilt.py`, `tools/gz_gimbal.py` ve MNT/SERVO
parametre konvansiyonunu (reversed + −38/+58) DEĞİŞTİRME.

**Sorun ya da sorun olursa bize sor**: soruyu bu dosyanın altına
`## SORULAR` bölümü açıp yaz (tarih + gözlenen çıktıyla birlikte),
commit'leyip push'la ya da tzi4'e ilet. Bu depoyu yazan agent (gimbal
dalının sahibi) soruları oradan görüp cevaplayacak. Belirsiz kaldığın
HER durumda sormak, tahmin etmekten iyidir.
