# NOTLAR_GIMBAL — Dikey (tilt) gimbal dalı

**Dal:** `gimbal` (2026-08-05). Kamera artık gövdeye sabit değil: yalnız dikey
eksende dönen, **kendini stabilize eden** gerçek bir gimbala bağlı. Motivasyon:
kadraj kayıplarının bir kısmı fiziksel FOV dışında — yazılım gimbali pikseli
döndürür ama fiziksel FOV'u döndüremez.

## Ne kuruldu

Hazır parçalar (sıfırdan gimbal YAZILMADI):

1. **Model:** `gimbal_small_2d` (SwiftGust/ardupilot_gazebo'dan indirildi)
   → `~/ardupilot_gazebo/models/gimbal_small_2d/`.
   İndirme: `https://raw.githubusercontent.com/SwiftGust/ardupilot_gazebo/master/models_gazebo/gimbal_small_2d/{model.config,model.sdf,meshes/{base_arm,base_main,tilt}.dae}`
   Kurulu kopyadaki ayarlar:
   - `tilt_joint` limitleri ±90° (orijinal −0.1..π)
   - **çarpışma geometrileri ORİJİNAL halinde AÇIK** (kullanıcı şartı: araca
     çarpma testleri için). Yere değmesinler diye montaj z=0.18'e alındı.
   - eklemdeki `implicit_spring_damper` + `damping` KALDIRILDI — ikisi ODE
     eklem motorunu kullanıyor ve plugin'in hız-servosunu her adımda hız=0
     ile ezip eklemi kilitliyordu (ölçüldü)
   - kütleler: base 0.2→0.005 kg, tilt 0.01→0.002 kg (0.21 kg burun yükü
     kontrol otoritesini bozuyordu — kullanıcı bildirdi). Ataletler 1e-5
     (daha küçüğü ODE kısıt kondisyonunu bozuyor, aşağıda).
   - yerleşik 640×480 kamerası kapalı (`always_on=0`) — asıl kamera webcam
   - **görselleri (mesh/silindir) kaldırıldı** (2026-08-05, kullanıcı isteği):
     stabilize kamera gövdeye göre dönünce (frenlemede burun yukarı → eklem
     aşağı) gimbal çatal kolları kadraja giriyordu. Kameraya %100 görünmez
     (−0.9 rad aşırı açıda bile koyu piksel %0 ölçüldü); ÇARPIŞMALAR DURUYOR,
     GUI'de de gimbal artık görünmez.
2. **Plugin:** `~/ardupilot_gazebo/src/GimbalSmall2dPlugin.cc` Gazebo 11'e
   portlandı ve **iki büyük değişiklik** yapıldı:
   - **STABILIZE modu** (`<stabilize>1</stabilize>`): komut, kameranın
     DÜNYA-çerçevesi pitch'i (elevasyon). Gövde ne kadar yatarsa yatsın
     eklem otomatik telafi eder — gerçek gimbal davranışı. `<camera_axis>`
     kamera optik ekseninin tilt_link yerel çerçevesindeki yönü (montajımızda
     `0 1 0`). stabilize=0 ile eski davranış (eklem açısı gövdeye göre).
   - **HIZ-SERVO kontrol** (tork-PID değil): ODE eklem motoru,
     `vel = clamp(kv·hata, ±vel_max)`, tork sınırı `fmax`, ölü bant.
     Varsayılanlar: kv=150, vel_max=6 rad/s, fmax=0.15 N·m, deadband=0.003 rad.
     SDF: `<servo_kv> <servo_vel_max> <servo_fmax> <servo_deadband>
     <initial_angle>`.
   - Derleme: `cd ~/ardupilot_gazebo/build && cmake .. && make GimbalSmall2dPlugin`
3. **Entegrasyon:** `models/suru_drone_{1..5}/model.sdf` — gimbal include
   (poz `0.35 0 0.18`, yaw −90° → tilt ekseni gövde Y'si), `cam_link`
   (webcam aynen) tilt pivotuna kaynaklı (`0.35 0 0.20`).

## Kontrol arayüzü

- komut : `/gazebo/default/iris-N/gimbal_tilt_cmd` (GzString, rad,
  **DÜNYA elevasyonu**, pozitif = yukarı; örn. `gz topic -p ... -m 'data: "0.3"'`)
- durum : `/gazebo/default/iris-N/gimbal_tilt_status` (rad, ~25 Hz'te bir örnek
  her 21 fizik adımında; kameranın GERÇEK dünya elevasyonu)
- Topic adı dünyadaki sarmalayıcı model adından gelir (plugin üst modele
  bağlanır): `worlds/*.world` → `iris-1..5`. Beş drone bağımsız.
- Doğruluk: ölü bant nedeniyle ±0.17° bandında oturur; izleme hızı 6 rad/s.

## Doğrulama (2026-08-05, hepsi headless, hepsi PASS)

1. **SITL uçuş testi** (`python3 tools/gimbal_ucus_test.py`, karar testi —
   kullanıcı ölçütü "gaza bas, pencere sabit kalsın"):
   GUIDED 30 m + 15 m/s ivme (10 s) + fren (5 s). Gövde pitch **−35.4..+35.2°**
   savrulurken kamera dünya pitch'i **max |0.65°|, p95 0.43°**. Sabit seyirde
   (gövde −25°) kamera 0.15°'de kilitli. Uçuş kararlı, failsafe yok.
   Baseline (gimbal'siz, aynı senaryo) da temiz — script `UCUS_STAB_BASELINE=1`
   ile gimbal'siz kıyas koşusu yapabiliyor.
2. **Sallanan platform** (fiziksel stabilizasyon izolasyonu): platform ±19°
   sallanırken kamera |p95| 1.37°.
3. **Optik test** (`python3 tools/gimbal_headless_test.py`): kırmızı referans
   kutusu 0.25 rad tiltte 251 px kaydı (beklenen 257, %2 içinde); kare akışı
   canlı; eklem komut takibi <0.06 rad.
4. **Sürü:** 5 gimbal bağımsız (iris-1..5 ayrı topic, ayrı hedefler).
5. **Yer:** 30 sn'de kayma 0.1 mm; gimbal çarpışma geometrisi AÇIK ama hiçbir
   şeyle temasta değil (montaj z=0.18 sayesinde).

## Ayar tarihçesi / TUZAKLAR (hepsi ölçülerek bulundu)

- **Tork-PID çalışmıyor:** yumuşak kazançta gövde salınımı kameraya geçiyor
  (±19'un ±15'i), sert kazançta tepki torku taşıyıcıyı sallayıp ikisi birlikte
  osilasyona giriyor. Hız-servo (ODE motor) mimarisi şart.
- **`implicit_spring_damper`/`damping` + `SetParam("vel")` çakışır:** Gazebo
  ikisini aynı ODE motoruyla uygular; sönümleme aktifken hız komutun her adımda
  ezilir, eklem kilitlenmiş görünür.
- **Minik atalet zinciri EKF'yi öldürür:** 1.5 kg gövdeye kaynaklı 2-10 g'lık
  linkler ataleti 1e-5'in altına inince ODE mikro-titreşim üretiyor → SITL
  yerde "Gyros not calibrated", ARM imkânsız. cam_link ataleti 1e-4'te,
  gimbal linkleri 1e-5'te tut.
- **fmax dengesi:** 0.3 N·m + 0.001 atalet → kalkışta takla (AngErr=171).
  0.02 → geçişlerde 22° sapma (ivme sınırlı). 0.15 + 1e-4 atalet → uçuş
  kararlı VE geçişler <0.65°.
- **Ölü bant şart:** kv=150 ölçüm jitter'ını sürekli hız komutuna çevirip
  gövdeye titreşim basıyordu (arm engeli). 0.003 rad bandı çözdü.
- **Kamera tembel render:** gazebo_ros kamerası abone yokken render etmiyor;
  tek-kare çek-bırak bayat kare döndürür. Sürekli abone kal (bbox_to_redis
  zaten öyle).
- **`gz topic -e` + `timeout`:** düşük hızlı topic'lerde stdout tamponu
  boşalmadan ölür → boş dosya. `-d <saniye>` bayrağını kullan.

## FAZ C — DİNAMİK TILT TAKİBİ (2026-08-06, VARSAYILAN AÇIK)

Tilt artık sabit değil: bbox her karede hedefin ölçülen dünya yükselişini
(−ey; canlı-eklem zinciri sayesinde tilt'ten BAĞIMSIZ ölçülür — yani bu
kapalı bir geri-besleme değil, ölçülen büyüklüğün süzgeçli takibi) izler.

Parçalar:
1. **`gz_tilt_pub`** (gimbal_kurulum/src, C++): kalıcı transport bağlantılı
   komut köprüsü. `gz topic -p` yayın başına ~1 s ödüyordu; terminalde
   eps = asin(down/r) son saniyelerde 40-70 °/s değişir, köprüyle yayın ~ms.
   `kur.sh` derler; yoksa TiltKomutcu otomatik gz-CLI yedeğine düşer (Faz A
   davranışı).
2. **`TiltTakip`** (tools/gz_gimbal.py): EMA süzgeç (tau 0.4 s) + slew
   sınırı (60 °/s) + kelepçe [−30°, +60°] + kayıp politikası (3 s tut →
   10 °/s ile standoff açısına dön = yeniden-edinim pozu). Birim testli.
3. **Güdüme canlı ε:** `tracker_bbox_stab`'a 8. eleman (o karede kullanılan
   kamera elevasyonu) → `Olcum.tilt_deg` → `mpc_gudum._kadraj_sabiti`
   `ey_ref = −(tilt_canlı + aim)` (alan yoksa statik YILDIZ_TILT'e düşer;
   eski kayıtlar ve tilt-kapalı mod kırılmaz). Guidance CSV'ye `tilt_deg`
   kolonu eklendi.
4. Kapatma düğmesi: `bbox_to_redis --tilt-sabit` (Faz A: sabit standoff tilt).

Doğrulama (2026-08-06):
- Köprü rampası: 0→34.4° / 2 s, ortanca takip hatası 0.86°.
- Hareketli hedef (mor kutu dikeyde taşındı): tilt 9.09→**2.0**→**28.2**→2.0
  hedefi izledi; tilt 28°'deyken HAM piksel ortancası 356 (merkez 360) —
  hedef fiziksel olarak kadraj merkezinde; tespit geçişlerde %95-100.
- Eklem zinciri tutarlılığı maks 0.0005°.
- Offline: mpc 86/86, takip 79/79, statik testler tam; `_kadraj_sabiti`
  canlı tilt birim testi (tilt 23.7 → ey_ref −23.7).

DİKKAT — düzeltilen tuzak: bbox `--down` argparse varsayılanı 13'te
kalmıştı (eski +30 montaj değeri); yanlış varsayılan tilt'i 27.5°'te
başlatıp hedefi FOV dışına atıyordu ve takip hiç EDİNEMİYORDU (tespit
yoksa takip başlayamaz — edinim pozu standoff geometrisine güvenir).
Varsayılan 4'e çekildi (YILDIZ_DOWN_TASARIM ile aynı); asıl kaynak yine
standoff_geom.sh.

## UYARILAR / bilinen sınırlar

- **ÖLÇÜM ZİNCİRİ:** `yildizlar_gimbal.py` de-rotasyonu kamerayı gövdeye sabit
  varsayar. Artık kamera stabilize: gövde pitch'i görüntüye HİÇ yansımıyor,
  yani mevcut de-rotasyon fazla düzeltme yapar. Güdüm tilt komutu vermeye
  başlamadan zincir güncellenmeli: kameranın gerçek elevasyonu =
  `gimbal_tilt_status` (dünya çerçevesi, hazır veri). Değişecek dosyalar:
  `yildizlar_gimbal.py` + `bbox_to_redis.py` + `tools/montaj_ayarla.py`.
- **`tools/montaj_ayarla.py` TARİHSEL oldu (2026-08-05).** Yazma yolu
  (`--uygula`/`--geri-al`) kapatıldı: SDF sensör pozuna yazmak, gimbalin
  komutladığı dünya elevasyonunun üstüne SESSİZ bir ofset bindirir. SDF `cam`
  pozu **0 kalmalı** ve hiçbir araç ona yazmamalı. Yerine
  **`tools/tilt_ayarla.py`**: yalnız `scripts/standoff_geom.sh` içindeki
  `YILDIZ_DOWN_TASARIM`/`YILDIZ_BACK`'i günceller, `YILDIZ_TILT =
  atan(down/back)`'i raporlar, SDF pozunun 0 olduğunu **doğrular** (0 değilse
  uyarır) ve terminal kadraj bütçesini basar (down/back için eps=asin(down/r)
  dikey yarı-FOV 20.07°'yi hangi menzilde aşıyor → Faz C devralma menzili).
- Ölçüm araçlarının yeni ölçütleri: `tools/gimbal_kanit.py` iki katmanlı
  (fiziksel: |corr(ham_ey,pitch)|<0.3; yazılım: ex·sin(roll) sızıntısı
  ham→stab azalmalı) + tilt zinciri sağlık raporu; `tools/gimbal_zaman_kalibre.py`
  artık pitch yerine roll sızıntısını minimize ediyor (pitch sinyali kalmadı).
  Her ikisi de yeterli tutum uyarımı yoksa "VERİ YETERSİZ" deyip 2 ile çıkar.
- Tilt yaw'ı çözmez (yatay kadraj airframe yaw'ıyla korunmalı).
- `~/ardupilot_gazebo` proje deposunun DIŞINDA — bu dosya oradaki
  değişikliklerin tek kaydı.
