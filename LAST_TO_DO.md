# LAST_TO_DO — LOS gerçek uçuşa geçiş ve son araştırma listesi

Son güncelleme: 2026-08-11

Bu liste çalışma sırasıdır. Bir alt başlığa geçmeden önce üst başlığın kabul
kapısı kapanmalıdır. Her değişiklik tek değişkenli A/B olarak yapılmalı;
mevcut `N=4, büyük-kare=5, alan=%3` tabanı referans koldur.

## Dondurulmuş referans

- [x] Görüntülü yasa: tek `TerminalLosKontrolcu`, `N=4`, VUR `4 m/s²`.
- [x] Geçiş: beş ardışık taze ve ≥829 px² bbox; merkez/telemetri kapısı yok.
- [x] Konumlu yaklaşım amacı: hedefin arkasındaki slotu kurmak.
- [x] RoboFly elips: 6/6 `<5 m`, CPA medyan/p90 `1,09/2,03 m`, iki temas.
- [x] RoboFly düz: 8/8 `<3 m`, CPA medyan/p90 `0,87/1,62 m`, iki temas.
- [x] Birim/kapalı-döngü: `terminal_los_test.py` 15/15 geçti.
- [ ] Değiştirilecek dosya/parametrelerden önce referans snapshot'ını, tam
  komutu, git diff'ini ve log yollarını deney kaydına yaz.

Başarı yalnız CPA değildir. Her deneyde en az şu sonuçlar birlikte yazılacak:

`CPA median/p90/p95/max · ilk-geçiş fiziksel temas · FOV'da kalma · en uzun
bbox kaybı · ex/ey p95 · yanal ivme/jerk/doyum · roll RMS/peak/frekans ·
desired↔actual roll gecikmesi · görüntü→komut latency p50/p95 · abort/crash`

### 2026-08-11 donanım bench ara kapısı

- [x] MAVProxy varsayılan 4 Hz akışının ATTITUDE/gimbal durumunu
  `125–237 ms` yaşlandırdığı bulundu; `--streamrate=20` ile gerçek akış
  `20,1 Hz`, görüntü–tutum eşleme medyan/p95 `24,5/50,5 ms` oldu.
- [x] `GIMBAL_DEVICE_ATTITUDE_STATUS` pitch'inin bu servo-mount kurulumunda
  dünya elevasyonu değil gövdeye göre eklem açısı olduğu dinamik pitch ile
  kanıtlandı; ikinci kez body-pitch çıkarma hatası düzeltildi.
- [x] Düz, `pitch=-16,7°` ve `pitch=+21,7°` duruşlarında gimbal ham dikey
  merkez hatası medyan `0,15–0,20°`; beklenen–bildirilen eklem farkı
  `0,01–0,07°` ölçüldü.
- [x] Gerçek kamera + fiziksel gimbal + Cube ile 90 s LOS output dry-run:
  `1796/1796 kuru=1`, araç STABILIZE/disarmed; `ex→yaw` ve `ey→NED-vz`
  işareti değerlendirilebilir örneklerin `%100`ünde doğru.
- [x] Balon testi için `donanim/balon_menzil.py` eklendi: hedef
  telemetrisinden kontrol yoluna yalnız `||hedef_ned-avci_ned||` çıkar;
  hedef konum/hız/yönü ve `ref_*` alanları dışarı kapalıdır.
- [x] Canlı fail-closed kapıları eklendi: sabit menzil canlıda yasak;
  LOCAL_POSITION 0,50 s, heartbeat 2,0 s veya gerçek menzil 2,0 s bayatsa
  angajman yok/setpoint yok ve mevcut angajman bırakılır.
- [ ] Microhard hedef linkini Pi `14604` endpoint'ine bağla ve ham telemetri
  menzilini ölçüm şeridi/GPS referansıyla doğrula.
- [ ] Açık havada EKF origin sonrası taze `LOCAL_POSITION_NED`/hız akışını
  doğrula; bugünkü masa koşusunda bu alanlar yoktu.
- [ ] Yeni uçak dedektörü confidence + benzersiz inference-id/timestamp
  yayımlasın; bugünkü mor dedektör yanlış mor hedefe angaje olabildi.

---

## P0 — Pervaneli görüntülü uçuş öncesi zorunlu

### P0.1 Tek komut yazarı ve zorunlu interlock

- [ ] `formation_KILLER.py` içindeki `_attacker_intercept_thread` için
  `tekdugum_durum`, `tekdugum_hayatta` TTL ve/veya tek bir otorite anahtarını
  gerçek protokol haline getir.
- [ ] Görüntülü `ANGAJE` olduğunda konumlu/formation sürecinin setpoint ve mod
  komutu göndermediğini MAVLink kayıt seviyesinde kanıtla.
- [ ] LOS süreci ölür, TTL düşer veya bbox bırakırsa konumlu katmana kontrollü
  geri dönüş/deadman davranışını kanıtla.
- [ ] Aynı araca ikinci bağımsız `SET_POSITION_TARGET` yazarı olmadığını
  router logundan doğrula.
- [ ] `formation_KILLER.py`nin repoda olmayan `config.py` bağımlılığını
  dağıtım manifestine ekle ve kullanılan gerçek sürümü arşivle.

Kabul: 30 dakikalık props-off entegrasyon koşusunda aynı 100 ms pencerede iki
farklı controller setpoint'i görülmeyecek; süreç öldürme testlerinin tamamı
beklenen güvenli moda gidecek.

### P0.2 Kamera yönü ve koordinat işareti

- [ ] Fiziksel IMX500 montajının gerçekten 180° ters olup olmadığını kaydet.
- [ ] Bench'te çalışan `hflip+vflip` davranışı ile üretim
  `donanim/kamera_kopru.py` çıktısını aynı ham karede eşleştir.
- [ ] Yedi konum için doğrudan kameradan ham 1280×720 kare al: sol-üst,
  üst-orta, sağ-üst, sol-alt, alt-orta, sağ-alt ve merkez.
- [ ] Her karede işareti doğrula: hedef sağda `ex>0`; hedef yukarıda tanımlı
  görüntü sözleşmesine göre `ey<0`; merkez `ex≈ey≈0`.
- [ ] `system_static_tests/` içindeki sekiz mevcut JPEG'in monitör fotoğrafı
  olduğunu ve intrinsics/zaman kalibrasyonu sayılmadığını kayıt altına al.
- [ ] Aynı testi `.rpk` dedektör çıktısıyla yap; bbox koordinatlarının dönüşten
  önce mi sonra mı geldiğini doğrula.

Kabul: yedi ham karede yanlış işaret sıfır; merkez biası ölçülmüş tolerans
içinde; yeniden başlatmalar arasında koordinatlar aynı.

### P0.3 Kamera kalibrasyonu ve detector sözleşmesi

- [ ] 1280×720 çözünürlük, gerçek HFOV/VFOV, `fx/fy/cx/cy` ve lens
  distorsiyonunu checkerboard/Charuco ile kalibre et.
- [ ] Modelin letterbox/ROI koordinatlarını tam görüntü koordinatına geri
  dönüşümle doğrula.
- [ ] `.rpk` confidence eşiğini gerçek dataset üzerinde precision/recall ve
  yanlış pozitif maliyetiyle seç; yalnız eski `0,70` değerini varsayma.
- [ ] Confidence'ın Redis'e taşınması mı, yoksa upstream kabulün yeterli
  olması mı istendiğine karar ver; geçişte kullanılan kanıtı logla.
- [ ] Exposure, gain, blur, rolling-shutter ve hedef boyutu değişiminde bbox
  merkez jitter'ını piksel/deg olarak ölç.

Kabul: bbox merkezi ve alanı bilinen hedef konum/boyutuyla kalibre; 5-kare
kapısının yanlış angajman oranı test setinde tanımlı eşiğin altında.

### P0.4 MAVLink ve Redis topolojisi

- [ ] Kamera köprüsü, LOS, konumlu süreç ve GCS için ayrı MAVLink çıkış
  portlarını tek şemada yaz; iki süreç aynı `udpin` portunu bağlamasın.
- [ ] MAVLink router/MAVProxy source-system kimliklerini ve hedef SysID'leri
  doğrula.
- [ ] `gudum_tek_dugum`un Redis'i `localhost`tan okuduğunu dikkate al;
  süreçler farklı bilgisayardaysa ortak Redis ve saat sözleşmesini tasarla.
- [ ] `devir_durumu.t_mono` değerinin yalnız aynı Linux monotonic saat
  alanında anlamlı olduğunu test et; makineler arası monotonic timestamp
  taşınmasın.
- [ ] Gerçekte alınan ATTITUDE, LOCAL_POSITION_NED, target telemetry ve
  VIBRATION hızlarını başlangıçta ve çalışma boyunca logla.

Kabul: her süreç doğru araç SysID'sini görür; port çatışması yok; veri hızı
asgari şartın altına düştüğünde sessizce uçmak yerine health kapısı kapanır.

### P0.5 Gimbal çalışma modu ve gerçek açı

- [ ] İlk uçuş modu seç: **önerilen ilk basamak sabit, kalibre edilmiş
  gimbal/mount**. Hareketli modu aynı uçuşta karıştırma.
- [ ] Sabit modda fiziksel kamera→gövde açısını ölç ve FC roll/pitch ile
  `SanalGimbal` işaretlerini sağ/sol bank tezgâhında doğrula.
- [ ] Hareketli mod için seçenek seç: encoder/pot, gerçekten ölçülmüş
  `GIMBAL_DEVICE_ATTITUDE_STATUS`, veya ölçülmüş komut→açı durum gözlemcisi.
- [ ] `SERVO_OUTPUT_RAW` PWM'inin mekanik açı olmadığını test planına yaz;
  stall/backlash/yük altında ayrıca gerçek açı ölç.
- [ ] Komut, PWM ve gerçek açı için step/sweep testi yap; rise time, lag,
  overshoot, backlash, saturation ve yük etkisini çıkar.
- [ ] Bench 50 Hz/120°/s ile üretim yaklaşık 10 Hz/150°/s davranışını A/B et;
  büyük basamakların LOS-rate'e etkisini ölç.
- [ ] Gimbal status/ATTITUDE okumayı ikinci `recv_match` thread'iyle değil,
  tek MAVLink receiver/dispatcher üzerinden tasarla.

Kabul: kullanılan kamera açısının gerçek açı hatası tüm uçuş zarfında
belirlenmiş sınırda; gimbal feedback bayatsa görsel komut verilmez.

### P0.6 Kamera–telemetri zaman eşleme

- [ ] Kamera capture zamanı, exposure ortası, inference bitişi, Redis yayını,
  guidance okuması ve MAVLink gönderim zamanını aynı monotonic tabanda logla.
- [ ] `tools/gimbal_zaman_kalibre.py` veya eşdeğer fiziksel sweep ile
  `--kamera-gecikme-ms` değerini ölç; varsayılan 0 ms bırakma.
- [ ] ATTITUDE'u görüntü capture anına interpolate et.
- [ ] ATTITUDE yok/bayatken üretim köprüsünün `(roll,pitch)=(0,0)` ile sessizce
  devam etmesini uçuşta yasakla; health failure üret.
- [ ] Fiilî telemetri hızını ölç; IRL arşivindeki yaklaşık 5 Hz ATTITUDE
  akışının 20 Hz dış guidance titreşimini çözmeye yetmediğini dikkate al.

Kabul: görüntü→komut latency p50/p95 biliniyor; capture-time tutum hatası ve
drop oranı logda; yapay yaw/roll faz farkı sınırlandırılmış.

### P0.7 Temiz kurulum ve bağımlılıklar

- [ ] Temiz Raspberry Pi ortamında iki giriş dosyası için import smoke testi
  çalıştır.
- [ ] `requirements.txt` ile gerçek import ağını eşleştir; eksik `filterpy`
  bağımlılığını kapat.
- [ ] `python3-picamera2`, `imx500-all`, Redis, OpenCV, pymavlink ve model
  sürümlerini sabitle.
- [ ] Repo dizin yapısını koruyarak bağımlılık manifestini paketle; yalnız iki
  giriş `.py` dosyasını kopyalama.
- [ ] Gerçek `config.py`, `.rpk`, ArduPilot parametre dump'ı ve servo mount
  parametrelerini aynı release paketine al.
- [ ] Boot sonrası Redis, model, kamera, MAVLink heartbeat ve log yazma health
  kontrollerini otomatik doğrula.

Kabul: ağsız temiz Pi'de reboot sonrası tüm süreçler başlar, dry-run logu
üretir ve eksik bağımlılıkta anlaşılır biçimde fail-closed olur.

### P0.8 Uçuş zarfı ve emniyet

- [ ] İlk IRL LOS tavanını mevcut arşivde kanıtlı `24–27 m/s` zarfının altında
  seç; güncel 35 m/s sim tavanını doğrudan kullanma.
- [ ] Bank, ivme, jerk, yükselme/alçalma ve minimum irtifa limitlerini gerçek
  aracın logundan çıkar.
- [ ] Geofence, pilot override, bağımsız kill/bırak, mode fallback ve
  telemetry-loss prosedürünü props-off ve tethered testte doğrula.
- [ ] Hedefle fiziksel temasın iki sim koşusunda aracı düşürdüğünü kabul
  ölçütüne ekle; “daha küçük CPA” emniyetin önüne geçmesin.
- [ ] VIBRATION vuruş eşiğini IRL tabanından yeniden kur. Eski ataklarda
  vibration tepe medyanı 18,5 iken simdeki `>10` eşiği gerçek vuruş kanıtı
  değildir.
- [ ] Impact kararını menzil + ivme/vibration impulse + hız düşüşü + health
  değişimi gibi çoklu kanıtla kur.

Kabul: bağımsız güvenlik sorumlusu/operatör her an komutu kesebilir; tüm
failsafe enjeksiyonları beklenen güvenli sonuçla biter.

---

## P1 — Ölçülebilir titreşim teşhisi

### P1.1 Zorunlu log kolonları

- [ ] Ham bbox merkezi/alan/confidence ve capture timestamp.
- [ ] Ham LOS, stabilize LOS, `lambda`, `lambda_dot`, `yaw_capture`,
  `yaw_now`, gimbal command/status/real angle.
- [ ] `a_yan_raw`, kelepçeli `a_yan`, ileri ivme, faz, doygunluk oranı ve
  faz değişimi nedeni.
- [ ] Gönderilen velocity setpoint ve per-step yanal ivme/jerk.
- [ ] Autopilot desired roll/rate, actual roll/rate, gyro, vibration, motor
  outputs ve saturation.
- [ ] Görüntü, inference, Redis, controller ve MAVLink zamanları.

Kabul: “komut titriyor mu, yoksa yalnız araç mı?” tek bir eşzamanlı grafikten
cevaplanabilir.

### P1.2 Katman ayırma deneyi

- [ ] Sabit hedef, düz hedef ve gimbal step için roll/roll-rate spektrumunu
  çıkar.
- [ ] `lambda_dot` ve setpoint titreşiyorsa detector/zaman/guidance koluna;
  desired roll düzgün, actual roll titreşiyorsa FC tune/mekanik/filtre koluna
  git.
- [ ] Yalnız gimbal hareket ederken oluşuyorsa gimbal–airframe görev
  paylaşımını değiştir.
- [ ] ArduPilot notch'u yalnız motor/gyro frekansı kanıtlanırsa ayarla;
  düşük frekanslı guidance salınımını notch ile gizleme.

Kabul: baskın frekans ve kök katman belirlenmeden kazanç/dead-zone değişikliği
yapılmayacak.

### P1.3 Referans tekrar sayısı

- [ ] Düz simde en az 20, tercihen 50 angajman; bağımsız yığın başlangıçları
  ve eş seed blokları.
- [ ] Elipste aynı tekrar sayısı.
- [ ] Güvenli gerçek uçuşta her hücre için en az 5 tekrar.
- [ ] Mevcut düz sekiz angajmanı ön kanıt say; nihai istatistik sayma.

Kabul: medyanla birlikte p90/p95/max ve koşular-arası dağılım raporlanır.

---

## P2 — LOS tabanını bozmadan titreşimi azalt

Değişiklikler aşağıdaki sırayla, tek tek yapılacak.

### P2.1 Capture-time yaw/LOS faz eşleme

- [ ] `pid_gudum.py`deki kanıtlı deseni terminal LOS'a taşı: yaw'ı görüntü
  capture zamanına getir ve `lambda=ex+yaw_capture` sinyalini birlikte türevle.
- [ ] Mevcut kodun `d_ex + yaw_rate_now` kolunu A/B referansı olarak koru.

Kabul: düz rota CPA/temas gerilemez; roll high-pass RMS, komut işaret
değişimi ve `lambda_dot` varyansı düşer.

### P2.2 Ölçüm kalitesine göre LOS-rate

- [ ] Bbox alanı, yaş, confidence ve gimbal hareketinden `q∈[0,1]` üret.
- [ ] Düşük `q`da P/ileri kapanmayı öldürmeden yalnız türev kazancını azalt ve
  filtre zaman sabitini büyüt.
- [ ] Bbox kaybı sonrası türev state'ini sıfırla/zayıflat.

Kabul: küçük/bayat bbox'ta sahte yanal komut azalır; yeniden edinimde spike
yok; FOV kaybı ve CPA bozulmaz.

### P2.3 Gerçek yanal jerk/constraint governor

- [ ] Ardışık döngüler arasında yanal ivme ve jerk sınırı ekle; 0,70 s
  ulaşılabilirlik zarfını gerçek per-step governor sanma.
- [ ] Ölçülmüş yaklaşık `a_max≈4 m/s²`, `tau≈1,7 s` ile başla.
- [ ] Bank/turn-rate sınırını hızla zamanla; governor'ın nominal PN'e en yakın
  uygulanabilir komutu seçmesini sağla.

Kabul: `+a_max↔−a_max` sıçramaları ve roll peak azalır; kapanma ve düz CPA
korunur.

### P2.4 Faz chatter'ını kaldır

- [ ] VUR için minimum dwell ve farklı giriş/çıkış histerezisi A/B et veya
  ikili `YERLES/VUR` yerine yanal otoriteye göre sürekli ileri-ivme karışımı
  dene.
- [ ] İleri gazı yalnız `|q|>11°/s` eşiğinde kesmek yerine
  `|a_yan_raw|/a_max` büyüdükçe sürekli azalt.
- [ ] Düz testte mevcut faz değişimi tabanını
  `8,2,7,3,12,2,14,2` olarak kullan.

Kabul: faz değişimi medyanı normal geçişte yaklaşık 2'ye yaklaşır; düz CPA
p90 `1,62 m`den anlamlı kötüleşmez.

### P2.5 Dead-zone yalnız ölçümden sonra

- [ ] Düz hedefte gerçek `lambda_dot` gürültü dağılımını çıkar.
- [ ] Gerekirse yalnız bu dağılımdan türetilmiş küçük ve yumuşak LOS-rate
  dead-zone'u A/B et.
- [ ] Geniş `ex` merkez dead-zone'u veya kör ham türev kesmesi ekleme.

Kabul: küçük gerçek manevralar kaybolmaz; dönüş CPA'sı kötüleşmez.

### P2.6 Yaw görev paylaşımı

- [ ] Yaw açık/kapalı/düşük-kazançlı FOV-yaw üçlü A/B yap.
- [ ] Roll/velocity ana yakalama kanalı; yaw yalnız hedef FOV kenarına
  yaklaşınca düşük kazanç ve hız sınırıyla yardım etsin.
- [ ] Yaw ve yanal kanalın aynı görüntü hatasını iki hızlı döngüyle
  kovalamadığını doğrula.

Kabul: hedef FOV'da kalır, yaw kapalıya göre kayıp artmaz; roll titreşimi ve
camera-yaw çapraz geri beslemesi azalır.

---

## P3 — Dönüşleri yakalama

### P3.1 Gerçek dönüş tabanı

- [ ] Düz, sağ/sol sabit `±4, ±8, ±12°/s`, daire/elips, ani yön değişimi,
  sinüs ve accelerated-escape rotaları oluştur/doğrula.
- [ ] Her hücrede arkadan, yandan ve düşey ofsetli başlangıçları dengele.
- [ ] Her hücre en az 20, tercihen 50 sim angajmanı; sağ/sol eş seed.
- [ ] Handoff anındaki menzil, kapanma, yaklaşım açısı, bbox yaşı/alanı ve
  gimbal durumu dengeli değilse yasa kıyasını geçersiz say.

Kabul: N=4'ün gerçek turn-rate→CPA eğrisi ve saturasyon başlangıcı biliniyor.

### P3.2 Gecikme kadar LOS öngörüsü

- [ ] Zaman eşleme tamamlandıktan sonra yalnız ölçülmüş gecikme kadar
  `lambda_pred=lambda+lambda_dot·delay` dene.
- [ ] Öngörüyü FOV/fiziksel turn-rate ile kelepçele; kayıpta kısa sürede
  sıfıra söndür.

Kabul: `±8/±12°/s` CPA iyileşir; düz rota ve yanlış yön aktivasyonu bozulmaz.

### P3.3 Güven kapılı constant-turn/APN

- [ ] Önce düşük dereceli, timestamp'li LOS/turn observer kur; ham bbox
  ikinci türevini doğrudan kullanma.
- [ ] Manevra birkaç ardışık yeni karede aynı işaret ve yeterli confidence ile
  görülürse sınırlı constant-turn/APN katkısı aç; confidence düşünce sıfıra
  sönsün.
- [ ] `rot_aci_deg/minAreaRect` yalnız log özelliği olarak kalsın; sağ/sol
  işareti ve menzil bağımlılığı doğrulanmadan kontrol girdisi olmasın.
- [ ] APN uçuş kapısı: düz faz yanlış aktivasyon `<%5`, doğru işaret `>%80`,
  gerçek turn-rate/ivme ile anlamlı korelasyon.

Kabul: sert dönüş CPA p90 düşer; düz CPA, FOV kaybı, jerk ve abort artmaz.

---

## P4 — Menzil, geçiş ve yeniden edinim

- [ ] 5×%3 geçişi değişmeden gerçek `.rpk` videosunda ölç; yanlış pozitif,
  geçiş gecikmesi ve bbox alan dağılımını yaz.
- [ ] Geçişin görüntü-only, terminalin menzil bağımlı olduğunu UI/logda açık
  ayır.
- [ ] Hedef telemetrisi bozulduğunda PN/t_go/dikey/ıska davranışını fault
  injection ile ölç.
- [ ] Bbox optik büyümesi (`area` veya angular size rate), bağımsız mesafe
  sensörü ve telemetri arasında uyuşmazlık kapısı araştır.
- [ ] Mutlak bbox boyunu tek başına menzil sayma; hedef görünüş/aspekt etkisini
  modelle.
- [ ] Yakın bbox kaybında ölü-hesap yalnız `R<12 m`, `<1 s`, FOV kelepçeli ve
  confidence düşen bir araştırma kolu olsun.
- [ ] ISKA/BIRAK sonrası yeniden arka-slot kurulumunu, ping-pong ve iki-yazar
  olmadan doğrula.

Kabul: yanlış yer telemetrisi terminali tehlikeli komuta sürüklemez; geçiş
performansı mevcut kapıdan kötüleşmez.

---

## P5 — MPC'den geri alınacaklar, sırayla

### P5.1 PN + constraint/reference governor — önerilen gerçek hibrit

- [ ] PN nominal ivmesini her döngüde koru.
- [ ] Aynı döngüde analitik veya küçük QP ile ivme, jerk, bank/turn-rate, hız,
  düşey ve FOV sınırına en yakın uygulanabilir komuta izdüşür.
- [ ] Solver başarısız/timeout olursa bit-aynı nominal PN fallback ver.
- [ ] Tam MPC ile değil, mevcut analitik `_erisilebilir` tabanıyla A/B et.

Kabul: kısıt ihlali ve titreşim azalır; CPA/temas ve CPU deadline gerilemez.

### P5.2 Kısa-ufuk LOS-rate/ZEM MPC — yalnız araştırma

- [ ] Durumu `[lambda, lambda_dot, R, R_dot, a_y]`, girdiyi yanal
  ivme/jerk ile sınırla.
- [ ] Ufku `T_h≤0,6–0,8·t_go` yap ve CPA sonrasını hiçbir maliyete sokma.
- [ ] Terminal maliyeti ZEM/kaçırma mesafesi olsun; post-CPA bbox alanı veya
  görüntü merkezi ödülü olmasın.
- [ ] Ölçülen `tau≈1,7 s`, `a_max≈4 m/s²` kullan.
- [ ] Önce replay, sonra kapalı-döngü sim; gerçek uçuş en son.

Kabul: constraint-governor'dan iki farklı rotada istatistiksel ve operasyonel
olarak üstün değilse üretim stackine alınmaz.

### P5.3 Tam perception-aware NMPC — son seçenek

- [ ] Yalnız P2–P5.2 başarısızsa ve eksik performans gereksinimi nicel olarak
  gösterildiyse değerlendir.
- [ ] Model belirsizliği, optimizer deadline, fallback ve donanım CPU yükü
  baştan kabul ölçütüne girsin.

Kabul: basit yöntemlerin çözemediği açık bir gereksinimi karşılamadan ek
karmaşıklık üretime girmez.

---

## P6 — Gerçek hayat test merdiveni

Sıra atlanmayacak:

- [ ] A. Props-off: temiz boot, router, Redis, camera, `.rpk`, video/log,
  yetki ve deadman fault injection.
- [ ] B. Props-off: sabit mount/gimbal yedi nokta işaret testi ve dinamik
  hedef sweep'i.
- [ ] C. Props-off: servo step/sweep; command, PWM, gerçek açı ve camera LOS
  birlikte.
- [ ] D. Sabit araç: `mpc_komut_izle.py --yasa los` ile shadow command;
  hiçbir hız komutu yok.
- [ ] E. Hover: kamera+gimbal açık, LOS yalnız dry-run; pilot hedefi tüm
  kadrajda gezdirir.
- [ ] F. Düşük hız, sabit gimbal, büyük geofence ve pilot override: yalnız
  yatay FOV takip; terminal hızlanma kapalı.
- [ ] G. Düşük hız düz hedef: sınırlı LOS, temas yok; desired/actual roll ve
  latency doğrulanır.
- [ ] H. Düşük hız dönüşlü hedef: `±4°/s`, sonra `±8°/s`; her basamakta kabul
  kapısı.
- [ ] I. Hareketli gimbal ancak açı geri bildirimi/gecikme modeli kanıtından
  sonra.
- [ ] J. Terminal kapanma/temas yalnız tüm önceki basamaklar ve bağımsız
  emniyet incelemesi geçince.

Her gerçek uçuşta zorunlu kayıt:

`ham video · detector metadata · camera CSV · LOS CSV · MAVLink tlog/bin ·
ArduPilot desired/actual attitude/rate · motor outputs · servo command/PWM/
real angle · authority transitions · operator/failsafe events`

---

## P7 — Release/operasyon

- [ ] Süreç başlangıç sırasını servis/supervisor ile sabitle:
  router → Redis → kamera/health → konumlu → LOS standby.
- [ ] LOS health hazır değilse geçişi fail-closed tut.
- [ ] Her süreç için ayrı log, rotation, disk-dolu davranışı ve ortak run-id.
- [ ] Kapanışta video başlığını tamamla, gimbali güvenli açıya, kontrol
  otoritesini güvenli moda döndür.
- [ ] Parametre dump'ı, model hash'i, git revizyonu/diff'i ve calibration
  dosyalarını her uçuş klasörüne kopyala.
- [ ] `DONANIM_GOREV.md` ve eski MPC notlarını güncel otorite olarak kullanma;
  release prosedürünü bu belge ve `NOTLAR_LOS.md`ye bağla.
- [ ] Son uçuş checklist'ini iki kişiyle doğrula: operatör + safety observer.

## Nihai karar kapısı

LOS/PN üretim tabanı olarak kalır, eğer:

- düz ve dönüş hücrelerinde CPA/temas hedefini,
- FOV ve yeniden edinim hedefini,
- roll/jerk/abort emniyet hedefini,
- gerçek zaman deadline ve failover hedefini

aynı anda sağlarsa.

PN + constraint governor, yalnız bu tabanı kısıt ihlalinde düzeltir. Tam MPC
ancak daha basit taban iki farklı rotada nicel gereksinimi karşılayamazsa ve
LOS-state MPC aynı veri üzerinde belirgin üstünlük gösterirse geri döner.
