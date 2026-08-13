# TO_TEST_TAKIP — ArduPilot takip yasası (`takip_gudum.py`) deney sırası

Oturum: 2026-08-05. Kapsam: **yalnız `ardupilot-takip` kolu.** MPC'nin deney
listesi ayrı dosyada (`TO_TEST.md`) ve **oraya dokunulmadı**.

## SAHİPLİK KURALI (bu dosyanın varlık sebebi)

Kullanıcı kuralı: *"MPC'ye ait olabilir çok az ihtimal bile diyorsan bırak."*

Bu dosyadaki her madde **yalnızca `guidance_allstar/takip_gudum.py` ya da
`takip_test.py` içinde yapılacak bir değişikliktir**. Bu, sahipliği tanım
gereği tekilleştirir: MPC'nin o dosyalarda hiçbir şeyi yok. Bir maddenin
ALTINDAKİ FİKİR MPC'ye de yarayabiliyorsa bunu açıkça yazıyorum ama
**maddeyi sahiplenmiyorum** — MPC tarafı onu kendi listesinde bağımsız
değerlendirir.

### BU DOSYAYA ALINMAYANLAR (MPC'ye ait olabilir → `TO_TEST.md`'de kalsın)

| TO_TEST maddesi | neden almadım |
|---|---|
| 1, 1b, 1c — J fonksiyonu / ödül şekli | maliyet fonksiyonu MPC'nin |
| 3 — maliyet ufku, 7 — çözücü metriği | takip'te ufuk/çözücü yok ama madde MPC'nin |
| 6 — `beta` pitch gecikmesi | `beta` MPC'nin kadraj değişkeni |
| **2 — devir kapısına geometri şartı** | `bbox_to_redis.py` **ortak dosya**, iki yasayı da besliyor |
| **4 — kör terminal "LOS hızını dondur, PN sürdür"** | MPC'nin kör-terminal kolu; benim T5'im ayrı yerde (takip'in kestiricisi) |
| **5b — dinamik tilt takibi** | gimbal **ortak**; tilt komutu `bbox_to_redis`'te |
| **8 — bearing-angle TMA** | yasadan bağımsız kestirici, ikisine de girer |
| **9 — ölçüm araçları**, **10 — eyleyici modeli (τ)** | `mpc_test.py`/`tools/` ortak |
| 0 — replay harness | MPC çözücüsü için tasarlandı |

**Not (T-maddesi değil, uyarı):** madde 10 `mpc_test.Benzetim`'in eyleyici
modelini değiştirmeyi öneriyor. O motor bu dosyanın da zemini →
`takip_test.py` test 0 (**motor mührü**) 9 fizik sabitini mühürlüyor; madde 10
uygulanırsa test patlar ve buradaki bütün taban sayıları yeniden ölçülür.

---

## KULLANICI KISITI VE AMAÇ (2026-08-05 akşam — bu liste buna göre çapalandı)

1. **ASIL AMAÇ ÇARPMA.** Kadraj/merkezleme/titreme metrikleri araç metriğidir,
   amaç değil. Her maddenin **birincil** kabul ölçütü çarpışma (CPA ≤ 2 m ya da
   `bitis == carpisma` oranı); kadraj vb. **yan-etki** olarak okunur.
2. **ÇÖZÜM UZAYI:** kullanıcı görüşü — görüntülü güdümün çözümü ya
   **ArduPilot'un içindeki, görüntülü güdüme uygulanabilecek kodlarda** ya da
   **MPC kodunda**dır; bunun dışında bir çözüm beklenmiyor.
   → Her madde artık **hangi ArduPilot/MPC mekanizmasına dayandığını** yazar.
   Akademik yasa "ithal etmek" bu listede gerekçe sayılmaz; olsa olsa
   *doğrulama/nicel referans* olarak dipnottur.
3. **DENENMİŞ VE ÇALIŞMAMIŞ (tekrar denenmesin):** *Precise Interception of
   Flight Targets by IBVS of Multicopter* (arXiv 2409.17497) — kullanıcı bu
   makaleyi okuyup **koda döktü, pratikte iyi çalışmadı**. Bu listede o
   makalenin bütünsel mimarisi (IBVS+PNG kaskadı) **kaynak olarak
   kullanılmıyor**. Yalnızca iki gözlemi arka plan dipnotu olarak duruyor
   (yaw kanalının ayrı ele alınması; hız artışının `arctan(a/g)` ile
   ilişkisi) — ikisi de zaten ArduPilot'ta kendi karşılığıyla mevcut ve
   maddeler **oraya** dayandırıldı.

---

## ÇALIŞMA KURALLARI

1. **Telemetriden yalnız MENZİL** (düşük güvenle). **Kameradan çıkarım
   SERBEST.** Yerden tespit ayrı branch'te.
   Hatırlatma: takip'in komut yolu menzili **hiç** kullanmıyor (kanıt:
   `takip_test` 6b/11), `--iska-kaynak alan` ile hakem de kullanmıyor.
   **Yeni bir madde bu özelliği bozuyorsa gerekçesi yazılmalı.**
2. Sim koşarken ortak dosyalara dokunma. Aynı anda tek sim. **Şu an sim'de
   başka ajan var** — offline maddeler önce.
3. Ajan: yalnız Opus ya da orkestratör.
4. Her madde: tek knob + ölçülebilir kabul ölçütü + offline→sim boşluğu işaretli.
5. Varsayılan değiştirmeden önce A/B; varsayılan değişirse `ARDUPILOT_TAKIP.md`
   tabloları tazelenir.

---

## TABAN (kabul ölçütlerinin referansı)

Çevrimdışı, `mpc_test.Benzetim` gimbal fiziğinde, 6 senaryo × 3 tohum, 40 s,
min menzil ortancası (m) — `ARDUPILOT_TAKIP.md` §6e:

| kol | düz/kuy | düz/çap | elips/kuy | elips/çap | wand/kuy | wand/çap |
|---|---|---|---|---|---|---|
| şimdiki varsayılan | 1.35 | 1.69 | 1.87 | **2.89** | 4.85 | **13.24** |
| + görsel hakem | 1.35 | 1.69 | 1.87 | 2.89 | 4.85 | **11.54** |

Kadraj kaybı: `0 / 0 / 0 / 10.1 / 19.3 / 2.1 %`.
Sim (gimbal ÖNCESİ, gövdeye sabit kamera): asılı 0.5 / sonsuz 1.2 / elips 0.2 /
wanderer 5.5 m. **Gimballe sim koşusu HENÜZ YOK** — T6/T7 orada.

**Kalan iki darboğaz:** (a) wanderer/çapraz 11.5–13.2 m — yanal gecikme,
(b) elips/çapraz'ın kadraj kaybıyla bitmesi (%10.1).

---

## DAYANAK: ArduPilot ve MPC'de HAZIR OLAN MEKANİZMALAR

Kısıt 2 gereği maddelerin dayanağı burasıdır. Hepsi yerel kopyada okundu
(`~/ardupilot`, 2025-07-13) ya da kendi MPC'mizde mevcut.

| mekanizma | nerede | takip'e ne verir |
|---|---|---|
| `AC_AttitudeControl::input_shaping_angle` | `AC_AttitudeControl.cpp:1103` | **Yaw'ın doğru hâli.** `desired_ang_vel += sqrt_controller(hata, 1/ATC_INPUT_TC, ATC_ACCEL_Y_MAX, dt)` sonra `input_shaping_ang_vel` ile ivme kelepçesi. **İleri besleme yuvası imzada zaten var** (ArduCopter oraya `ang_vel_target` koyuyor) → T1 |
| `AP_Follow` / `mode_follow`'un `vel_of_target` terimi | `mode_follow.cpp` | **Öndeleme (lead) mekanizması.** Sildiğimiz terim buydu; geri konursa saf takip lag'i yapısal olarak kapanır → T2 |
| `AC_PrecLand` EKF + ölü hesap | `AC_PrecLand.cpp:514,552` | Kare düştüğünde hedefi sürdürme → T5 |
| `AC_PrecLand` `land_slowdown` kapısı | `mode.cpp:667` | "Hizalanmadan yaklaşma" kapısı → T8 alternatifi |
| `AP_Math::shape_vel_accel` / `sqrt_controller` | `control.cpp` | Zaten portlu; T3'ün türetimi |
| MPC `BozucuKestirici` | `mpc_gudum.py:1435` | **bbox'tan hedef açısal hareketi** kestirimi; kendi yaw hızımızı çıkarıyor. T2'nin kestirim kaynağı (yöntem olarak; kod takip'e kendi yazılır) |
| MPC ISKA durum makinesi | `mpc_gudum.py:2579` | Zaten kopyalandı (aynı eşikler) |

**Ölçülen uyumsuzluk (T1'in gerekçesi):** `params/swarm_copter.parm`'da
`ATC_ACCEL_Y_MAX = 72000` cdeg/s² = **720 °/s²**, yani araç çok hızlı yaw
ivmesine ayarlı. Buna karşılık iskelet (`goruntulu_temel`) yaw komutunu
**120 °/s²** ile kelepçeliyor — **6× daha yavaş**. O kelepçe ORTAK DOSYADA,
takip içinden aşılamaz; T1 bunu ölçüp raporlar, değiştirme kararı ortak.

### Arka plan dipnotları (dayanak DEĞİL, yalnız nicel referans)

- *Towards Safe Mid-Air Drone Interception* (arXiv 2405.13542): 100 yörüngede
  **saf takip %72 başarı / ilk temas 32.2 s**, PN karışımı **%100 / 5.93 s**.
  Bizim yasa saf takip → T2'nin **niceliksel** gerekçesi. (Yasayı oradan
  almıyoruz; ArduPilot'un kendi FF terimini geri koyuyoruz.)
- Tau/time-to-contact: `(dA/dt)/A = 2/TTC` — hakemde zaten kullandığımız
  büyüklük; T8 bunu hız kanalına da taşımayı sorar.
- Tek eksen gimbal literatürü: tilt **roll'u telafi etmez** → T9 ölçüm maddesi.
- *IBVS of Multicopter* (arXiv 2409.17497): **kullanıcı denedi, kodda iyi
  çalışmadı** — mimari olarak kullanılmıyor (bkz. Kısıt 3).

---

# MADDELER (öncelik sırası)

## T1. YAW'I ArduPilot'un KENDİ ŞEKİLLENDİRMESİNE GEÇİR  [offline, ucuz]

**Dayanak:** `AC_AttitudeControl::input_shaping_angle` (`:1103`) — ArduCopter
yaw açısını böyle sürüyor:
```cpp
desired_ang_vel += sqrt_controller(hata, 1.0/ATC_INPUT_TC, ATC_ACCEL_Y_MAX, dt);
desired_ang_vel  = constrain(±ATC_RATE_Y_MAX);
return input_shaping_ang_vel(onceki, desired_ang_vel, ATC_ACCEL_Y_MAX, dt, 0);
```
Bizde `_yaw()` **ham P**: `4.5·ex`, 60 dps kelepçe. Yani otopilotun kendi
kanalında olan üç şey bizde yok: (a) `sqrt_controller` şekli, (b) **ileri
besleme yuvası** (`+=` — ArduCopter oraya `ang_vel_target` koyar), (c) ivme
kelepçesi ArduPilot'un `ATC_ACCEL_Y_MAX`'ıyla.

**Sorun (ölçüldü):** sim'de `|ex|` p95 **24.6°** (yatay yarı-FOV 33°) → marj
8°'ye iniyor; kadraj kayıplarının **%8–37'si yan kenardan**. Gimbal yaw'ı
çözmüyor (tek eksen). Ayrıca `ATC_ACCEL_Y_MAX=720 °/s²` iken iskelet
komutu **120 °/s²** ile kelepçeliyor (6× yavaş, ortak dosyada).

**Kol:** `_yaw(ex)` → ArduPilot'un zincirinin birebir portu; `sqrt_controller`
zaten `takip_gudum.py`'de portlu. İleri besleme yuvasına **görüntüden ölçülen
kerteriz hızı** konur (kendi yaw hızımız çıkarılmış — MPC `BozucuKestirici`
ile aynı yöntem, kod takip'e kendi yazılır).

**Kabul (birincil = ÇARPMA):** wanderer/çapraz min menzil taban **13.24'ün
altına**; çarpışmayla biten koşu oranı artar. *Yan-etki:* `|ex|` p95 <15°,
yan kadraj kaybı payı düşer, yaw komut adım-farkı rms artmaz.
**Ayrıca raporla:** iskeletin 120 °/s² kelepçesi bağlıyor mu (bağlıyorsa
ArduPilot ayarıyla 6× uyumsuzluk kullanıcıya taşınır — ortak dosya kararı).
**Risk:** türev gürültüsü → LPF şart. **Zeka:** orta-yüksek. **Nerede:** offline.

## T2. `mode_follow`'un SİLDİĞİMİZ İLERİ BESLEME TERİMİNİ GERİ KOY  [en yüksek tavan]

**Dayanak (ArduPilot'un kendi kodu):** `mode_follow.cpp`'nin yasası
```cpp
desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P;
                       ^^^^^^^^^^^^^ BUNU SİLDİK
```
`vel_of_target`'ı sildik çünkü **telemetriden** geliyordu (kural). Sonucu
ölçtük ve mimariyi o belirledi: saf P hareketli hedefe çarpamaz, biz de
büyüklüğü seyir tavanına bağladık (`hiz_kaynagi='tavan'`). Ama **yön hâlâ saf
takip** — öndeleme yok.

**Kuralın yeni hâli bu terimi geri açıyor:** kameradan çıkarım SERBEST. Yani
`vel_of_target` **telemetriden değil görüntüden** kestirilerek ArduPilot'un
kendi terimi geri konabilir. Bu, "dışarıdan yasa ithal etmek" değil,
**sildiğimiz satırı geri koymak**.

**Kestirim kaynağı (MPC kodundan yöntem):** MPC `BozucuKestirici`
(`mpc_gudum.py:1435`) bbox'tan hedefin açısal hareketini kestiriyor ve kendi
yaw hızımızı çıkarıyor. Aynı yöntem takip'e kendi yazılır. (Tam sürümü —
bearing-angle TMA — `TO_TEST.md` madde 8'de ve **ORTAK**; onu sahiplenmiyorum,
üretirlerse tüketirim.)

**Menzilsizlik uyarısı:** `vel_of_target`'ın LOS'a dik bileşeni `r·λ̇` ister →
menzil girer. Menzilsizliği korumak için **açı formunda** uygula: yön vektörünü
`δ` kadar döndür, `δ̇ = K·λ̇`, `|δ| ≤ δ_max` (sapma açısı formu). Böylece `r`
sadeleşir. İki kol A/B: (a) açı formu (menzilsiz), (b) `r·λ̇` formu (menzilli,
ArduPilot'a daha sadık) — **hangisinin daha çok çarptığı ölçülür.**

**Kabul (birincil = ÇARPMA):** wanderer/çapraz **<9 m** ve/veya çarpışma oranı
artışı; elips/çapraz ≤2.89 m korunur; düz/kuyruk regresyon YOK (≤1.5 m).
*Yan-etki:* kadraj kaybı artmamalı (öndeleme hedefi kenara iter — ana risk,
T1 ile **birlikte** ölçülmeli).
**Risk:** yüksek `K` → salınım + kadraj kaybı; `δ_max` ile sınırla.
**Zeka:** yüksek (orkestratör). **Nerede:** offline → sim.
**Nicel referans (dayanak değil):** saf takip %72 / PN karışımı %100 başarı
(arXiv 2405.13542).

## T3. İVME ŞEKİLLENDİRMESİNİ FİZİĞE OTURT  [ucuz, temizleyici]

**Durum:** `ivme_sekillendirme_mps2 = 3.0` **ampirik** seçildi (tarama:
0/2/3/5 → kadraj kaybı %50-72 / %0-8 / %0-23 / %7-41).

**Literatür:** IBVS makalesi aynı problemi türetiyor: hız artışını
`Δq_d ≤ arctan(k_a/g)` ile sınırla (`k_a ≈ 1–3 m/s²`), yani **kabul edilebilir
LOS hatası** ile ivme arasında kapalı form var. Bizde eşleniği:
`Δpitch ≈ arctan(a/g)` → dikey yarı-FOV payı. Gimbal pitch'i sildiği için
artık **kısıt yatayda**: yatma açısı `arctan(a/g)` roll üretir, roll tek eksen
gimbalde telafi EDİLMİYOR.

**Kol:** sabit 3.0 yerine `a_max = g·tan(pay_açısı)`; `pay_açısı` yatay FOV
marjından türetilsin. Tek knob: `pay_açısı`.
**Kabul:** aynı kadraj kaybı ≤ mevcut, min menzil regresyonu yok, sayı artık
**türetilmiş** (montaj/FOV değişince kendiliğinden güncellenir).
**Risk:** düşük. **Zeka:** orta. **Nerede:** offline.

## T4. KARE GECİKMESİ TELAFİSİ  [ucuz, ölçülmemiş]

**Sorun:** `Olcum.t_capture` (karenin yakalanma anı) iskelet tarafından
veriliyor ama takip **kullanmıyor**; komut, gecikmiş bir kerterize göre
üretiliyor. Sim'de döngü 20 Hz, kamera 30 Hz → 30–80 ms tipik.

**Literatür:** IBVS makalesi bunun için ayrı bir **gecikmeli KF** koyuyor
("to mitigate image processing delays").

**Kol:** `komut()` içinde `ex/ey`'yi `(t_şimdi − t_capture)` kadar **kendi
açısal hızımızla** ileri taşı (yalnız kendi telemetrimiz — kural temiz).
İkinci kol: T1'in `ė_x`'i ile birlikte tam öngörü.
**Kabul:** `|ex|` p95 düşer; gecikme telafisi kapalı/açık A/B'de min menzil
regresyonu yok. **Risk:** yanlış işaret → kararsızlık; işaret testi şart.
**Zeka:** orta. **Nerede:** offline (motor `t_capture` üretiyor mu — önce onu
doğrula; üretmiyorsa madde **sim'e** kayar, NOT DÜŞÜLÜR).

## T5. KÖR FAZDA HEDEFİ SÜRDÜR (PrecLand EKF'i)  [taban 10.1% kadraj kaybı]

**Sorun:** bbox bayatlayınca takip'in kestirimi donuyor; iskelet süzülmeye
geçiyor. elips/çapraz artık **kadraj kaybıyla** bitiyor (%10.1).

**Kaynak:** `AC_PrecLand` eksen başına EKF + ataletsel ölü hesap:
`_target_pos_rel_est_NE −= inertialNavVelocity·dt`, NIS ile aykırı eleme.
Bizde `_hedef_kestir()`'in yanına konur.

**Kol:** hedefin göreli konumunu **kendi hızımızla** ölü hesapla sürdür
(hedef hızı YOK — kural), taze bbox gelince düzelt. Kör süre boyunca `u_los`
sürdürülür, dondurulmaz.
**Kabul:** kadraj kaybıyla biten koşu oranı düşer; elips/çapraz ≤2.89 m
korunur ya da iyileşir; kör fazda komut yönü sıçraması yok.
**Risk:** uzun körlükte ölü hesap kayar → süre kapısı (ör. >1.0 s sonra bırak).
**Zeka:** yüksek. **Nerede:** offline.
**Not:** `TO_TEST.md` madde 4 MPC'nin kör-terminal kolu; bu ayrı bir yerde
(takip'in kestiricisi), onu sahiplenmiyorum.

## T6. GÖRSEL HAKEMİN SİM DOĞRULAMASI  [kanıt boşluğu]

**Durum:** `--iska-kaynak alan` çevrimdışı **bedelsiz** ölçüldü ve gimballe
wanderer/çaprazda menzil hakeminden **iyi** (13.24 → 11.54). Ama çevrimdışı
motor kalite kapısını (`min(w,h) ≥ 9 px`) hiç tetiklemiyor — **gerçek kamerada
doğrulanmadı**.

**Kol:** sim'de `GORUNTULU="takip_gudum.py --iska-kaynak alan"`, elips + wanderer.
**Kabul:** ıska sebepleri makul (oran kolu ateşliyor, "çok küçük" kolu yanlış
ateşlemiyor); min menzil menzil-hakemi koşusundan kötü değil; `s_lpf`/`s_tepe`
tanı kolonları anlamlı.
**Risk:** gerçek bbox gürültüsü oran testini titretebilir → debounce zaten var.
**Zeka:** düşük-orta. **Nerede:** SİM (sıra bekliyor).

## T7. İLERLEME SAATİNİN SİM DOĞRULAMASI  [kanıt boşluğu]

**Durum:** varsayılan oldu; çevrimdışı elips/çapraz 21.4 → 5.9 (gövdeye sabit),
20.6 → 2.89 (gimbal). Gerçek sim loglarında da desen doğrulanmıştı (takip'te
zaman aşımı ıskalarının %79'u kapanırken). **Ama gimballi sim koşusu yok.**

**Kol:** aynı koşuda T6 ile birlikte ölç (`--iska-zaman-kaynak duz` kıyas kolu).
**Kabul:** zaman aşımı ıskalarının kapanma hızı medyanı ≤0 (artık kazananı
kesmiyoruz); angajman sayısı düşer, angajman başına min menzil iyileşir.
**Zeka:** düşük-orta. **Nerede:** SİM.

## T8. TAU (TIME-TO-CONTACT) TABANLI HIZ PROFİLİ  [açık, orta tavan]

**Fikir:** `(dA/dt)/A = 2·kapanma/r = 2/TTC` — yani **görüntüden doğrudan
time-to-contact** okuyoruz ve bunu zaten hakemde kullanıyoruz. Literatürde
(Tau teorisi) bu büyüklük **hız profili** üretmek için kullanılıyor.

**Kol:** `hiz_kaynagi` için üçüncü bir seçenek: `tau` — komut büyüklüğü
sabit tavan yerine hedef bir TTC'yi tutturacak şekilde. Menzil GİRMEZ.
**Kabul:** düz/kuyruk regresyon yok; terminal titreme (pitch/roll hızı) düşer;
çarpışma oranı korunur. **Hipotez:** tavan kolu kadar hızlı kapatır ama
terminale daha yumuşak girer → kadraj kaybı azalır.
**Risk:** TTC gürültülü (alan epizodik) → yalnız yakın bantta devreye al.
**Zeka:** yüksek. **Nerede:** offline.
**Not:** MPC'nin ödül şekli sorusu (`TO_TEST.md` 1b) **ayrı bir şey**; bu
madde takip'in hız kanalı, oraya karışmıyor.

## T9. GİMBAL ARTIĞI: ROLL'UN takip'e ETKİSİNİ ÖLÇ  [ölçüm, ucuz]

**Neden:** gimbal tek eksen — **roll telafi edilmiyor** (literatürde de tek
eksenin bilinen sınırı; agresif manevrada 0.5–2° tipik). Yanal manevrada
roll `arctan(a_yan/g)` mertebesinde; takip'in `ex/ey`'si roll de-rotasyonundan
geçiyor ama **fiziksel kadraj** dönüyor.

**Kol:** `takip_test`'e ölçüm: kayıp anlarında roll dağılımı ve kaybın
roll ile korelasyonu; `ivme_sekillendirme` kollarında roll tepe değeri.
**Kabul:** "yanal kayıpların ne kadarı roll kaynaklı" sorusu SAYIYLA
yanıtlanır → T1/T2/T3'ün önceliği buna göre güncellenir.
**Zeka:** orta. **Nerede:** offline (ölçüm maddesi, düzeltme değil).

## T10. YASA KOLLARININ SİM ABLASYONU  [tamamlayıcı]

Çevrimdışı ölçülmüş ama sim'de hiç koşulmamış kollar: `--yasa poscon`,
`--fren ap`, `--hiz-kaynagi p`, `--ivme-sekil 0`.
**Kabul:** her kol için tek sim koşusu; `ARDUPILOT_TAKIP.md` §9 ablasyon
tablosunun sim karşılığı dolar. **Öncelik düşük** — bilimsel tamlık için.

---

## DURUM İZLEME

| madde | konu | durum | yer |
|---|---|---|---|
| T1 | yaw → `input_shaping_angle` (ArduPilot'un kendi zinciri) | **SIRADA** | offline |
| T2 | `mode_follow`'un `vel_of_target` FF'ini görüden geri koy | AÇIK | offline → sim |
| T3 | ivme şekillendirmesini türet | AÇIK | offline |
| T4 | kare gecikmesi telafisi | AÇIK | offline (önce t_capture doğrula) |
| T5 | PrecLand EKF, kör fazda sürdürme | AÇIK | offline |
| T6 | görsel hakem sim doğrulaması | SİM BEKLİYOR | sim |
| T7 | ilerleme saati sim doğrulaması | SİM BEKLİYOR | sim |
| T8 | tau tabanlı hız profili | AÇIK | offline |
| T9 | roll artığının etkisini ölç | AÇIK | offline |
| T10 | yasa kollarının sim ablasyonu | AÇIK | sim |

**Sıra gerekçesi:** T1 ve T2 kalan iki darboğazın (yanal gecikme, yan kadraj
kaybı) doğrudan üstüne gidiyor ve **ikisi de ArduPilot'un kendi kodunda hazır
duran mekanizmalar** (Kısıt 2) — T1 `input_shaping_angle`, T2 `mode_follow`'un
sildiğimiz `vel_of_target` terimi. T3/T4 ucuz temizlik. T5 kadraj kaybının geri
kalanını hedefliyor (`AC_PrecLand`). T6/T7 sim boşaldığında, kanıt borcunu
kapatmak için.

**Hatırlatma:** birincil ölçüt her maddede ÇARPMA'dır (CPA ≤ 2 m / çarpışma
oranı); kadraj, merkezleme, titreme yan-etki sütunudur.
