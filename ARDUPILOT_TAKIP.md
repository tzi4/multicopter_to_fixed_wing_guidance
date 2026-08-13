# ArduPilot güdüm yasalarıyla görüntülü güdüm — araştırma ve `takip_gudum.py`

Branch: `ardupilot-takip`. Soru: **MPC yerine ArduPilot'un kendi takip
yasası kullanılabilir mi?** Bu dosya (a) ArduPilot'ta gerçekten ne olduğunu,
(b) bizim kısıtlarımızla neyin kullanılabildiğini, (c) ölçülen sonucu
anlatır. Kaynak: yerel `~/ardupilot` kopyası (2025-07-13) + upstream
`plane_follow.lua`.

---

## 1. ArduPilot'ta görüntülü hedefe gitmek için NE VAR

Dört aile var ve **ikisi birbirini tamamlıyor** — biri kestirim, biri kontrol.

### (A) `AC_PrecLand` — "land" ailesi. **KESTİRİM tarafı.**

Precision Landing, kamera/işaret sisteminden gelen **birim LOS vektörünü**
alıp mesafeyle çarparak hedefin göreli konumunu kurar:

```cpp
// AC_PrecLand.cpp: construct_pos_meas_using_rangefinder()
if (retrieve_los_meas(target_vec_unit, frame)) {          // kerteriz  (kamera)
    ...
    _target_pos_rel_meas_NED = (target_vec_unit_ned * dist_to_target)
                                + cam_pos_ned_rel_imu;    // x mesafe (rangefinder)
}
```

Üstüne **eksen başına EKF** kurar (`_ekf_x`, `_ekf_y`), kareler arasında
ataletsel hızla ölü hesap yapar, NIS ile aykırı ölçüm eler:

```cpp
_target_pos_rel_est_NE.x -= inertialNavVelocity.x * dt;   // kare yokken sürdür
if (NIS_x < _outlier_reject_num) _ekf_x.fusePos(meas, var);
```

Ve `mode.cpp::precland_run()` içinde **kadraj/hizalanma kapısı** var:
yatayda hedeften uzaksan alçalma, hata büyükse alçalma hızını kıs:

```cpp
const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent/acceptable_error));
cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms + land_slowdown);
```

**Bizim aldığımız:** birinci satır — `takip_gudum._hedef_kestir()` tam olarak
`birim_LOS(ex, eps) × menzil` yapıyor. **Almadığımız (bilinçli, madde 7):**
EKF + ölü hesap ve hizalanma kapısı.

`plane_precland.lua` ise yalnızca QuadPlane sarmalayıcısı: `AC_PrecLand`'in
hedefini okuyup VTOL fazında `vehicle:set_target_location` çağırıyor. Kendi
kontrol yasası yok, normal sabit kanatta da çalışmıyor.

### (B) `ArduCopter/mode_follow.cpp` + `AP_Follow` — **KONTROL tarafı.**

Copter-4.4'teki hâli (kullanıcının bahsettiği ~150 satırlık cpp) tam olarak
şu — ve bizim kopyaladığımız yasa bu:

```cpp
desired_velocity_neu = vel_of_target + dist_vec_offs_neu * FOLL_POS_P;
|v_xy| <= WPNAV_SPEED;   v_z ∈ [-WPNAV_SPEED_DN, WPNAV_SPEED_UP];
avoid.limit_velocity_2D(PSC_POSXY_P, WPNAV_ACCEL*0.5, ...);   // yaklaşırken yavaşla
v_z <= avoid.get_max_speed(PSC_POSZ_P, WPNAV_ACCEL_Z*0.5, |dz|);
yaw  = hedefin kerterizi;                                      // FOLL_YAW_BEHAVE=0
```

Copter-4.5+ aynı işi `pos_control->input_pos_vel_accel_NE()` ile yapar; oradaki
iç çevrim `AC_P_2D::update_all` = `sqrt_controller(hata, PSC_POSXY_P, ivme, dt)`.
İkisi de `takip_gudum.py`'de var (`--yasa klasik|poscon`).

Hedef konumu `AP_Follow`'a **MAVLink telemetrisinden** gelir
(`GLOBAL_POSITION_INT` / `FOLLOW_TARGET`). Yani yasa hazır, **veri kaynağı
bizde yasak.**

### (C) `plane_follow.lua` — sabit kanat, **mimari şablon.**

Görüntü kullanmıyor, `AP_Follow`'dan konum+hız+heading alıyor. Bizim için
değerli olan kısmı kontrol değil **mimarisi**: yön ve hız AYRI kanallar.

```
GUIDED_CHANGE_HEADING (43002)   <- kerteriz + crosstrack PID
GUIDED_CHANGE_SPEED   (43000)   <- hedef airspeed'i etrafında mesafe PID'i
GUIDED_CHANGE_ALTITUDE(43001)   <- hedefin irtifası
```

Hız, mesafeyle ölçeklenen bir P çıkışı **değil**; ayrı bir kanal. Bu ayrım
bizim için hayati oldu (madde 4).

### (D) `CAMERA_TRACKING_IMAGE_STATUS` / `fake_camera_tracking.lua`

Kullanıcının LLM'inin dediği gibi: bu mesaj ArduPilot'ta **navigasyon girdisi
değil**, yalnızca GCS arayüzü. Güdüme bağlanamaz.

---

## 2. Neden Python'da yeniden yazdık, uçuş yazılımını kullanmadık

En "yerli" seçenek şu olurdu: companion `FOLLOW_TARGET` MAVLink mesajıyla
görüntüden kurduğu hedef konumunu otopilota yollasın, kopter **FOLLOW
modunda** uçsun, yasa gerçekten ArduPilot'un kendi kodunda koşsun.

Denemedik, çünkü:

1. **FOLLOW modu "yanında dur" moduymuş gibi frenler** — `limit_velocity_2D`
   parametreden kapatılamıyor. 21 m/s'lik hedefe 25 m'de 10.9 m/s tavanla
   yaklaşırsın; çarpma imkânsız (§4, madde 3).
2. **Devir mimarisi kırılırdı.** Bütün yığın `komut_yetkisi` + GUIDED hız
   setpoint'i üzerine kurulu (`bbox_to_redis` ↔ `goruntulu_temel`). Mod
   değiştirmek ıska/devralma zincirini komple yeniden yazmak demek.
3. **Karşılaştırma kirlenirdi.** MPC ile A/B'nin anlamlı olması için iskelet,
   LPF, hız kelepçesi, ıska ölçütü aynı kalmalı. Aynı `GoruntuluKontrolcu`
   arayüzünde kalmak bunu bedavaya veriyor.

Kullanıcının sorduğu LLM de aynı yeri işaret ediyordu: görüntü işleme ve
hedef kestirimi companion'da, `vision:get_estimated_target()` diye bir
ArduPilot binding'i yok. Doğru; o fonksiyonun karşılığı bizde
`_hedef_kestir()`.

---

## 3. Kural uyumu (hedeften yalnız menzil)

| Girdi | Kaynak | İzinli mi |
|---|---|---|
| `ex`, `ey` | sanal gimbal (`tracker_bbox_stab`) | evet, görüntü |
| `menzil` | `MenzilKestirici` | evet, tek izinli telemetri türevi |
| `pos/vel/yaw/pitch/vibe` | **kendi** telemetrimiz | evet |
| hedef hızı/yönü/ivmesi | — | **kullanılmadı** |

`mode_follow`'un `vel_of_target` ileri besleme terimi **tamamen silindi**.
Bunun bedeli ölçüldü ve mimariyi belirledi (§4 madde 4).

---

## 4. Dört zorunlu uyarlama (hepsi ölçüldü, hepsi tek düğme)

**(1) Hedef nerede?** `AP_Follow` yerine `AC_PrecLand` yöntemi:
`hedef_pos = kendi_pos + menzil × Rz(yaw)·LOS(ex, eps)`.
Ekstrapolasyon yok (hedef hızı yasak) → kestirim daima "şimdi"ye ait.

**(2) `FOLL_POS_P` 0.1 → 1.0.** İleri besleme olmayınca P terimi hedefin
tüm hızını tek başına üretmek zorunda; kalıcı takip mesafesi
`d* = v_hedef / kp`:

| kp | denge mesafesi (21.05 m/s hedef) | tavana doyum |
|---|---|---|
| 0.10 (AP varsayılanı) | 210.5 m | 350 m |
| 0.35 | 60.1 m | 100 m |
| 1.00 | 21.1 m | 35 m |

Angajman zarfı ≤60 m olduğu için AP varsayılanı **matematiksel olarak**
yakalayamaz.

**(3) Yaklaşma freni KAPALI.** `sqrt_controller(25 m, 1.0, 2.5)` = 10.90 m/s.
Hedef 21.05 m/s. Fren açıkken kapanma imkânsız; kapalı dongu: min menzil
30.00 m (hiç kapanmıyor). `--fren ap` ile geri açılır (istasyon tutma /
güvenli mesafe senaryosu için doğru davranış budur).

**(4) Hız büyüklüğü yasadan değil seyir tavanından.** (2)'nin kaçınılmaz
sonucu: `|v| = kp·hata` olan her saf P yasası hareketli hedefe çarpamaz;
`d*`'ta menzil donar. Kapalı döngüde doğrulandı:

```
saf mode_follow (kp=1.0):  min menzil 22.47 m  (teori 21.1)  kapanma +0.8 m/s
'tavan' kolu            :  min menzil  1.54 m  ÇARPIŞMA      kapanma +4.3 m/s
```

Çözüm `plane_follow.lua`'nın kendi mimarisi: **yön** FOLLOW yasasından,
**hız** seyir tavanından (`GUIDED_CHANGE_SPEED` mantığı). Saf kol
`--hiz-kaynagi p` ile duruyor.

---

## 5. Kadrajı kim koruyor? (bu yasanın asıl açığı)

**Hiçbir terim.** MPC'de FOV sert kısıtı, ivme cezası, terminal dikey
hizalama var; burada yok. Ölçülen zincir:

```
devir anı: komut 17 → 35 m/s BASAMAK
  → ileri ivme doyuyor (5 m/s²) → burun 15.7° AŞAĞI
  → gövdeye sabit kamera onunla iniyor
  → eksenin ÜSTÜNDE olan hedef ÜST kenardan çıkıyor
  → 18 kare (≈0.9 s) KÖR
```

> **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05]** Yukarıdaki zincir
> `ardupilot-takip` dalında ölçüldü ve orada geçerlidir. **`gimbal` dalında
> üçüncü halka ("gövdeye sabit kamera onunla iniyor") KOPTU:** kamera kendini
> stabilize eden fiziksel tek eksen (tilt) gimbalinde, gövde −35.4…+35.2°
> savrulurken kamera dünya pitch'i max |0.65°| ölçüldü. Yani orada bu açığın
> DİKEY bileşeni yok; kalan bileşenler yaw (gimbal tek eksen) ve roll.
> İki dal birleşirse `ivme_sekillendirme_mps2` ayarı yeniden ölçülmeli —
> aşağıdaki tablo gövdeye-sabit kamerayla alındı. bkz. `NOTLAR_GIMBAL.md`

Tek savunma ArduPilot'un kendi aracı: **kinematik şekillendirme**
(`shape_vel_accel` / `WPNAV_JERK` karşılığı, `ivme_sekillendirme_mps2`).
6 senaryo × 4 tohum, min menzil ortancası (m) / kadraj kaybı:

| a [m/s²] | düz/kuy | düz/çap | elips/kuy | elips/çap | wand/kuy | wand/çap | kayıp |
|---|---|---|---|---|---|---|---|
| 0.0 (kapalı) | 1.78 | 15.36 | 7.96 | 28.86 | 14.96 | 30.00 | %50–72 |
| 2.0 | 5.37 | 18.65 | 5.48 | 18.02 | 8.60 | 23.42 | %0–8 |
| **3.0 (varsayılan)** | **1.35** | **1.37** | **1.88** | **21.48** | **5.20** | **17.78** | %0–23 |
| 5.0 | 1.68 | 2.83 | 2.50 | 37.11 | 1.74 | 39.81 | %7–41 |

**MPC'de bunun tersi ölçülmüştü** (DEVAM.md tur-2: `ileri_ivme_tavani`
2 m/s² → min menzil 1.93 → 11.17 m, "ölçülerek elendi"). Çelişki değil:
MPC kadrajı zaten maliyetinde koruyordu, orada şekillendirme yalnız kapanma
bedeli getiriyordu. Burada kadrajı koruyan hiçbir şey yok, o yüzden
şekillendirmenin kazandırdığı kadraj kaybettirdiği kapanmadan büyük.

ArduPilot'un bu soruna **asıl** cevabı güdümde değil donanımda:
`mode_follow.cpp::init()` FOLLOW moduna girerken gimbali hedef sysid'ye
kilitliyor (`AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER`).

---

## 6. Çevrimdışı kıyas — AYNI motor, AYNI kural

`mpc_test.Benzetim` (nokta-kütle avcı + gerçek sanal gimbal + iskelet
zinciri), hedef 21.05 m/s, 1.5 s kadrajsızlıkta koşu biter (`KAYIP_BITIS_S`).
Min menzil [m], 2 tohum ortalaması:

| senaryo | MPC | takip (a=3) | |
|---|---|---|---|
| düz / kuyruk | 1.63 | **1.35** | eşit |
| düz / çapraz | 32.9 | **1.37** | **takip çok daha iyi** |
| elips / kuyruk | 1.90 | 1.88 | eşit |
| elips / çapraz | 20.7 | 21.5 | eşit |
| wanderer / kuyruk | **3.18** | 5.20 | MPC iyi |
| wanderer / çapraz | **11.5** | 17.8 | MPC iyi |

Döngü maliyeti: **43 µs** (MPC çözücüsü p95 ≈ 13 000 µs) — 300× ucuz.

Okuma: yasa **manevrasız geometride MPC ile eşit**, hatta düz/çaprazda daha
iyi (MPC orada 33 m'de ıska ilan ediyor). Hedef manevra ettikçe fark açılıyor
ve fark **kadraj kaybından** geliyor (%23 vs %30-35 değil, kayıp anları
kritik anlara denk geliyor).

---

## 6b. SİM SONUÇLARI (2026-08-05, dört senaryo)

`tools/karsilastir.py` hükmü — `VURUS` = en yakın geçiş ≤3 m, `YAKIN` ≤8 m.
**Uyarı (aracın kendi notu):** Gazebo iki SITL aracı arasında **temas
modellemiyor**; gerçek çarpmada vibe sıçramaz, o yüzden `min_m` ana ölçüttür
(`vurus_basarili` vibe kapısı bu senaryoda hiç ateşlemedi — beklenen).

| rota | koşu | hüküm | min_m | devir | ex_rms | sıçrama | tespit% |
|---|---|---|---|---|---|---|---|
| asılı hedef | takip 121424 | **VURUS** | **0.7** | 13 | 2.53 | 0.88 | 56.4 |
| asılı, ıska 15 s | takip 124843 | **VURUS** | **0.5** | 6 | **1.33** | **0.14** | 75.5 |
| sonsuz (düz) | takip 122116 | **VURUS** | **1.2** | 10 | 11.30 | **0.15** | 61.5 |
| elips | takip 122843 | **VURUS** | **0.2** | 12 | 13.44 | 1.82 | 68.8 |
| wanderer | takip 123835 | YAKIN | 5.5 | 9 | 18.88 | 3.03 | 72.2 |

Aynı ortam/aynı iskeletle MPC referansları:

| rota | MPC koşusu | hüküm | min_m |
|---|---|---|---|
| asılı hedef | mpc 031804 (15 s ıska eşiği) | VURUS | 0.5 |
| sonsuz | mpc 093533 (tur-4) | VURUS | 1.3 |
| sonsuz | mpc 063956 (tur-3) | VURUS | 0.3 |
| elips | mpc 021132 (35 m/s turundan ÖNCE) | ISKA | 9.2 |
| wanderer | mpc 190919 / 191719 | YAKIN | 6.7 / 4.9 |

**Okuma:**

* Üç senaryoda MPC ile **aynı sınıfta**; elipste tablodaki en iyi sayı
  (0.2 m) — ama MPC'nin o rotada tur-4 koşusu yok, kıyas eksik.
* Wanderer ikisinde de açık: takip 5.5 m, MPC 4.9-6.7 m.
* **Devir sıçraması** (devirde komut büyüklüğü adımı) sonsuzda **0.15** —
  tablodaki en yumuşak devir. Kinematik şekillendirmenin doğrudan sonucu.
* **`ex_rms` 2.53** (asılı) tablodaki en iyi merkezleme; ama sonsuz/elips/
  wanderer'da 11-19 — yaw kanalı hızlı geometride geriden geliyor
  (`|ex|` p95 24.6°). MPC'nin yaw'ı maliyetten geliyor, bizimki saf P.
* **Asılı hedefte 9 angajmanın hepsi 8 s ıska zaman aşımıyla kesildi**
  (en iyi menziller 20.8/19.2/14.3/13.4/8.8/5.6/4.6 m). Sebep tanı
  logunda net: devir **hareketsiz** araca yapılıyor (tohum hızı 0) ve komut
  3 m/s² ile rampalanıyor — bir angajmanda `cmd_hiz` 0.15 → 16.4 m/s,
  menzil 27.2 → 5.6 m ve tam orada süre doluyor. Yani kesen şey yasa değil
  **koşu yapılandırması**; 8 s eşiği ~20 m/s'de başlayan kuyruk angajmanları
  için kalibre edilmişti (MPC'nin asılı rekoru da 15 s eşikle alınmıştı).
  `--iska-zaman-asimi 15` bunun için eklendi.
* **Hipotez sim'de doğrulandı** (takip 124843): 15 s eşikle min menzil
  0.7 → **0.5 m**, devir sayısı 13 → 6 (angajmanlar artık tamamlanıyor,
  boşuna yeniden devralma yok), `ex_rms` 2.53 → **1.33** ve `ey_rms`
  8.80 → **3.54** — ikisi de tüm karşılaştırma tablosundaki en iyi
  değerler. Tespit %56 → %76, devir sıçraması 0.14. MPC'nin aynı
  senaryodaki rekoruyla (0.5 m) **eşit**.

## 6c. MENZİLE NE KADAR YASLANIYORUZ? (2026-08-05 öğleden sonra)

Kullanıcının kuralı: menzil **yerden tespitten** geliyor; görüntülü güdümün
amacı zaten yerden tespitin hatasını atıp hedefi kendi gözüyle görmek. O yüzden
menzil "mecburen eklenen, pek güvenilmeyen" bir girdi. Soru: bu yasa ona ne
kadar yaslanıyor?

### Cevap: komut yolu menzili HİÇ kullanmıyor (cebirsel)

Varsayılan ayarda (`ofs=0`, `hiz_kaynagi='tavan'`):

```
hata = r·u_los + 0
v    = kp·hata                = kp·r·u_los
'tavan':  v = v·(V/|v|)       = V·u_los          ← r TAM SADELEŞİR
yatay tavan: |v_xy| = V·cos(eps) ≤ V             ← hiç bağlamaz
dikey kelepçe: v_z = clip(−V·sin(eps), −10, +5)  ← yalnız eps'e bağlı
```

Komut `(ex, ey, yaw, aim)`'in fonksiyonudur. Test 6b bunu bit düzeyinde
doğruluyor: menzil 10 m / 60 m / 200 m ve **menzil ölçümü hiç yokken**
komut `[19.9908, 1.3979, −2.1062]` — dördü de aynı.

### Kapalı döngü: 8 bozma kolu, hepsi birebir aynı

Yerden tespitin bozulma biçimleri (`test 11`), 3 senaryo × 3 tohum:

| bozma | düz/kuyruk | elips/kuyruk | wanderer/kuyruk | elips/çapraz (ISKA) |
|---|---|---|---|---|
| temiz | 1.35 | 1.87 | 5.20 | 21.43 |
| yanlılık ×0.5 | 1.35 | 1.87 | 5.20 | 21.43 |
| yanlılık ×2.0 | 1.35 | 1.87 | 5.20 | 21.43 |
| yanlılık +20 m | 1.35 | 1.87 | 5.20 | 21.43 |
| gürültü %30 | 1.35 | 1.87 | 5.20 | 21.43 |
| donmuş (ilk değer) | 1.35 | 1.87 | 5.20 | 21.43 |
| kopuk %50 | 1.35 | 1.87 | 5.20 | 21.43 |
| **MENZİL HİÇ YOK** | 1.35 | 1.87 | 5.20 | 21.43 |

*(Tuzak: bozucuya ayrı RNG verilmezse döngü jitter'i kayar ve "gürültü menzili
bozdu" diye okunan şey aslında farklı bir `dt` dizisidir — bir kez düşüldü,
`bozucu_rng` ile ayrıldı.)*

### Kalan tek bağımlılık: ISKA hakemi — ve o BOZULUYOR

Yukarıdaki senaryolarda ıska hep **zaman aşımıyla** ateşlediği için menzile
bağlı kollar hiç denenmedi. Sentetik "kapan sonra açıl" profiliyle doğrudan
sınandı (`test 12`): ıskanın ateşlediği **gerçek** menzil —

| menzil bozması | ×0.5 | temiz | +20 m | ×2.0 |
|---|---|---|---|---|
| ateşleme menzili | 70.6 m | **18.7 m** | 40.7 m | 25.7 m |

Yayılım **51.9 m**. Sebep: kural saf fark kuralı değil; fark testleri
(`r > en_iyi + 30`) mutlak kapılarla (`12 / 45 / 120 m`) iç içe, yanlılık
kapıları kaydırınca hangi kolun ateşlediği değişiyor.

### Öneri ve uygulaması: GÖRSEL HAKEM (`--iska-kaynak alan`)

Hakemin sorduğu soru **mutlak değil oransal**: "menzil en iyinin kaç katına
açıldı?" `s = sqrt(bbox alanı) ≈ C/r` olduğu için

```
r / r_en_iyi  ==  s_tepe / s        →  C SADELEŞİR, kalibrasyon GEREKMEZ
```

Üç kural, üçü de ölçek-bağımsız: (1) `s < s_tepe/oran` → menzil açılıyor,
(2) alan büyüyordu şimdi küçülüyor → geçtik, (3) bbox kalite kapısının altında
ısrarla kalıyor → hedef çok uzak (`iska_mutlak_m`'nin menzilsiz karşılığı;
sim'de 402 m'de yapılan sahte devri yakalayan kol oydu).

**Ölçülen sonuç:**
- Aynı sentetik profilde dört bozma kolunda da **tam olarak 31.2 m**'de
  ateşliyor (menzil hakemi 18.7–70.6 arası savruluyordu). Bedeli ~0.7 s
  gecikme (LPF 0.35 s + debounce 0.3 s).
- Kapalı döngüde **6 senaryonun altısında da menzil hakemiyle birebir aynı**
  sonuç (en büyük fark 0.000 m) → **bedeli yok**.
- `iska_kaynak='alan'` + **menzil hiç yok** → sistem aynen uçuyor
  (en büyük fark 5.7e-14 m). Yani menzil kablosu kesilse arac aynı uçar.

### Sınır: bbox alanı MENZİL KAYNAĞI OLAMAZ (ölçüldü, öneri çürütüldü)

12 koşu / 14.816 kare log madenciliği (uçak grubu, kalite kapılı):

| bant | görsel `C/√alan` MAE / p90 | mevcut kestirim MAE / p90 |
|---|---|---|
| 0–20 m | 2.74 m / **21.4 m** | 1.07 m / **2.21 m** |

Karelerin %45'inde görsel hata >3 m (kestirimde %0). Model yalnızca **~9–50 m**
arasında geçerli: `min(w,h) ≤ 8 px` altında C 128–407'ye çöküyor (gerçek 698),
`> 80 px` (≈7 m, yani vuruş anı) üstünde de bozuluyor. Hata beyaz gürültü değil
**epizodik** (>%30 hata epizotlarının p90 süresi 0.65 s), o yüzden filtreleme
kurtarmıyor. **Bu yüzden alan yalnız ORAN olarak kullanılıyor, mutlak menzil
olarak asla.** Kalite kapısı bu ölçümden geliyor: `min(bbox_w,bbox_h) ≥ 9 px`
(≈ `kapsama_pct ≥ %1`).

Yan bulgu: `HEDEF_ARAC=drone2` ("asılı") koşularında `menzil_m ≡ gerçek menzil`
(karelerin %97–99'unda tam sıfır fark) — o koşularda güdüm **gerçek menzili**
kullanmış, kestirimi değil. Asılı koşuların menzil sayıları bu yüzden
"kestirim kalitesi" olarak okunamaz.

## 6d. HAKEM KAZANANI KESİYOR (gerçek sim loglarıyla doğrulandı)

Çevrimdışı izde yakalanan desen — ıska, biz **kapanırken** ilan ediliyor —
14 koşunun olay + telemetri loglarında (61 ISKA, 128 angajman) sınandı.

**ISKA anındaki kapanma hızı, sebebe göre:**

| sebep | n | kapanma med | önceki 2 s med | gerçek menzil med | kapanma>+2 m/s |
|---|---|---|---|---|---|
| **takip** / zaman aşımı | 14 | **+3.50** | +3.22 | 17.2 m | **%79** |
| takip / menzil açılıyor | 7 | −12.54 | −12.77 | 58.2 m | %0 |
| mpc / zaman aşımı | 22 | −1.53 | −1.61 | 31.5 m | %9 |
| mpc / menzil açılıyor | 14 | −5.18 | −5.89 | 52.3 m | %0 |

**Angajman boyunca kapanma hızı (çeyrek medyanları):**

| kol | Q1 | Q2 | Q3 | Q4 |
|---|---|---|---|---|
| takip / zaman aşımı | −0.10 | +1.71 | +2.36 | **+3.79** |
| mpc / zaman aşımı | +0.55 | +0.42 | −1.34 | −1.03 |

takip'te kapanma **monoton artıyor** ve tam en hızlı kapandığımız çeyrekte
sayaç doluyor. Somut kesikler: 6.4 m'de +9.6 m/s, 14.6 m'de +12.3 m/s,
13.3 m'de +7.5 m/s ile kapanırken bırakılmış.

**Dürüst kırılım (ajanın eklediği nüans):** "kapanırken kesildi" tek başına
yeterli değil — angajman içinde CPA'dan çoktan geçilmiş olabilir. Hem
kapanırken hem de **hiç CPA'dan geçmemiş** olan gerçek "kesilmiş kazanan"
sayısı 7/61 (%11); takip'te 5/23 (%22), MPC'de 2/38 (%5). Bunların 3'ü
2 saniye içinde sıfır menzile inecekti.

**İki ayrı sonuç:**
- `menzil aciliyor` mantığı **her koşulda doğru** (21 ıskanın hiçbirinde
  kapanma > +2 m/s, medyan −6.9 m/s). Dokunulmamalı.
- Kusur **yalnız sabit süre sayacında**. `iska_zaman_kaynak='ilerleme'`
  tam bu kolu hedefliyor.
- **MPC'de bu sorun yok** (%5): MPC angajmanlarının %77–93'ünde ıskadan
  ortalama 4.6–6.9 s önce zaten CPA'dan geçilmiş, medyan 22 m ile
  ıskalanmış. Yani MPC'nin ıskası hakemden değil güdümden geliyor —
  bu, MPC kök-neden analiziyle ("kapanma prim olarak kodlanmış") birebir
  örtüşüyor.

**Uyarı (ölçüldü):** MPC/zaman aşımı kolunda kapanma medyanı −1.53 m/s iken
bbox alanı karelerin %57'sinde büyüyordu — yani alan ile menzil çelişebiliyor.
Bu yüzden 'ilerleme' saati **sıfırlanmaz, geri sarar** (sızıntılı integratör)
ve üstüne mutlak tavan (25 s) konur.

## 6e. YASA vs FİZİK — 2×2 (gimbal dalı sonrası, 2026-08-05 akşam)

Soru: yasanın bu hâli ile önceki hâli arasında ciddi bir fark var mı, ve
bunun ne kadarı **gimbalden** geliyor?

Önce bir tespit: **gimbal refaktörü güdüm yasasını DEĞİŞTİRMEDİ.**
`d0da79d..HEAD` arasında `takip_gudum.py`'de kontrol yoluna ait tek satır bile
değişmedi; tek fark `cevre_mount_deg()`'in artık kamera eksenini
`$YILDIZ_TILT`'ten okuması (eksenin *kaynağı*, değeri değil). Yasadaki gerçek
değişiklik benim eklediklerim: **ilerleme saati** (varsayılan) ve isteğe bağlı
**görsel hakem**. Değişen asıl şey **tesisin fiziği**: `Benzetim`'de
`gimbal_kamera` varsayılanı `False → True`.

Bu ikisini ayırmak için 2×2 (6 senaryo × 3 tohum, 40 s, min menzil ortancası):

**A) Gövdeye sabit kamera (ESKİ fizik)**

| kol | düz/kuy | düz/çap | elips/kuy | **elips/çap** | wand/kuy | wand/çap |
|---|---|---|---|---|---|---|
| ESKİ yasa (düz saat) | 1.35 | 1.29 | 1.87 | **21.43** | 5.20 | 17.79 |
| YENİ yasa (ilerleme) | 1.35 | 1.29 | 1.87 | **5.93** | 5.20 | 17.79 |
| YENİ + görsel hakem | 1.35 | 1.29 | 1.87 | 5.93 | 5.20 | 17.79 |

**B) Gimbal (YENİ fizik)**

| kol | düz/kuy | düz/çap | elips/kuy | **elips/çap** | wand/kuy | **wand/çap** |
|---|---|---|---|---|---|---|
| ESKİ yasa (düz saat) | 1.35 | 1.69 | 1.87 | **20.60** | 4.85 | 16.44 |
| YENİ yasa (ilerleme) | 1.35 | 1.69 | 1.87 | **2.89** | 4.85 | **13.24** |
| YENİ + görsel hakem | 1.35 | 1.69 | 1.87 | 2.89 | 4.85 | **11.54** |

Kadraj kaybı (yeni yasa): gövdeye sabit `0 / 7.0 / 0.9 / 18.5 / 22.9 / 23.4 %`
→ gimbal `0 / 0 / 0 / 10.1 / 19.3 / 2.1 %`.

### Okuma

1. **Dört senaryoda hiçbir şey değişmedi** (1.35 / 1.87 zaten çarpışma; 5.20
   ve 1.29→1.69 gürültü bandında). Yani "ciddi değişen bir şey" **yalnız zor
   iki senaryoda** var — ve orada çok ciddi.
2. **İki değişim ÇARPIŞIYOR, toplanmıyor** (elips/çapraz):
   - eski yasa + eski fizik: **21.43 m**
   - yalnız gimbal: 20.60 m → **%4 kazanç** (neredeyse hiç)
   - yalnız yasa: 5.93 m → **3.6×**
   - ikisi birden: **2.89 m** → **7.4×**
   Gimbalin CPA'ya tek başına katkısı yok denecek kadar az; ama yasa düzelince
   gimbalin değeri **açığa çıkıyor** (5.93 → 2.89, ayrıca 2.05×).
3. **wanderer/çapraz'da yasa tek başına HİÇBİR ŞEY yapmıyor** (17.79 → 17.79)
   ama gimballe birlikte 13.24, görsel hakemle 11.54. Orada darboğaz kadraj:
   kayıp %23.4 → %2.1 (11×).
4. **Görsel hakem bedava değil, KÂRLI:** eski fizikte nötrdü (5.93 = 5.93),
   gimballe wanderer/çaprazda 13.24 → 11.54. Menzili hiç okumayan kol artık
   menzil okuyandan **iyi**.

### Sonuç

Daha önce "gimbal tavanı yükseltir, algoritma tabanı düzeltir" demiştim; 2×2
bunu sayıyla doğruluyor ve bir şey ekliyor: **tavan yükselmeden taban
düzelmesi de tam getirisini vermiyor.** İkisi ayrı ayrı yapılırsa kazanç
%4 ve 3.6×; birlikte 7.4×.

## 6f. SAHİPLİK: TO_TEST maddeleri ve ORTAK MOTOR kime ait?

İki ayrı sahiplik sorusu var ve karıştırılmamalı.

### (a) `TO_TEST.md` maddeleri

Dosya MPC kök-neden analizinden doğdu ama maddelerin hepsi MPC'ye ait değil:

| madde | kime ait | neden |
|---|---|---|
| 1 (q_alan), 1b (ödül şekli), 1c (J'ye ek terim) | **MPC** | maliyet fonksiyonu; takip'te J yok |
| 3 (maliyet ufku), 7 (çözücü metriği/bütçe) | **MPC** | ufuk/çözücü; takip'te ikisi de yok |
| 6 (beta pitch gecikmesi) | **MPC** | `beta` MPC'nin kadraj değişkeni |
| **2 (devir kapısına geometri şartı)** | **ORTAK** | `bbox_to_redis.py` — devri **iki yasaya da** o veriyor |
| **4 (kör terminal)** | **ORTAK** | ikisi de terminalde körleşiyor |
| **5b (dinamik tilt takibi)** | **ORTAK** | gimbal ortak donanım; takip'in üst-kenar kaybı da bundan |
| **8 (bearing-angle TMA)** | **ORTAK** | yasadan bağımsız kestirici; takip'in yanal açığının doğal çözümü |
| **9 (ölçüm araçları)** | **ORTAK** | `karsilastir.py`/`kosu_anlat.py` iki metodu da okuyor |
| **10 (eyleyici modeli τ)** | **ORTAK** | **`mpc_test.py` içinde** — takip'in de motoru |
| 0 (replay harness) | ORTAK olabilir | takip'in çözücüsü yok ama kapalı-döngü replay ikisine de yarar |

**Takip'e özel, TO_TEST'te hiç olmayan maddeler:** PrecLand EKF (kare düşünce
sürdürme), yaw için FOV kontrolcüsü, görsel hakemin sim doğrulaması.

Ayrıca bir gözlem: **madde 1'in vardığı sonuç takip'te yapısal olarak zaten
var.** Madde 1 diyor ki "kapanma sinyalini menzilden değil BBOX ALANINDAN sür";
takip'te kapanma bir maliyet terimi değil, `hiz_kaynagi='tavan'` ile **mutlak
talep**. Yani takip, madde 1 için hazır bir **kontrol grubu**: "kapanma amaç
olarak kodlanırsa ne olur" sorusunun çalışan cevabı (elips 0.2 m, sonsuz 1.2 m).

### (b) `mpc_test.Benzetim` — asıl karışıklık burada

`Benzetim` **MPC'ye ait değil**: nokta-kütle avcı + gerçek sanal gimbal +
iskeletin LPF/kelepçe/ivme zinciri. İki yasanın da ortak zemini; yalnızca
tarihsel olarak MPC'nin dosyasında oturuyor. `takip_test.py` onu import ediyor.

Bu somut bir risk: **TO_TEST madde 10 tam da bu motorun eyleyici modelini
(τ) değiştirmeyi öneriyor** ve madde 7c `mpc_test.py:596`'daki bütçe kapısını.
Biri bunu yaptığında bu dosyadaki bütün referans sayılar (§6e 2×2 tablosu
dahil) **sessizce** kayar.

**Alınan önlem (bu dalda):** `takip_test.py` test 0 = **motor mührü** —
9 fizik sabiti + `gimbal_kamera` varsayılanı sabitlenmiş durumda. Motor
değişirse test **patlar** ve "sayıları tazele" der. Sessiz kayma yerine
gürültülü kırılma.

**Kalıcı çözüm (önerilir, yapılmadı):** `Benzetim` ve fizik sabitleri nötr bir
modüle (`guidance_allstar/benzetim.py`) taşınıp hem `mpc_test.py` hem
`takip_test.py` oradan import etsin. Yapılmadı çünkü `mpc_test.py` üzerinde
şu an başka bir ajan çalışıyor; taşıma iki tarafın koordinasyonunu ister.

## 7. Denenmemiş, denenmeye değer (öncelik sırası)

0. **SIRADAKİ DARBOĞAZ: devir anındaki kadraj kaybı.** İlerleme saati
   elips/çapraz'ı 21.4 → 5.9 m'ye çekince o senaryo artık ıskayla değil
   **kadraj kaybıyla** bitiyor. Çevrimdışı izde yeri kesin: devirden
   **1.8 s sonra, r≈46 m'de, hedef ÜST kenardan** çıkıyor (son görülen
   piksel y=14, kayıp anında y=−4). Yani hâlâ aynı zincir: ivmelenme
   burnu aşağı eğiyor → sabit kamera onunla iniyor → eksenin üstündeki
   hedef üstten çıkıyor. Kinematik şekillendirme (3 m/s²) bunu
   hafifletti ama çapraz devirde bitirmedi. Aday çözümler 1 ve 3.

1. **PrecLand'in EKF'i.** Kare düştüğünde hedefi ataletsel hızla sürdürmek.
   Şu an bbox bayatlayınca iskelet süzülüyor; kayıp anları doğrudan CPA'yı
   bozuyor. Kural uyumlu (kendi hızımız + son görü).
2. **PrecLand'in `land_slowdown` kapısı.** "Yatayda hizalanmadan alçalma"nın
   bizdeki karşılığı: "kadraj kenarına yaklaşmışken hız talebini kıs".
   MPC'nin FOV kısıtının tek parametreli ucuz hâli.
3. **Gimbal** (`MOUNT_FOLLOW_ON_ENTER`). Gerçek donanımda pitch-servo gimbal
   zaten var; sim'de `montaj_ayarla.py` ile A/B. Ortak dosyalara dokunduğu
   için bu branch'te yapılmadı — kıyas kirlenmesin.
4. **Menzil hızından kısmi ileri besleme.** `ṙ = v_hedef·u − v_biz·u` olduğu
   için `v_hedef·u = ṙ + v_biz·u` — yani hedef hızının LOS bileşeni izinli
   veriden kurulabilir (MPC `menzil_hizi`'yi zaten böyle kullanıyor).
   `mode_follow`'un silinen FF teriminin LOS bileşenini geri verir.
   **Kural yorumu gerektirir**, o yüzden yapılmadı: kullanıcı kararı.

---

## 8. Nasıl koşulur

```bash
cd guidance_allstar && python3 takip_test.py          # 55/55, sim gerekmez

# tam deneme (METOT=takip)
SURE=360 GORUNTULU="takip_gudum.py" PLAN=missions/hedef_sonsuz.plan tools/senaryo.sh

# ablasyonlar
GORUNTULU="takip_gudum.py --hiz-kaynagi p"     # saf mode_follow
GORUNTULU="takip_gudum.py --fren ap"           # AP 'yanında dur'
GORUNTULU="takip_gudum.py --yasa poscon"       # Copter >= 4.5 yön yasası
GORUNTULU="takip_gudum.py --ivme-sekil 0"      # şekillendirme kapalı
```

Tanı logu: `guidance_allstar/logs/takip_tani_*.csv` (kolonlar bilinçli olarak
`mpc_tani` ile örtüşür: `durum`, `vurus`, `menzil`, `menzil_hizi`, `en_iyi`,
`cmd_*`, `vibe`, `vuruldu`).
