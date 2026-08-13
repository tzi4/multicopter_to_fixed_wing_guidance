# donanim/ — ROS'suz gercek donanim katmani

> Güncel varsayılan (2026-08-11): `gudum_tek_dugum.py --gudum los
> --buyuk-kare 5 --alan-pct 3`. Aşağıdaki MPC anlatımı tarihsel mimari
> ayrıntıları da içerir; kısa ve güncel uçuş sırası için
> `../NOTLAR_LOS_DONANIM.md` dosyasını kullan.

Iki dosya, iki ayri is:

| Dosya | Isi |
|---|---|
| **`kamera_kopru.py`** | **Kamera -> tespit -> `tracker_bbox` / `tracker_bbox_stab` + gercek gimbal komutu.** ROS'suz. Bolum **10**'a bakin. |
| `gudum_tek_dugum.py` | O kanallari okuyup **angajman karari + varsayılan LOS/PN + hiz komutu**. MPC yalnız `--gudum mpc` A/B seçeneğidir. |

Ikisi birlikte sim'deki `bbox_to_redis.py` + `goruntulu_temel.py` + `mpc_gudum.py`
zincirinin donanim karsiligidir. `kamera_kopru.py` olmadan donanimda
`tracker_bbox` kanalini **yayinlayan kimse yoktur** — `gudum_tek_dugum.py`
sessizce bos dinler.

---

## 1–9: `gudum_tek_dugum.py`

`gudum_tek_dugum.py`: **bagimsiz, tek surecli, kendi kararini veren** goruntulu
gudum dugumu. Mevcut `bbox_to_redis.py` + `goruntulu_temel.py` + `mpc_gudum.py`
zinciriyle **ayni isi** yapar; farki mimaridedir.

Mevcut dosyalarin hicbiri degistirilmedi — gudum yasasi ve emniyet
katmani **import** edilerek yeniden kullanilir.

---

## 1. Neden var

Bugunku zincirde karari **tespit sureci** veriyor:

```
bbox_to_redis.py  --(Redis 'komut_yetkisi' = goruntulu|konumlu)-->  gudum
goruntulu_temel.GoruntuluDongu   ->  yetkiyi BEKLER, gelince MPC'yi kosar
```

Yani "angaje olayim mi" sorusunu **goren goz** cevapliyor, silahi tutan el
degil. Takim arkadasinin itirazi bu. Yeni dugum ayni islevi **tek surecte** ve
**kendi karariyla** yapar:

```
bbox_to_redis.py --('tracker_bbox_stab' yayini = SALT OLCUM)--> gudum_tek_dugum.py
gudum_tek_dugum: kapilari degerlendir -> ANGAJE -> MPC -> hiz komutu -> BIRAK
```

`komut_yetkisi` **hic okunmaz**.

---

## 2. Mimari fark tablosu

| | Mevcut (bbox_to_redis + goruntulu_temel + mpc_gudum) | Yeni (`gudum_tek_dugum.py`) |
|---|---|---|
| Surec sayisi | 2 (tespit karar verici + gudum) | **1** (tespit yalniz olcum yayinlar) |
| Angajman karari | bbox_to_redis verir, Redis `komut_yetkisi` ile bildirir | **Gudum kendi verir** (`AngajmanKapisi`) |
| Karar kurali | 25 karede >=20 gecerli + alan %2 + menzil <=60 m | **Ayni kural**, siniftan sinifa tasindi |
| Basit kural | `YILDIZ_GECIS_BASIT=1` + `YILDIZ_GECIS_KARE` | `--basit-kare N` (varsayilan **kapali**) |
| Gudum yasasi | `mpc_gudum.MpcKontrolcu` | **Ayni sinif, import** (kopya yok) |
| Emniyet katmani | `GoruntuluDongu.calistir()` govdesinde | Ayni mantik tasindi (import edilebilir yuzeyi yok) |
| Menzil | estimator (`MenzilKestirici`), tek kaynak | estimator **veya** Redis (`--menzil-kaynak`) |
| Menzil kesilirse | `None` -> MPC `menzil_yoksa_m` (55 m) varsayar | **Son gecerli menzil DONDURULUR + uyari** |
| Kalp atisi | `goruntulu_hayatta` (bbox_to_redis okur) | `tekdugum_hayatta` (cakismaz) |
| Durum ilani | yok (yalniz `komut_yetkisi`) | `tekdugum_durum` + **MAVLink STATUSTEXT** |
| ISKA (MPC birak) | `goruntulu_birak` yazilir, bbox_to_redis **onaylar** | Dugum **kendi birakir**, onay beklemez |
| Log | `goruntulu_*.csv` (75 kolon) | Ayni 75 kolon + 15 tek-dugum kolonu |
| Masa testi | yok | `--dry-run` / `--sahte-mavlink` |

---

## 3. Ne yapar (adim adim)

1. **Olcum**: Redis `tracker_bbox_stab` kanalina abone olur
   (`[sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]`). Sanal gimbal
   kapaliysa 7 elemanli ham `tracker_bbox` kanalindan piksel merkeziyle
   acisal hata **turetilir** (yedek yol; govde salinimindan arindirilmamistir,
   kosuda gorulurse gimbal zincirinde ariza var demektir).
2. **Menzil** (yalniz menzil — proje kurali): `--menzil-kaynak estimator`
   (varsayilan) hedefin `GLOBAL_POSITION_INT`'ini kendi MAVLink baglantisindan
   (udpin:14604, `source_system 252`) okuyup IMM'e verir ve **disariya yalnizca
   menzil** cikar. `--menzil-kaynak redis` ise `devir_durumu.range_m` okunur.
   Menzil kesilirse **son gecerli deger dondurulur** ve stderr'e uyari basilir;
   `menzil_taze` / `menzil_yas_s` kolonlarindan sonradan ayirt edilir.
3. **Karar**: `AngajmanKapisi` (asagida).
4. **Gudum**: `mpc_gudum.MpcKontrolcu` (import) `Olcum` alir, `Komut` doner.
5. **Komut**: `SET_POSITION_TARGET_LOCAL_NED`, yalniz `vx,vy,vz` (+ istege bagli
   `yaw_rate`), avci baglantisi udpin:14654 / `source_system 251`.
   Emniyet: komut LPF (tau 0.35 s) -> hiz kelepcesi
   (`GORUNTULU_MAX_SPEED_MPS`) -> **irtifa tabani 15 m** -> yaw slew
   (120 dps^2) + yaw LPF (0.15 s).
6. **Ilan**: `tekdugum_durum`, `tekdugum_hayatta` (TTL), STATUSTEXT.
7. **Log**: `donanim/logs/tekdugum_mpc_<damga>.csv` (+ `_olay.csv` +
   MPC tani logu), 20 satirda bir flush.

---

## 4. Angajman durum makinesi

```
BEKLE  --(angajman kapisi acildi)-->  ANGAJE
ANGAJE --(kayip merdiveni | tespit kaybi penceresi | MPC ISKA)--> BIRAK -> BEKLE
                                                          |
                                                    SOGUMA 3 s
```

### Kapi (varsayilan: uc kapi)

| Kapi | Esik | Env |
|---|---|---|
| Kararlilik | son **25 karenin >= 20**'si gecerli (%80) | `YILDIZ_PENCERE_KARE` / `YILDIZ_PENCERE_ORAN` |
| Alan | bbox alani kadrajin (%2 x %2) dikdortgeninden buyuk | `YILDIZ_GECIS_ALAN_PCT` |
| Menzil | `menzil <= 60 m` (menzil yoksa/bayatsa **atlanir**) | `YILDIZ_GECIS_MENZIL` |

Ucu de `bbox_to_redis.py` ile **ayni** sayilar ve ayni env degiskenleridir.

**Pencere KARE cinsindendir, sure degil.** 25 kare 30 fps'te 0.83 s, 20 Hz'lik
YOLO'da 1.25 s eder. (Eski 45 kare sirasiyla 1.5 s / 2.25 s idi; kod
degismeden donanimda yarim saniye gec angaje oluyordu.) Bu farki logdan tahmin
etmeden gormek icin **iki hiz da loglanir**:

* `tespit_fps` — bbox_to_redis'in **yayin** hizi (ayri sayac thread'i olcer)
* `gozlem_fps` — dongunun **gordugu** ayri kare hizi = `min(loop_hz, tespit_fps)`

Pencere gozlenen karelerle isler (gudum ancak gordugu kareye tepki verebilir),
yani 20 Hz'lik bir donguda 25 kare 1.25 s eder — kamera 30 fps olsa bile.
Daha kisa isteniyorsa `--loop-hz 30` ya da `--pencere-kare`.

Alternatifler:
* `--basit-kare 5` — uc kapi atlanir, "art arda 5 gecerli kare" yeter
  (bbox_to_redis'teki `YILDIZ_GECIS_BASIT` mantigi). Varsayilan **kapali**.
* `--pencere-s 1.5` — pencere kare yerine **sure** cinsinden kurulur
  (varsayilan degil; fps cok oynayan donanim icin hazir duruyor).

### Birakma

| Yol | Kosul |
|---|---|
| Kayip merdiveni | bbox yasi > `--birak-s` (2.5 s) kesintisiz |
| Tespit kaybi penceresi | penceredeki gecerli kare <= 3 **ve** 2 s dwell (titrek tespit) |
| ISKA | `mpc_gudum` durum makinesi `Komut.birak=True` dedi |
| Kapanis | SIGINT/SIGTERM — angajedeysek birakip cikar |

Kayip merdiveni (goruntulu_temel ile ayni esikler ve gerekceler):

| bbox yasi | durum kolonu | komut |
|---|---|---|
| <= 0.7 s | `taze` | MPC kosar |
| 0.7 – 1.7 s | `tut` | son gecerli komut tutulur (yaw haric) |
| > 1.7 s | `suz` | **olculen hiza sonumleme** (sifir degil — sifir tam fren komutudur) |
| > 2.5 s | — | BIRAK |

---

## 5. Redis anahtarlari

| Anahtar | Yon | Icerik |
|---|---|---|
| `tracker_bbox_stab` | **okur** (abone) | `[sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]` |
| `tracker_bbox` | **okur** (abone) | ham bbox — kapsama + yedek acisal hata |
| `devir_durumu` | **okur** | warm-start tohumu (`cmd_vel_ned`) + `--menzil-kaynak redis` icin `range_m` |
| `tekdugum_durum` | **yazar** | JSON angajman durumu (TTL yok, `t_mono`/`t_unix` ile bayatlik) |
| `tekdugum_hayatta` | **yazar** | TTL'li (2 s) kalp atisi / olu-adam anahtari |
| `komut_yetkisi` | **yazar** (ilan) | angajede `goruntulu`, birakinca `konumlu`. **Okunmaz.** `--no-yetki-yaz` ile kapanir |

`tekdugum_durum` ornegi:

```json
{"durum": "ANGAJE", "sebep": "uc_kapi(25/25 kare, alan %2, menzil <= 60 m, fps=19.9)",
 "yontem": "mpc", "kural": "uc_kapi_kare", "pid": 95009, "t_mono": 14259.14,
 "t_unix": 1786063317.7, "menzil_m": 48.92, "tespit_fps": 29.6,
 "gozlem_fps": 19.9, "gecerli_oran": 1.0, "kuru": false}
```

> **UYARI — cakisma:** `bbox_to_redis.py` karar verici rolunde de
> `komut_yetkisi` yaziyor (yalniz mod degisiminde). Ikisi ayni anda kosarsa
> **son yazan kazanir**. Tek dugum kurulumunda `bbox_to_redis` salt tespit
> rolunde kalmali; konumlu surec (`simple_guided_follow.py`) hic kosmuyorsa
> `--no-yetki-yaz` verin. (Kalici cozum `bbox_to_redis`'e bir `--no-karar`
> bayragi eklemektir; bu is kapsaminda mevcut dosyalar degistirilmedi.)

---

## 6. formation_KILLER entegrasyonu

Iki kanal var; **otorite Redis'tir**, STATUSTEXT insan/GCS ekosudur.

### a) Redis `tekdugum_durum` (makine okunur)

```python
import json, redis
r = redis.Redis()
d = json.loads(r.get('tekdugum_durum') or b'{}')
if d.get('durum') == 'ANGAJE':
    ...   # arac goruntulu gudumde: formation slot komutu GONDERME
```

Canlilik: `tekdugum_hayatta` anahtari TTL'li (2 s). **Yoksa surec olmustur** —
`tekdugum_durum` eski bir `ANGAJE` gosteriyor olabilir, ona guvenmeyin.

### b) MAVLink STATUSTEXT (protokol metinleri)

| Metin | Anlam |
|---|---|
| `GORUNTULU: komut bende` | Dugum angaje oldu; araci artik o suruyor |
| `GORUNTULU: biraktim, dinliyorum` | Dugum birakti; arac formation/konumlu tarafina serbest |

`formation_KILLER.py` STATUSTEXT'i zaten logluyor
(`_handle_statustext` -> `swarm.log` + `events.jsonl`). Metinler
`gudum_tek_dugum.ST_ANGAJE` / `ST_BIRAK` sabitlerinde; degistirilirse iki
tarafta birden degistirilmeli. Mesajlar `source_system 251` ile gider, yani
panelde **d251** olarak gorunur (aracin kendi sysid'si degil).

`--no-statustext` ile kapatilir (Redis ilani devam eder).

---

## 7. Calistirma

### Sim

```bash
# 1) tespit + sanal gimbal (KARAR VERICI ROLU KULLANILMIYOR)
python3 bbox_to_redis.py --no-display &

# 2) tek dugum gudum (varsayilan: uc kapi, estimator menzili)
python3 donanim/gudum_tek_dugum.py --loop-hz 20
```

### Pi / donanim

```bash
# YOLO ~20 Hz basiyorsa 25 kare 1.25 s eder; loglarda tespit_fps'i dogrulayin
python3 donanim/gudum_tek_dugum.py \
    --loop-hz 20 --menzil-kaynak estimator --mount 30
```

### Masa testi (arac/SITL YOK)

```bash
python3 donanim/gudum_tek_dugum.py --dry-run --sahte-mavlink --sure 15
```

`--dry-run` komutu **hesaplar ve loglar, MAVLink'e yazmaz**.
`--sahte-mavlink` ayrica arac baglantisini da kurmaz (sahte durum + sahte
kapanan menzil) ve `--dry-run`'i kendiliginden acar.

### Onemli CLI bayraklari

```
--loop-hz 20              gudum dongusu [Hz]
--menzil-kaynak {estimator,redis}
--pencere-kare 25         kararlilik penceresi [kare]
--gecerli-oran 0.8        penceredeki asgari gecerli oran (-> 20/25)
--pencere-s X             ISTEGE BAGLI: pencereyi sure cinsinden kur
--basit-kare N            BASIT kural (uc kapi atlanir), varsayilan kapali
--alan-pct / --gecis-menzil    alan ve menzil kapisi esikleri
--birak-s 2.5 / --soguma-s 3.0
--no-statustext / --no-yetki-yaz
--dry-run / --sahte-mavlink
--mount / --aim / --no-yaw / --hiz-tavani / --ufuk / --adim-s / --no-iska  (MPC)
```

---

## 8. Log kolonlari

Ana CSV = `goruntulu_temel.LOG_KOLONLARI` (**import edilir**, 75 kolon —
saglik kolonlari `dongu_hz_ort`, `dt_asim`, `hayatta_ttl`, `ap_mod`,
`hb_yas_s` dahil) + su 15 tek-dugum kolonu:

| Kolon | Anlam |
|---|---|
| `tekdugum_durum` | `BEKLE` / `ANGAJE` |
| `kural` | `uc_kapi_kare` / `uc_kapi_sure` / `basit` |
| `tespit_fps` | bbox yayin hizi (LOG-ONLY, ayri sayac thread'i) |
| `gozlem_fps` | dongunun gordugu kare hizi (pencere bununla isler) |
| `pencere_ornek` | penceredeki ornek sayisi (<= 25) |
| `gecerli_kare` | penceredeki gecerli ornek sayisi (esik 20) |
| `gecerli_oran` | oran [0..1] |
| `kapi_alan` | alan kapisi acik mi (0/1) |
| `kapi_menzil` | menzil kapisi acik/atlanmis mi (0/1) |
| `ardisik_kare` | basit kural sayaci |
| `kayip_s` | kesintisiz bbox kaybi [s] |
| `menzil_kaynak` | `estimator` / `redis` / `sahte` |
| `menzil_taze` | 1 = olculdu, 0 = **dondurulmus** |
| `menzil_yas_s` | son gecerli menzilden beri gecen sure |
| `kuru` | 1 = `--dry-run` (komut MAVLink'e gitmedi) |

Satirlar **sozlukten** kurulur (indisten degil): `goruntulu_temel`'e yeni bir
kolon eklenirse burada sessizce kaymaz, bos kalir.

`yetki` kolonu eski araclarla uyum icin `goruntulu`/`konumlu` yazmaya devam
eder (angaje = `goruntulu`); asil durum `tekdugum_durum` kolonundadir.

---

## 9. Cevrimdisi dogrulama

`py_compile` + sahte Redis / sahte MAVLink harness'i (bkz. gorev raporu;
harness `/tmp/tekdugum_kuru_test.py`). Dogrulanan davranislar:

* uc kapili kural 25 gozlenen karede ANGAJE oluyor (`--sure 15`),
* `--basit-kare 5` ile 5 karede ANGAJE oluyor,
* tespit kesilince `taze -> tut -> suz -> BIRAK` merdiveni isliyor,
* soguma sonrasi yeniden ANGAJE oluyor,
* komut vektoru uretiliyor (`cmd_vx/vy/vz`, `|v|` 12–31 m/s),
* `--dry-run`'da MAVLink'e **0** komut gidiyor,
* `tekdugum_durum` / `tekdugum_hayatta` (TTL 2) / `komut_yetkisi` yaziliyor,
* menzil kaynagi bayatlayinca son deger donduruluyor (`menzil_taze=0`).

---
---

# 10. `kamera_kopru.py` — kamera koprusu (ROS'suz giris noktasi)

## 10.1 Neden var

Sim'de `tracker_bbox` / `tracker_bbox_stab` kanallarini `bbox_to_redis.py`
uretiyor — ama o dosya bir **ROS Image abonesi**dir ve Pi 5'te ROS yoktur.
Yani gercek donanimda **o iki kanali yayinlayacak kimse yoktu**; gudum
tarafi (`gudum_tek_dugum.py`, `goruntulu_temel.py`) ise sadece onlari
dinler. `kamera_kopru.py` o boslugu kapatir:

```
kamera / sahte hedef -> tespit (hsv|yolo|sahte) -> SanalGimbal zinciri
   -> Redis 'tracker_bbox' + 'tracker_bbox_stab'
   -> (istege bagli) TiltTakip -> MavlinkTiltKomutcu -> ArduPilot mount -> servo
```

**Kopya yok, hepsi import:** `bbox_to_redis.TutumOkuyucu` (MAVLink ATTITUDE,
zaman damgali tampon + kare aninda interpolasyon), `bbox_to_redis.hsv_tespit`
(sim ile **ayni** tespit kodu), `yildizlar_gimbal.SanalGimbal` +
`eklem_acisi`, `tools.gz_gimbal.TiltTakip`, `tools.mavlink_tilt.
MavlinkTiltKomutcu`. Gimbal arka ucu **Gazebo degil MAVLink**'tir.

**Bu kopru karar verici DEGILDIR:** `komut_yetkisi` anahtarina dokunmaz.
Donanimda angajman karari gudum dugumunundur (bolum 4). Burasi yalniz olcum
yayinlar.

## 10.2 Uc kademeli devreye alma

Sirayi ATLAMAYIN — her kademe bir sonrakinin varsayimini dogrular.

### Kademe 1 — kamera YOK, gimbal YOK (masada, 30 saniye)

```bash
python3 donanim/kamera_kopru.py --dedektor sahte --kaynak dosya \
    --no-gimbal --sahte-bbox 940,360,60,30 --log /tmp/k1.csv --sure 10
```

Hedefi kadrajin **istediginiz yerine koyar** ve tum zinciri (stabilizasyon +
Redis + log) gercekmis gibi kosturur. "Hedefi suraya koyunca MPC nereye
komut veriyor" sorusu boylece ucus olmadan cevaplanir: ayni anda
`python3 donanim/gudum_tek_dugum.py --dry-run --sahte-mavlink` kosturun.

Beklenen isaretler (1280x720, hfov 66 -> `fx=985.5`, `cx=640`, `cy=360`):

| `--sahte-bbox` (merkez) | `ex` | `ey` |
|---|---|---|
| `610,345,60,30` (640,360 = tam orta) | **0.000** | **0.000** |
| `910,345,60,30` (940,360 = SAGDA) | **+16.931** | 0.000 |
| `610,145,60,30` (640,160 = YUKARIDA) | 0.000 | **−11.472** |
| `610,545,60,30` (640,560 = ASAGIDA) | 0.000 | **+11.472** |

**Isaret sozlesmesi:** sag -> `ex > 0`; yukari -> `ey < 0` (ve hedefin dunya
yukselisi `-ey > 0`). Bu tablodan sapiyorsa daha ileri gitmeyin.

### Kademe 2 — kamera VAR, gimbal YOK

```bash
python3 donanim/kamera_kopru.py --kaynak cv2 --cihaz 0 --dedektor hsv \
    --no-gimbal --mount 30 --log /tmp/k2.csv --goster
```

Mor bir cisim tutun. `--goster` bassiz Pi'de **kapali** birakilir; onun
yerine `--kaydet` ile mp4 alin. Pi'nin libcamera yiginda `cv2.VideoCapture(0)`
acilmazsa: `--kaynak picamera2`, ya da
`rpicam-vid -t 0 --inline --width 1280 --height 720 --codec h264 -o udp://127.0.0.1:8554`
+ `--kaynak dosya --dosya udp://127.0.0.1:8554`.

`--mount`, kamera **govdeye sabitken** montaj acisidir; `ham_ey` ile
`stab_ey` arasinda tam olarak `--mount` kadar fark gorunmelidir (olculdu:
mount +30 -> fark 30.000 deg).

### Kademe 3 — kamera VAR, gimbal ACIK

```bash
python3 donanim/kamera_kopru.py --kaynak cv2 --cihaz 0 --dedektor hsv \
    --mavlink /dev/ttyACM0 --tilt 0 --tilt-alt -35 --tilt-ust 55 \
    --log /tmp/k3.csv
```

**Once `tools/mavlink_tilt.py` sweep testini gecin** (bkz.
`donanim/GIMBAL_TAKIP_TESTI.md`): komut zinciri kanitlanmadan takip
denenmez. Basari olcutu: cismi dikeyde gezdirince servo ~yarim saniyede
izler ve cisim kadrajda dikeyde merkeze oturur; cismi saklayinca 3 s tutar,
sonra `--tilt` degerine ~10 deg/s ile doner.

`--mavlink` verildiginde **tek** MAVLink baglantisi acilir ve tutum okuyucu
ile tilt komutcusu onu **paylasir**. Sebep: `udpin:` adresi ikinci kez
acilamaz (port bind), seri port ikinci acilista bozuk cerceve verir.
Mission Planner acikken seri portu paylasamazsiniz — mavproxy ile bolun:
`mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14601 --daemon`,
sonra `--mavlink udpin:127.0.0.1:14601`.

## 10.3 COZUNURLUK — sessiz felaket

Piksel -> aci donusumu `fx = (genislik/2)/tan(hfov/2)`, `cx = genislik/2`
ile yapilir. Varsayilan cerceve **1280x720, hfov 66** -> `fx=985.5, cx=640`.

Kameraya **1920x1080** besleyip cerceveyi guncellemezseniz, kadrajin **tam
ortasindaki** hedef `px=960`'ta gorunur ve zincir onu
`atan((960−640)/985.5) = +18.0 deg saga sapmis` sanar. Hata mesaji cikmaz,
gudum bos yere doner. Bu yuzden kopru:

* `--genislik/--yukseklik/--hfov` degerlerini **`SanalGimbal`'e gecirir**
  (`bbox_to_redis` bunu yapmiyordu, hep 1280x720 varsayiyordu),
* gelen kare farkli boyuttaysa **kac derece sapma olacagini hesaplayip**
  uyarir ve kareyi yeniden olcekler,
* `--boyut-kati` verilirse olceklemek yerine **cikar** (exit 1).

Dogrulandi: 1920x1080 kaynak, `--genislik 1280` (olcekleme) ve
`--genislik 1920` (dogal) kosularinin ikisi de `ex = +17.99 deg` verdi.

**Kural:** kameranizin gercek cozunurlugunu ve gercek HFOV'unu verin;
olcekleme bir emniyet agidir, cozum degildir.

## 10.4 YOLO kolu — letterbox geri donusumu

Ag 640x640 gibi sabit bir girise **letterbox** ile beslenir; ciktisi o
tuvalin pikselindedir. Isleme cercevesine geri tasinmazsa hedef sol ust
kosele kayar ve `ex/ey` ~2 kat kucuk okunur — **sessiz** bir hata.

`ultralytics.predict()` geri donusumu kendisi yapar ve kutulari **verdiginiz
karenin** pikselinde dondurur; sart, kareyi **oldugu gibi** vermektir (once
elle resize edip sonra geri olceklemeyi unutmak klasik hatadir). Kopru tam
olarak isleme cercevesini verir ve her kutuyu cerceve sinirlarina karsi
**saglar**; tasma varsa `UYARI: ... letterbox geri donusumu bozuk` basar.

IMX500 (.rpk) kolunda ag sensorde kosar, kutular kare metadata'sindan
`convert_inference_coords()` ile cerceveye tasinir (elle carpma yapmayin).
Bu kol **donanimda henuz dogrulanmadi**; Pi'de ilk kosuda `--dedektor sahte`
ve `--dedektor hsv` ile karsilastirilarak dogrulanmali.
`ultralytics` / `picamera2` yoksa ikisi de kurulum komutunu soyleyen
anlasilir hata verir.

## 10.5 Log kolonlari — sahada teshis nasil okunur

`--log X.csv`, 20 satirda bir flush (cakilmada kaza anina en yakin blok
kaybolmasin).

| Kolon | Anlam |
|---|---|
| `t` | `time.monotonic()` — Redis `t_capture` ile ayni saat |
| `tilt_cmd_deg` | takip yasasinin **istedigi** elevasyon |
| `tilt_status_deg` | komutcunun **son yayinladigi** deger (olu bant/hiz siniri sonrasi) |
| `ham_ex_deg`, `ham_ey_deg` | hicbir de-rotasyon olmasaydi okunacak aci |
| `stab_ex_deg`, `stab_ey_deg` | sanal gimbal zincirinden cikan aci (gudumun gordugu) |
| `bbox_w`, `bbox_h` | tespit boyutu (menzil/alan kapisinin girdisi) |
| `gecerli` | 1 = tespit var |
| `fps` | son ~60 karenin hizi |

**Teshis agaci — arizayi ikiye boler:**

* `ham_*` **yanlis** (hedef kadrajda ortadayken sifir degil) -> sorun
  **tespitte ya da ic parametrelerde**: cozunurluk/HFOV uyusmazligi,
  yanlis `--genislik`, YOLO olcek geri donusumu, HSV'nin yanlis blob'u
  secmesi. Once bolum 10.2 kademe 1 tablosuna donun.
* `ham_*` **dogru** ama `stab_*` **yanlis** -> sorun **tutum / roll / zaman
  senkronunda**: MAVLink ATTITUDE gelmiyor (`--mavlink` yok mu?), roll
  isareti ters, ya da kamera boru hatti gecikmesi olculmemis
  (`--kamera-gecikme-ms`, bkz. `tools/gimbal_zaman_kalibre.py`). Govde
  duruyorken ikisi **esit** olmalidir (gimbal kapali ve `--mount 0` iken).
* `tilt_cmd_deg` hareket ediyor ama servo kimildamiyor -> sorun **komut
  yolunda**: `tools/mavlink_tilt.py` sweep testini kosun.
* `fps` beklenenden dusuk -> pencere kapilari **kare** cinsindendir; devir
  gecikir (bolum 4'teki `tespit_fps` notu).

Ayrica ~1 saniyede bir **stderr**'e tek satirlik ozet basilir (stdout'a
degil, boru hattina verilen olcum akisina karismasin):

```
[KOPRU] fps= 29.9 kare=418 tespit=%97 bbox=(910,345,60,30) ex=+16.93 ey= +0.00 deg tilt(cmd/st)=+11.5/+11.4 deg
```

## 10.6 Bilinen sinir (ACIK IS)

`tilt_status_deg` **bagimsiz bir aci geri beslemesi degildir** — son
yayinlanan komuttur. Sim'deki `gimbal_tilt_status`'un donanim karsiligi
`MOUNT_ORIENTATION` mesaji olurdu; onu okumak ayni baglantida **ikinci bir
recv tuketicisi** ister ve pymavlink thread-safe degildir. ArduPilot mount
surucusu komutu <1 s'de oturttugu icin yavas rejimde guvenlidir; hizli
terminal manevrada gecikme payi vardir. Cozulmesi gereken is: paylasilan
baglantida tek recv tuketicisinden `MOUNT_ORIENTATION` dagitimi.

**Bunun OLCULEN sonucu — komutta asim (cevrimdisi benzetim, hedef +20 deg):**
zincir kameranin acisi olarak *komutu* varsaydigindan, servo geride
kaldiginda takip yasasi bir **integratore** doner (`cmd_yeni = cmd +
(hedef − gercek_aci)`). Kelepce ve slew siniri yuzunden kacmaz, ama komut
gercek acinin onune gecer:

| Servo gecikmesi | Tepe komut | Asim | Oturma |
|---|---|---|---|
| yok (ideal) | +20.00 | %0 | 0.53 s |
| tau = 0.15 s | +24.03 | %20 | 0.57 s |
| tau = 0.30 s | +27.35 | %37 | 0.73 s |
| tau = 0.60 s (agir gimbal) | +32.77 | %64 | 0.93 s |

Sahada anlami: `--tilt-alt/--tilt-ust` kelepcesini mekanik uclara
**yapistirmayin** (varsayilan −35/+55, gercek uclar −38/+58 — pay bilerek
birakildi) ve ilk masa kosusunda `tilt_cmd_deg` kolonunun tepe degerine
bakin. `MOUNT_ORIENTATION` okuyucusu eklenince bu asim **sifirlanir**
(sim'deki `gimbal_tilt_status` yolu boyle calisir).

Kusursuz servo varsayimiyla kapali dongu **dogrulandi**: hedef dunyada
+20.0 deg'de sabitken tilt komutu 0.5 s'de **+20.007 deg**'e oturdu ve
hedef kadrajda dikeyde merkezden **0.1 piksel** uzaga dustu; tespit
kesilince 3 s tutup ~10 deg/s ile `--tilt` degerine dondu.

## 10.7 Redis sozlesmesi (`bbox_to_redis` ile BIREBIR)

```
tracker_bbox       [x, y, w, h, kapsama_pct, gecerli, t_capture]
tracker_bbox_stab  [sx, sy, w, h, ex_deg, ey_deg, t_capture, tilt_eps]
```

`t_capture` **`time.monotonic()`** tabanlidir (sim'de ROS saatiydi).
Tuketiciler onu kare **imzasi** ve yas hesabi icin kullanir; CLOCK_MONOTONIC
Linux'ta sistem capinda oldugu icin iki surec dogrudan karsilastirir.
8. eleman (`tilt_eps`) o karede zincirin kullandigi kamera elevasyonudur;
gimbal kapaliyken `null`'dur ve `mpc_gudum` `ey_ref`'i ona gore kurar.
