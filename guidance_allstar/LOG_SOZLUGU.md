# LOG SOZLUGU

Bu dosya gudum yiginindaki HER log kolonunun ne oldugunu, biriminin ne
oldugunu, isaretinin nereye baktigini ve gordugun degerin ne ANLAMA geldigini
anlatir. Amac tek cumleyle: **bu dosyayi okuyan bir ajan, loglara bakip
kosuda ne oldugunu kullaniciyla ayni netlikte anlatabilmeli.**

Loglarin "hangi sayilar vardi"yi anlatip "ne oldu"yu anlatmamasi somut bir
maliyet dogurdu (2026-08-04): kullanici bir videoda "hedef bana dogru
geliyordu ama MPC hedeften kaciniyor gibi" dedi; bunu dogrulamak icin hedefin
gidis yonu ile kerteriz arasindaki aci ELLE hesaplanmak zorunda kaldi, cunku
hicbir logda "bu karsilasma kuyruk takibi mi, kafa kafaya mi" yazmiyordu.
Cikan sonuc (yakin menzilde karsilasmalarin buyuk kismi kafa kafaya) logda
OLMALIYDI. Bu sozluk ve `ref_*` kolonlari o boslugu kapatmak icin var.

Hizli baslangic: bir kosuyu **okumadan once anlat**:

    python3 tools/kosu_anlat.py run/denemeler/<deneme>

---

## 0. ALTIN KURAL: `ref_` oneki

`ref_` ile baslayan her kolon **YALNIZ ANALIZ** icindir; gudum onu GORMEZ.

Kullanici kurali (2026-08-03): hedefin 3D telemetrisine az guveniyoruz;
gudumun ondan turetmesine izin verilen TEK buyukluk **menzildir**
(`menzil_m`). Hedefin konumu, hizi, ivmesi, donusu gudume GIRMEZ.

Ama ANALIZ bunlar olmadan yapilamaz. Cozum ayrimdir:

| kolon | kim okur |
|---|---|
| `menzil_m` | GUDUM okur (izinli tek hedef buyuklugu) + analiz |
| `ref_*` | YALNIZ analiz/arac. Kontrolcu okursa bu bir KURAL IHLALIDIR. |

Kodda ayrim gorunur: `MenzilKestirici.menzil()` gudume acilir,
`MenzilKestirici.ref_hedef_durum()` yalniz log yazicisi tarafindan ve komut
gonderildikten SONRA cagrilir (`goruntulu_temel.py`, "LOG" basligindan
sonrasi). Bir kontrolcude `ref_hedef_durum` ya da `ref_` gorursen bildir.

---

## 1. ZAMAN VE UC LOGU HIZALAMA

Uc farkli saat dolasiyor. Karistirilirsa hizalama yanlis olur:

| ad | ne | nerede |
|---|---|---|
| `t_mono` | `time.monotonic()`. Linux'ta CLOCK_MONOTONIC **surecler arasi ORTAKTIR** (makine acilisindan beri gecen sure). Geri gitmez, NTP'den etkilenmez. | goruntulu CSV `t`/`t_mono`, konumlu CSV `wall_time` (adina ragmen monotonic!), mpc_tani CSV `t` |
| `t_unix` | `time.time()`, mutlak epoch saniyesi. Video dosyalari, dosya damgalari ve duvar saati ile tek koprü. | goruntulu CSV `t_unix` (2026-08-05'ten sonraki kosular), mpc_tani CSV `t_unix` (SONDA), bbox.log `t_unix=` (HEDEF satiri sonunda) |
| `t_capture` | Karenin YAKALANMA ani, ROS saati (`tracker_bbox_stab[6]`). **Baska saatle karistirma**; yalniz ardisik olcumlerin FARKI anlamlidir (gecikme-duyarli turevler, lambda-nokta faz eslemesi). | goruntulu CSV `t_capture`, mpc_tani |

**Hizalama tarifi**

1. goruntulu / konumlu / mpc_tani CSV'leri ayni monotonic saatte -> dogrudan
   yan yana konur. Baska donusum gerekmez.
2. Mutlak saate gecmek icin goruntulu CSV'den `ofset = t_unix - t` alinir;
   ayni ofset diger iki dosyaya da uygulanir (ayni makine, ayni saat).
   Eski (t_unix'siz) kosularda ofset dosyanin mtime'indan kestirilir (+-1 s).
3. `bbox.log` **ZAMAN DAMGASIZDIR** -- dedektor yalnizca satir basar. Onu
   zamana baglamanin tek yolu yeni goruntulu CSV'deki `px_ham_cx`/`px_ham_cy`
   kolonlaridir: ayni ham bbox merkezleri orada zaman damgasiyla durur.
   `tools/kosu_anlat.py` bbox.log'u yalniz toplu istatistik olarak kullanir.

Bunlarin hepsini `tools/kosu_anlat.py` icindeki `Hizalayici` sinifi yapar.

**Zaman ile ilgili tuzak:** konumlu surec, yetkiyi goruntuluye devrettikten
SONRA da olcup loglamaya devam eder (yalniz setpoint gondermeyi keser). Bu
yuzden iki CSV'yi zamana gore ic ice dizmek "her dongude yetki el degistiriyor"
yanilsamasi verir. Dogru kural: **goruntulu CSV yalniz yetkiliyken satir
yazar**; satirlar arasi 1 s'den buyuk bosluk = yetki konumluya dondu.
(`kosu_anlat.yetki_araliklari`, `karsilastir.py` ile ayni tanim.)

---

## 2. DOSYALAR

| dosya | uretici | ne anlatir |
|---|---|---|
| `guidance_allstar/logs/goruntulu_<metot>_<damga>.csv` | `goruntulu_temel.GoruntuluDongu` | GORUNTULU gudumun ana logu. Asagida tam sozluk. |
| `guidance_allstar/logs/goruntulu_<metot>_<damga>_olay.csv` | ayni | AYRIK OLAYLAR (devir, kayip, kisit, menzil esigi, en yakin gecis, iska). |
| `guidance_allstar/logs/mpc_tani_<damga>.csv` | `mpc_gudum.py` | MPC'nin IC tanilari (cozucu, CBF sinirlari, bozucu kestirimi). |
| `guidance_allstar/logs/guided_follow_<damga>.csv` | `simple_guided_follow.py` | KONUMLU gudum: standoff slotu, IMM kestirimi, kurtarma makinesi. |
| `logs/gimbal_<damga>.csv` | `bbox_to_redis.py --gimbal-log` | GIMBAL zinciri: komut vs gerceklesen tilt, ham vs stabilize hata, tutum interpolasyon boslugu. **2026-08-07'den beri HER kosuda uretilir** (`yildizlar_gudum.sh` bayragi varsayilan verir; `YILDIZ_GIMBAL_LOG=0` kapatir). 21 kolon, bkz. 4.5. |
| `logs/bbox.log`, `run/denemeler/<deneme>/bbox.log` | `bbox_to_redis.py` | Dedektor metni. Zaman damgasi YOK. |
| `run/denemeler/<deneme>/{guidance,goruntulu,aim,ozet}.{log,txt}` | `tools/senaryo.sh` | Kosu basina toplanan surec ciktilari. |

---

## 3. `goruntulu_<metot>_<damga>.csv` -- TAM SOZLUK

**75 kolon** (2026-08-07: 69 -> 70 `tilt_deg` ile, -> 75 sistem sagligi
blogu ile). Kolon SIRASI degil ADI baglayicidir (tum araclar
`csv.DictReader` kullanir; indis tabanli okuyan arac YOK -- kontrol edildi
2026-08-07). Tek kaynak: `goruntulu_temel.LOG_KOLONLARI`.
Bos hucre = "o an bu buyukluk YOKTU" (0 DEGIL -- ayrimi koru).

### 3.1 Zaman ve faz

| kolon | birim | anlam |
|---|---|---|
| `t` | s | `time.monotonic()`. Eski araclarla uyum icin korunan ad. |
| `t_mono` | s | `t` ile AYNI deger, adi acik olsun diye. |
| `t_unix` | s | `time.time()`, mutlak epoch. bbox/video hizalamasinin anahtari. |
| `dt` | s | **OLCULEN** dongu adimi (nominale guvenilmez). Tipik 0.050. |
| `yetki` | - | Bu satirda hep `goruntulu` (dongu yalniz yetkiliyken yazar). |
| `durum` | - | Kontrol yolunun hangi kolda gittigi: `taze` \| `tut` \| `suz`. |

`durum` degerleri:
* `taze` -- bbox var ve taze; kontrolcu gercek olcumle calisti. **Istenen.**
* `tut` -- KISA bosluk (bbox yasi 0.7-1.7 s): son gecerli komut TUTULUYOR,
  yaw_rate VARSAYILAN OLARAK tutulmuyor. Kor donus hedefi yatayda da
  kaybettirir.
  **[YILDIZ_TUT_YAW=1, 2026-08-08]** Env dugmesi acikken `tut` kolunda son
  yaw komutu SONUMLENEREK (`exp(-gecen/1.0 s)`) ve SURE SINIRLI (azami
  1.5 s) surdurulur; sonra `None`'a duser. `durum` yine `tut` yazilir --
  taban kosulariyla faz yuzdelerinin kiyaslanabilir kalmasi icin BILINCLI.
  Surdurmenin kac karede calistigi **`durum=='tut'` VE `cmd_yaw_rate_dps`
  bos degil** kosulundan okunur (kapali kolda bu oran tam %0'dir).
  Gerekce: elips hedefte donus karelerinin %38-42'sinde araca HIC yaw
  komutu gitmiyordu; hedef viraj yonunde kadrajdan kayinca tespit dusuyor
  ve daha cok `tut` karesi uretiliyordu (pozitif geri besleme).
* `suz` -- UZUN kayip: komut, OLCULEN hiza sonumleniyor ("coast").
  **Sifir DEGIL** -- sifir yazmak 18 m/s'den tam durusa FREN komutudur ve
  ESKI GOVDEYE-SABIT KAMERA doneminde su kisir donguyu kuruyordu: fren ->
  burun yukari pitch -> sabit kamera yukari bakar -> kenardaki hedef
  busbutun cikar -> kayip kalicilasir (2026-08-04,
  `mpc_20260804_160604`de olculdu).
  **[GIMBAL DALI 2026-08-05]** Bu donguyu kuran DIKEY halka koptu (fiziksel
  tilt gimbali, govde pitch'i goruntuye yansimiyor). Ama `suz` kolu yine de
  dogru: fren aracin hedeften kopmasi, yaw'in savrulmasi ve roll'un artmasi
  demek. Sifir yazmak hala yanlis.

### 3.2 Goruntu / image plane

Kadraj 1280x720, yatay FOV 66 deg, fx = fy = 985.5 px, merkez (640, 360).

| kolon | birim | anlam / isaret |
|---|---|---|
| `ex_deg` | deg | SANAL gimbalden yatay acisal hata. **+ = hedef SAGDA.** Gudumun kullandigi hata sinyali. |
| `ey_deg` | deg | Dikey acisal hata. **+ = hedef ASAGIDA** (dikkat: eksi = hedef YUKARIDA). |
| `bbox_w`, `bbox_h` | px | Ham bbox boyutlari. |
| `alan_kok` | px | `sqrt(w*h)`. Gecmis kosularla uyum icin saklanir. |
| `alan_px2` | px^2 | `w*h`. **BIRINCIL ODUL** budur (lineer alan, karekok degil; kullanici karari 2026-08-04). Buyumesi = yaklasma. |
| `kapsama_pct` | % | Yatay kapsama = `w / 1280 * 100`. Devir esigi %6 civari (ucak ~6.4 m). |
| `bbox_yas_s` | s | Son STAB tespitten beri gecen sure. `> 0.7` -> `durum` artik `taze` degil. |
| `t_capture` | s (ROS) | Karenin yakalanma ani. Yalniz FARKLARI kullan. |
| `px_sanal_x`, `px_sanal_y` | px | SANAL (stabilize) piksel. Govde salinimi matematiksel olarak temizlenmis kadraj. Kadraj disina tasabilir -- bu bir hesap sonucudur, hata degil. |
| `px_ham_cx`, `px_ham_cy` | px | **HAM** bbox merkezi (dedektorden dogrudan). Hedefin FIZIKSEL olarak kadrajda nerede oldugu. `cy < 360` = hedef kadrajin ust yarisinda. |
| `kadraj_kenar_px` | px | Ham merkezin en yakin kadraj kenarina uzakligi. **Kucuk = hedef kadrajdan cikmak uzere.** < 50 px alarm. |
| `kadraj_kenar_deg` | deg | Ayni sey aci olarak (`atan(px/985.5)`). |
| `ham_yas_s` | s | HAM kanaldaki tespitin yasi (stab kanali gimbal kapaliyken gelmez, ham hep gelir). |
| `tilt_deg` | deg | **[GIMBAL DALI]** Kameranin o karedeki DUNYA elevasyonu (`tracker_bbox_stab[7]`, `bbox_to_redis` yayinlar). Fiziksel tilt gimbali dinamik oldugu icin kamera ekseni artik BUNDAN okunur, statik `YILDIZ_TILT`'ten degil. Bos = tilt zinciri kapali (`--no-tilt`) ya da o kare bayat. Komut/gerceklesen ayrimi icin gimbal CSV'sindeki `tilt_cmd_deg`/`tilt_status_deg`e bak. |

**Sanal vs ham:** `ex/ey` sanal gimbalden gelir ve gudumun girdisidir;
`px_ham_*` hedefin gercekten kadrajda olup olmadigini soyler. Ikisi ayrisirsa
(sanal merkezde, ham kenarda) hedef fiziksel FOV'un disina cikmistir.

> **[GIMBAL DALI 2026-08-05]** Eskiden buraya "sabit kamerali kopterin klasik
> sorunu: yazilim gimbali salinimi temizler ama hedefi kadraja GERI
> GETIREMEZ" yaziyordu. Bu cumle DIKEY eksende artik gecerli degil: kamera
> kendini stabilize eden FIZIKSEL tek eksen tilt gimbalinde, yani govde pitch
> salinimi zaten fiziksel FOV'u dondurmuyor (ucusta olculdu: govde
> -35.4..+35.2 deg iken kamera dunya pitch'i max |0.65| deg). Ayrisma hala
> mumkun, ama sebebi degisti: (a) YATAY eksen (yaw) hala gimballi degil,
> (b) ROLL goruntuye yansiyor (tek eksen gimbal), (c) tilt komutu yanlissa
> ya da eklem sinirindaysa. bkz. NOTLAR_GIMBAL.md

### 3.3 Menzil (gudume izinli tek hedef buyuklugu)

| kolon | birim | anlam |
|---|---|---|
| `menzil_m` | m | `MenzilKestirici` (filterwndr IMM) kestiriminden LOS uzunlugu. **Gudumun kullandigi menzil budur.** Veri bayatsa (>2 s) bos. |

Ayrica `ref_menzil_gercek_m` vardir (asagida): olculen telemetriden dogrudan.
Ikisi 5 m'den fazla ayrisiyorsa estimator kaymistir.

### 3.4 Komut ve ortak korumalar

| kolon | birim | anlam |
|---|---|---|
| `cmd_vx`, `cmd_vy`, `cmd_vz` | m/s | GONDERILEN hiz setpointi, NED. `vz` **+ = ASAGI**. |
| `cmd_hiz_mps` | m/s | `norm(cmd_v)`. Tavan `GORUNTULU_MAX_SPEED_MPS` (**35**, 2026-08-05). MPC'nin kendi tavani da artik ayni kaynaktan geliyor (`mpc_gudum.cevre_hiz_tavani`); eskiden 18'de sabit yazili oldugu icin kelepceye HIC degilmiyordu. |
| `cmd_yaw_rate_dps` | deg/s | Gonderilen yaw hizi (slew + LPF sonrasi). Bos = yaw otopilotta. |
| `kelepce_hiz` | 0/1 | Hiz tavanina degdi (LPF cikisi 18 m/s'yi asti). |
| `kelepce_irtifa` | 0/1 | MUTLAK IRTIFA TABANI devrede: 15 m altinda ALCALMA komutu kesildi. 2026-08-04 cakilma dersinin son savunmasi. **Terminal fazda surekli 1 ise dikey kanalda hata var.** |
| `kelepce_yaw_slew` | 0/1 | yaw ivme (slew) kelepcesi kesti (120 dps^2 * dt). Surekli 1 = kontrolcu yaw'i cirpiyor. |

Komut yolu sirasi: kontrolcu -> LPF(tau=0.35, devirde tohumlu) -> hiz
kelepcesi -> irtifa tabani -> (yaw icin) slew + LPF(tau=0.15) -> MAVLink.

### 3.5 Kendi durumumuz

| kolon | birim | anlam |
|---|---|---|
| `pos_x`, `pos_y`, `pos_z` | m | LOCAL_POSITION_NED. **`z` asagi-pozitif degil: yukseklik = -z.** `pos_z = -54` -> 54 m irtifa. |
| `irtifa_m` | m | `-pos_z`, okuma kolayligi icin. |
| `vel_x`, `vel_y`, `vel_z` | m/s | NED hizi. `vel_z` + = alcaliyoruz. |
| `hiz_mps` | m/s | `norm(vel)`. |
| `acc_x_mps2`, `acc_y_mps2`, `acc_z_mps2` | m/s^2 | KENDI ivmemiz: hizin sayisal turevi (backwards O(h^2)) + LPF(tau=0.20). IMU'dan DEGIL -- 20 Hz telemetriden turetilmis, ~0.2 s gecikmeli. Manevra siddetinin gostergesi; 5 m/s^2 uzeri sert. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | deg | ATTITUDE. `pitch` + = burun yukari. |
| `rota_deg` | deg | Yer rotasi (course over ground), 0=kuzey, +dogu. Yaw ile farki = yengec acisi (crab). |
| `vibe_max` | m/s^2 | VIBRATION'un en buyuk ekseni. **Carpmanin delili DEGIL:** Gazebo iki SITL araci arasinda TEMAS MODELLEMIYOR (0.92 m'lik geciste vibe 0.9 olculdu). Vibe'in isi YER temasini ayirt etmek: orada 150-345'e firlar. |

### 3.6 `ref_*` -- HEDEF DURUMU (yalniz analiz)

| kolon | birim | anlam |
|---|---|---|
| `ref_hedef_x/y/z` | m | Hedefin OLCULEN konumu (GLOBAL_POSITION_INT -> NED). |
| `ref_hedef_vx/vy/vz` | m/s | Hedefin OLCULEN hizi (telemetri), NED. |
| `ref_hedef_hiz_mps` | m/s | `norm`. Wanderer planinda ~15-20. |
| `ref_hedef_rota_deg` | deg | Hedefin gidis yonu, 0=kuzey, +dogu. |
| `ref_hedef_ax/ay/az_mps2` | m/s^2 | IMM'in ivme kestirimi (durum vektoru 10D: `[x,y,z, vx,vy,vz, ax,ay,az, omega]`). Manevra tespiti. |
| `ref_hedef_donus_dps` | deg/s | IMM'in etkin donus hizi (CT modu). Sifirdan uzaklasmasi = hedef viraja girdi. |

### 3.7 `ref_*` -- KARSILASMA GEOMETRISI (yalniz analiz)

**Bu blok, "ne oldu"yu anlatan asil bilgidir.**

| kolon | birim | anlam / isaret |
|---|---|---|
| `ref_menzil_gercek_m` | m | Olculen konumlardan mesafe. `menzil_m` ile karsilastir: estimator kaymasi. |
| `ref_kerteriz_deg` | deg | Bizden hedefe pusula kerterizi, 0=kuzey, +dogu, [0,360). |
| `ref_yukselis_deg` | deg | Hedefin bize gore yukselis acisi. **+ = hedef YUKARIDA.** Kamera ekseniyle karsilastirilir; fark +-20.07 deg'i (dikey yari-FOV) asarsa hedef fiziksel olarak kadraj disindadir. **[GIMBAL DALI 2026-08-05]** Kamera ekseni artik `montaj + pitch` DEGIL, gimbalin dunya elevasyonu: `tilt` (komut `YILDIZ_TILT` = atan(down/back), gercek deger gimbal log'undaki `tilt_status_deg`). Govde pitch'i esitlige GIRMEZ (gimbal telafi ediyor; olculen artik pitch etkisi max 0.65 deg). Eski `montaj + pitch` formulu yalniz `--no-tilt` (dondurulmus govdeye-sabit) yolunda gecerlidir. |
| `ref_yaklasim_acisi_deg` | deg | **KARSILASMA ACISI.** Hedefin GIDIS YONU ile "hedeften bize" vektoru arasindaki aci. `0` = hedef tam uzerimize geliyor; `180` = hedef bizden uzaklasiyor, biz arkasindayiz. |
| `ref_karsilasma_tipi` | - | `kafa_kafaya` (<60 deg) \| `capraz` (60-120) \| `kuyruk` (>120) \| `durgun` (hedef hizi <1 m/s). |
| `ref_kapanma_hizi_mps` | m/s | Menzilin kapanma hizi. **+ = mesafe KISALIYOR.** |
| `ref_tgo_s` | s | `menzil / kapanma` (yalniz kapaniyorken). |
| `ref_cpa_m` | m | En yakin gecis mesafesi, SABIT HIZ varsayimiyla. |
| `ref_cpa_s` | s | Ona kalan sure. `0` yazilmissa gecis GECMISTE kaldi (kelepcelenir, `ref_cpa_m` o an menzile esitlenir). |

**Isaret hatirlaticisi (en cok karistirilan):**
```
ref_yaklasim_acisi_deg =  0  -> KAFA KAFAYA (hedef uzerimize geliyor)
                        = 90  -> CAPRAZ (yandan geciyor)
                        = 180 -> KUYRUK TAKIBI (biz onun arkasindayiz)
ey_deg  > 0 -> hedef kadrajin ALTINDA
ref_yukselis_deg > 0 -> hedef bizden YUKARIDA
pos_z   < 0 -> havadayiz (irtifa = -pos_z)
cmd_vz  > 0 -> ALCALMA komutu
```

**Gecis anindaki tuzak:** `ref_yaklasim_acisi_deg` en yakin gecis aninda
0'dan 180'e SAVRULUR (yanindan gecerken 90'dan gecer). "Bu karsilasma neydi"
sorusunun cevabi gecis anindan degil **gecisten ~2 s ONCEKI** satirdan
okunur. `kosu_anlat.py` bunu boyle yapar.

### 3.8 `olay`

O dongude ureyen olay adlari, birden fazlaysa `|` ile ayrilmis. Cogu satirda
bostur. Detayli hali `_olay.csv` dosyasindadir (bkz. 4).

### 3.9 SISTEM SAGLIGI (2026-08-07, gercek ucus icin eklendi)

Bes kolon da **LOG-ONLY**: hicbiri gudum kararina, komuta ya da zamanlamaya
girmez. Kaza sonrasi ilk sorulan uc soruyu cevaplarlar: *dongu gercek
zamanda miydi, surec hayatta miydi, otopilot bizi dinliyor muydu.*

| kolon | birim | anlam |
|---|---|---|
| `dongu_hz_ort` | Hz | Son ~2 s'lik pencerede `1/dt` ortalamasi (pencere = 2 s / nominal adim; 20 Hz'de 40 ornek). Kelepcelenmemis HAM adimdan hesaplanir, yani `dt` kolonunun 0.5 s tavani BURAYI kirpmaz. `loop_hz`'in belirgin altina inmesi dongunun boguldugu demektir -- bu, dev daireler ve titremenin bilinen kok nedenidir. |
| `dt_asim` | 0/1 | `ham_dt > 1.5 x nominal adim` ise 1. Tek tek 1'ler normaldir (GC, disk); **ORANI** bak. 10 ARDISIK 1 gorulunce stderr'e `*** DONGU YAVAS ***` uyarisi basilir (5 s'de bir, spam yok). |
| `hayatta_ttl` | s | Redis `goruntulu_hayatta` olu-adam anahtarinin KALAN TTL'i. Saglikli kosuda hep `2` (= `HAYATTA_TTL_S`). `1` gorunuyorsa dongu ~1 s'den uzun takiliyor; `-1` anahtar hic yok / Redis cevapsiz -- o durumda karar verici yetkiyi goruntuluye DEVRETMEZ ya da geri alir (2026-08-05 arizasi: yetki devredilmis ama kontrolcu hic kosmuyordu). 10 dongude bir ORNEKLENIR (Redis gidis-donusu pahali), arada son deger tekrarlanir. |
| `ap_mod` | - | Otopilotun HEARTBEAT'inden mod adi (`GUIDED`, `LOITER`, `RTL`, `LAND`, `STABILIZE`...). **`GUIDED` DISI bir deger gorulen her satirda gudum kolonlarinin tamami yaniltir**: komut gonderiliyordur ama araca gecmiyordur. Bos = otopilottan hic heartbeat gelmedi. |
| `hb_yas_s` | s | Son otopilot HEARTBEAT'inin uzerinden gecen sure = **MAVLink baglanti sagligi**. Otopilot 1 Hz basar; tipik 0-1 s. Buyuyorsa o satirdaki `pos_*`/`vel_*`/`roll_deg`... degerlerinin HEPSI donmus onbellek verisidir (okuyucu son bilineni dondurur). Bos = hic heartbeat yok. |

**Neden sadece bu bes kolon:** `yetki` kolonu her satirda `goruntulu`
oldugu icin bilgi tasimaz (dongu yalniz yetkiliyken yazar). Gercek
"calisiyor muyum" sorusunun cevabi `hayatta_ttl` + `hb_yas_s` ikilisidir.

---

## 4. `goruntulu_<metot>_<damga>_olay.csv` -- OLAY LOGU

Kolonlar: `t` (monotonic), `t_unix`, `olay`, `menzil_m`, `detay`.
Kenar (edge) tetiklidir: bir kisit 100 dongu acik kalirsa bir kez `_acik`,
bir kez `_kapali` olayi yazilir.

| olay | ne demek |
|---|---|
| `devir_alindi` | Yetki goruntuluye gecti. `detay`: tohum hizi + `devir_durumu` var/yok + `kural=` (devri tetikleyen kapi: `basit(5 ardisik kare)` ya da `eski(38/45 kare, alan %2, menzil kapisi 60 m)`; kaynak Redis `gecis_sebep`). **`devir_durumu=YOK` ise gecis sicramali olabilir** (LPF konumlunun son komutuyla degil, kendi olculen hizimizla tohumlandi). |
| `yetki_konumluya_dondu` | Karar verici `konumlu`ya dondu; komut kesildi. |
| `tespit_taze_to_tut` | bbox bayatladi, son komut tutuluyor. |
| `tespit_tut_to_suz` | 1.7 s'yi asti, suzulmeye (coast) gecildi. |
| `tespit_suz_to_taze` | Hedef geri geldi. |
| `kelepce_hiz_acik/kapali` | 18 m/s tavani. |
| `kelepce_irtifa_acik/kapali` | 15 m irtifa tabani alcalmayi kesti. |
| `kelepce_yaw_slew_acik/kapali` | yaw ivme kelepcesi. |
| `menzil_100m` ... `menzil_3m` | Esigin ILK gecilmesi. `detay`: o andaki karsilasma tipi + kapanma. |
| `en_yakin_gecis` | Kapanma hizi isaret degistirdi (kapaniyordu -> aciliyor) ve menzil < 200 m. `detay`: tip + yaklasim acisi (**gecis anindaki** deger; bkz. 3.7 tuzagi). |
| `iska` | En yakin gecisten sonra menzil iki katina (ya da +15 m) cikti. |
| `ap_mod_degisti` | **OTOPILOT UCUS MODU DEGISTI** (2026-08-07). `detay`: `<eski> -> <yeni>`. Ilk gorulen mod da yazilir (`- -> GUIDED`), yani devir aninda aracin hangi modda oldugu sabitlenir. **`GUIDED -> LAND/RTL/STABILIZE` satiri kaza zaman cizelgesinin baslangicidir**: o andan sonra gudumun gonderdigi hicbir setpoint araca gecmemistir. Ayni bilgi ana CSV'de `ap_mod` kolonunda satir satir durur. |
| `vurus_basarili` | **FIZIKSEL TEMAS** (2026-08-05, tur-4). Kontrolcunun KENDI vibrasyonundan tespit: `vibe > 15` VE olculen menzil `< 3 m`. `detay`: vibe degeri, menzil, faz ve vurus karisimi. Angajman basina BIR KEZ (latch). **Kosu ozetinde vurus sayisini bu satirlari sayarak al** -- her basarili vurus koşuyu bitirdigi icin (arac takla atip duser) CPA tahmini yaniltici ve orneklem kucuk kaliyordu. |

---

## 4.5 `gimbal_<damga>.csv` -- GIMBAL ZINCIRI

`bbox_to_redis.py` yazar, **kare basina bir satir** (~30 Hz, yalniz gecerli
tespitlerde). `t_kare` `time.monotonic()`, yani diger CSV'lerle DOGRUDAN yan
yana konur. 2026-08-07'den once bu dosya HIC uretilmiyordu (bayrak
verilmiyordu); artik varsayilan acik.

| kolon | birim | anlam |
|---|---|---|
| `t_kare` | s | Karenin islenme ani (monotonic). Hizalama anahtari. |
| `bbox_cx`, `bbox_cy`, `bbox_w`, `bbox_h` | px | HAM bbox merkezi ve boyutu. |
| `roll_deg`, `pitch_deg` | deg | Kare anina INTERPOLE edilmis govde tutumu. |
| `menzil_m` | m | bbox genisliginden kestirilen menzil (yalniz aim sonumlemesi icin; gudum menzili telemetriden alir). |
| `ham_ex_deg`, `ham_ey_deg` | deg | Yazilim de-rotasyonu HIC olmasaydi gorulecek acisal hata. |
| `stab_ex_deg`, `stab_ey_deg` | deg | Gudume giden hata (goruntulu CSV'deki `ex_deg`/`ey_deg` ile ayni). |
| `aim_deg`, `aim_etkin_deg` | deg | Aim trim durumu (tilt modunda kullanilmaz). |
| `ros_stamp` | s (ROS) | `t_capture` ile ayni saat. |
| `interp_bosluk_ms` | ms | Tutum interpolasyonunda kullanilan ornek araligi. Buyumesi = tutum akisi seyreldi, de-rotasyon guvenilmez. |
| `gecikme_ms` | ms | Kamera boru hatti gecikmesi varsayimi (`--kamera-gecikme-ms`). |
| `tilt_cmd_deg` | deg | Gimbale GONDERILEN tilt hedefi. |
| `tilt_status_deg` | deg | Gimbalden OKUNAN gerceklesen tilt. |
| `tilt_yas_ms` | ms | Tilt durum okumasinin yasi. Buyuyorsa gimbal telemetrisi kesilmis. |
| `eklem_deg` | deg | De-rotasyon zincirine giren canli eklem acisi. |

**Ilk bakilacak yer:** `tilt_cmd_deg` vs `tilt_status_deg`. Ayrisiyorsa
gimbal komutu takip edemiyor ya da eklem sinirindadir; ikisi de dogruyken
`ey_deg` hala kaciyorsa hata standoff geometrisindedir (bkz. 8).

---

## 5. `mpc_tani_<damga>.csv` -- MPC IC TANILARI

`t`, `dt` goruntulu CSV ile AYNI monotonic saattedir; satir satir yan yana
konabilir. Onemli kolonlar:

| kolon | anlam |
|---|---|
| `t_unix` | `time.time()`, mutlak epoch (EN SONDA, indis-koruma geregi). |
| `bbox_yas`, `ex`, `ey` | goruntulu CSV'deki karsiliklariyla ayni. |
| `eps` | Standoff dikey hata artigi. |
| `beta`, `beta_sinir` | FOV kisitinin degeri ve siniri; `beta` sinira dayaniyorsa kisit AKTIF. |
| `derinlik` | Kestirilen derinlik (bbox'tan). |
| `fov_serbest` | 1 = FOV kisiti gevsek, 0 = baglayici. |
| `bos_sayac` | Ust uste kac dongudur tespit yok. |
| `r_ic`, `r_olcum` | Ic model menzili vs olculen menzil; ayrisma = model kaymasi. |
| `alan`, `alan_hizi` | bbox alani ve buyume hizi (odul sinyali). |
| `d_ex`, `d_ey` | Bozucu kestirimi (hedef hareketinin GORUNTUDEN gorulen artigi). Hedef hizi telemetriden TURETILMEZ; bu, gorsel artiktir. |
| `u1`, `u2`, `u3`, `yaw_dps` | MPC'nin ic komutu (iskeletin LPF'sinden ONCE). goruntulu CSV'deki `cmd_*` bunun LPF+kelepce sonrasi halidir. |
| `vz_alt_cbf`, `vz_ust_cbf`, `yaw_alt_cbf`, `yaw_ust_cbf` | CBF'lerin urettigi anlik alt/ust sinirlar. `u` sinira yapisiksa o CBF baglayicidir. |
| `pitch_lpf` | Suzulmus govde pitch'i. **[GIMBAL DALI 2026-08-05]** Eskiden kamera ekseni bundan hesaplanirdi (`eksen = montaj + pitch`); fiziksel tilt gimbaliyla bu bag KOPTU -- kamera ekseni artik `tilt` (bkz. `tilt_status_deg`), pitch ise yalnizca aracin tutumunu anlatan bir teshis kolonudur. |
| `sure_ms`, `iter` | Cozucu suresi ve iterasyon. **Butce: p95 <= 12.7 ms, tavan 13 ms** (20 Hz dongu). `sure_ms = 0` ve `iter = 0` ise cozucu O DONGUDE HIC KOSMADI: ya ISKA (frenli suzulme) ya da VURUS kor suzulmesi. |
| `butce_kesti` | **0/1, EN SONDA** (2026-08-07). 1 = FISTA durma olcutune ULASAMADAN cikti: ya iterasyon tavanina (`iterasyon_tavani`) ya da sure butcesine (`sure_butcesi_ms`) carpti; yani o dongude **optimal olmayan bir komut verildi**. `iter` ve `sure_ms` bunu tek basina soylemez -- tavana degen cozum de yakinsayan cozum de ayni sayilari basabilir. **Raspberry Pi 5'te izlenecek kolon budur:** CPU sikisinca cozucu SESSIZCE bozulur, tek gorunur imzasi bu bayragin oranidir. Saglikli kosuda cogunlukla 0 (soguk baslangictaki ilk birkac cozum haric). Cozucunun hic kosmadigi satirlarda (ISKA/VURUS kor suzulmesi) 0. |
| `maliyet` | QP amac degeri. |
| `durum` | Faz: `KAPANMA` \| `TERMINAL` (<=45 m) \| `VURUS` (<=22 m) \| `ISKA`. TERMINAL yalniz bir etikettir; **VURUS ve ISKA kontrol yasasini DEGISTIRIR**. |
| `en_iyi_menzil`, `menzil_hizi` | Bu angajmanda ulasilan en kucuk `r` [m]; `d(r_ic)/dt` suzulmus [m/s] (**negatif = kapaniyoruz**). Terminal gecis, `menzil_hizi` isaret degistirdigi andir. |
| `vurus` | VURUS karisim katsayisi [0..1] (2026-08-05). `durum=VURUS` iken menzille dogrusal: 22 m'de 0.00, 15 m'de 0.50, 8 m ve altinda 1.00. Ne yapar: FOV bantlarini fiziksel kenara acar (14/17.5 -> 19/19), fren/hizlanma bant daraltmasini sonumler, `q_ex`/`q_ey`'yi 3x, alan odulunu 2x buyutur, `q_ivme`'yi 0.35x kisar. **Faz LATCH'li, karisim degil**: iskalanip menzil acilirsa `vurus` 0'a doner ama `durum` VURUS kalir. |
| `ivme_carp` | `q_ivme`'ye (ivme = YATMA ACISI cezası) o döngüde uygulanan çarpan (2026-08-05). VURUŞ çarpanı; varsayılan 1.0 (VURUŞ ivme cezasına dokunmuyor — tur-1'de 0.35 idi, kaldırıldı). **Not:** devir ivme rampası tur-2 sim'inde denenip GERİ ALINDI (kapanmayı yarıya düşürdü), bu yüzden bu kolon artık ~hep 1.0. |
| `vuruldu` | **FİZİKSEL TEMAS latch'i** (0/1), 2026-08-05 tur-4. `vibe > 15` VE **ölçülen** menzil `< 3 m` görülünce 1'e geçer ve angajman boyunca kalır. **Koşu özetinde vuruş SAYISI = bu kolonun 0→1 geçiş sayısıdır** — CPA tahminine gerek yok. Eşikler tur-3'ten ölçüldü: gerçek temas vibe 17.4-25.5, temassız yakın geçiş (tur-2, 0.85 m) yalnızca 3.3. Aynı anda `_olay.csv`'ye `vurus_basarili` satırı ve `goruntulu.log`'a `OLAY:` satırı yazılır. |
| `vibe` | KENDİ vibrasyonumuzun en büyük ekseni (VIBRATION mesajı). **Bizim telemetrimiz, hedefin değil** — "hedeften yalnız menzil" kuralı ihlal edilmiyor. `> 150` ve irtifa ~0 ise YER teması (bkz. §8). |
| `hiza_ref` | VURUŞ terminal dikey hizalama biası [deg/s] (2026-08-05, tur-3). `eps` (hedef görünen yükselişi) RAHAT bandını (10°) aşınca **pozitif** olur = "hatta tırman, standoff'u erit" komutu; kamera düzleşir, hedef üst kenardan çıkmaz. `0` = deadzone içinde (eps≤10°), VURUŞ dışı, ya da hedef eksenin altında (tek yanlı — alçalma zorlanmaz). Üst-kenar kaybının geometrik kökünü (standoff `down` → eps=asin(down/r) FOV'u aşar) saf güdümle kapatır. **Sim'de bu kolonu izle**: terminalde 0 kalıyorsa mekanizma tetiklenmiyor (avcı zaten hizada) demektir. |
| `tau_eff`, `u_doyum` | **DOYUMLU EYLEYICI TANISI (2026-08-08, EN SONDA).** `tau_eff` = ILK adimin YATAY etkin zaman sabiti [s] = `max(tau_lin, |e_yatay|/a_max)`. **BOS = kol KAPALI** (eski sabit `hiz_gecikme_tau_s`=1.00 s yolu). `tau_lin`'e (1.7) yapismissa dogrusal bolgedeyiz; buyudukce plan DOYUMDA demektir. Olculen (G/Gb kosulari): donus p50 3.1-3.5 / p90 9.1-11.4 s, duz p50 2.9 / p90 8.7-11.4 s -- ucus olcumuyle (tau_etkin 5.5-6.8 s) ayni mertebede. `u_doyum` = `|u_yatay|` hiz tavaninin %99'una degdi mi (0/1). **Kol ACIKKEN bu oran DUSER** (olculdu: donus %40.8 -> %14.9-27.6, duz %36.4 -> %5.9-11.0) -- ulasilamaz komut sicramalarinin azaldiginin dogrudan olcusu. Mekanizma: `mpc_gudum.cevre_eyleyici()`. |
| `durgunluk` | **ILERLEME SAATI TANISI (2026-08-08, EN SONDA).** ISKA zaman asimini atesleyen **durgunluk saati** [s]. **BOS = kol KAPALI** -- o zaman karar eski DUVAR saatinden (`t - yetki_t0`) verilir ve saat hic hesaplanmaz (kapali kolda tek mikrosaniye harcanmaz). Kol acikken: ilerliyorsak saat `1 - ilerleme_kazanci` = **-0.5 s/s ile GERI sarar**, durgunsak +1 s/s ilerler; `iska_zaman_asimi_s`'i (8) asinca ISKA (`sebep` dizesi `"durgunluk (... ; angajman ...)"` olur). ILERLEME OLCUSU = **en iyi menzilin (best-so-far) iyilesme hizi** > `ilerleme_kapanma_esigi_mps` (1.0 m/s); best-so-far MONOTON oldugu icin olcu salinima yapisi geregi bagisiktir (anlik ve pencereli HAM menzil surumleri sinusle kandirildi -- bkz. `cevre_ilerleme_saat`). Mutlak tavan `ilerleme_tavan_s` (22 s) sonsuz donguyu keser; sebep `"ilerleme tavani"`. **"menzil aciliyor" ve "mutlak menzil" kapilarina DOKUNULMADI.** |
| `v_dik`, `a_dik`, `apn_a` | **APN TANISI (2026-08-08, EN SONDA).** `v_dik` = kestirilen hedef dik hizi [m/s] (`d_ex * r / KDEG`) -- **kol KAPALIYKEN de yazilir**, cunku APN'in ne kazandiracagini olcmek icin taban kosusunda da gerekli. `a_dik` = `v_dik`'in suzulmus turevi [m/s^2], **HAM kestirim** (olu bant ve guven carpani UYGULANMAMIS; `apn_tau_s`=0.6 LPF, `+-6` kelepce, `r<15 m`'de d_ex ile birlikte DONAR). `apn_a` = yasada **gercekten kullanilan** deger: olu bant (`apn_olu_bant_mps2`=0.5, cikarmali) + bozucu guven rampasi + `YILDIZ_APN` carpani uygulanmis hali. **Kol kapaliysa `apn_a` her zaman 0.** Okuma kurali: `a_dik` buyuk ama `apn_a` 0 ise katkiyi gurultu degil OLU BANT kesmistir; ikisi de ~0 ise hedef gercekten duz uculuyordur. Donus/duz faz ayrimini `ref_hedef_donus_dps` ile yapip bu ucluyu p50/p90 olarak oku. Mekanizma: `mpc_gudum.cevre_apn()`. |
| `bant_ust` | O anki UST kadraj bandi [deg]. Nominal 17.5; VURUS'ta 19.0'a acilir; ILERI IVMELENME planlaninca daralir. **[GIMBAL DALI 2026-08-05]** Bu daraltmanin ESKI gerekcesi "burun asagi -> govdeye sabit kamera asagi bakar -> hedef ust kenara" idi; fiziksel tilt gimbaliyle bu zincir KOPTU (govde pitch'i kadraji dondurmuyor). Daraltma hala zararsiz bir emniyet payi ama artik pitch'e karsi degil, ivmelenmedeki roll/yaw savrulmasina ve tilt takip gecikmesine karsi; gimbal dalinda YENIDEN olculmeli. Tabani `fov_ust_taban_bant_deg` = 6.0. `beta` bu banda **NEGATIF** tarafta yaklasir: `beta < -bant_ust` ust kenardan kayip demektir. |

---

## 6. `guided_follow_<damga>.csv` -- KONUMLU GUDUM

`wall_time` **monotonic'tir** (adi yaniltici; `time.monotonic()`).

| kolon | anlam |
|---|---|
| `range_m` / `true_range_m` | Kestirilen / gercek hedef mesafesi. |
| `closing_velocity`, `t_go_s` | Kapanma hizi ve kalan sure. |
| `pursuer_x/y/z`, `pursuer_vx/vy/vz` | Kendi durumumuz (NED). |
| `meas_x/y/z` | Hedefin OLCULEN konumu. |
| `est_x/y/z` | IMM'in kestirdigi konum. `meas` ile ayrisma = filtre gecikmesi/kaymasi. |
| `slot_x/y/z`, `slot_vx/vy/vz` | Standoff slotu (hedefin arkasinda tutulan nokta) ve hizi. |
| `aff_*`, `aff_mag` | Ileri besleme ivmesi. |
| `recovery_state` | `CHASE` / kurtarma makinesinin durumu. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | Tutum. |
| `cpa_range_m` | Kestirilen en yakin gecis. |
| `kill_mode`, `failsafe_active`, `hit_count` | Mod ve failsafe bayraklari. |
| `loop_dt_meas_s`, `simtime_ratio` | **OLCULEN** dongu adimi ve sim/gercek zaman orani. `simtime_ratio` basari ongorusudur: 1.0'dan uzaklasmasi (ozellikle <0.8) dongunun bogulmasi demektir ve dev daireler/titreme onun kokudur. |
| `yaw_frozen`, `z_limited`, `aim_limited`, `alt_floored`, `turn_clamped`, `carrot_limited` | Hangi korumanin devrede oldugu. |

Hedefin HIZI bu logda YOKTUR. `kosu_anlat.py` gerektiginde `meas_x/y/z`'yi
~0.5 s'lik merkezi farkla turevleyerek turetir.

---

## 7. `bbox.log` -- DEDEKTOR

Zaman damgasi YOK. Iki satir tipi:

```
HEDEF merkez=(669,232) bbox=(665,229,9,7) cov=0.70%
[OZET] kare=1485 fps=30.0 tespit_orani=%59.5 mod=konumlu | gimbal: sanal=(646,79) hata=(+0.33,-15.94) deg tutum=(-0.3,-9.1) aim=-0.22
```

* `merkez` HAM piksel merkezidir (kadraj 1280x720, orta 640/360).
  **`y < 360` = hedef kadrajin UST yarisinda.**
* `bbox=(x, y, w, h)` -- `x,y` SOL UST kosedir.
* `cov` yatay kapsama (%); devir esigi ~%6.
* `[OZET]`da `hata=(ex, ey)` sanal gimbalin acisal hatasidir (deg),
  `tutum=(roll, pitch)`, `aim` dikey aim trim'i.

Zamana baglamak icin yeni goruntulu CSV'nin `px_ham_cx/cy` kolonlarini
kullan; ayni merkezler orada zaman damgasiyla durur.

---

## 8. "SU DEGERI GORURSEN SU ANLAMA GELIR"

| gozlem | yorum |
|---|---|
| `dt` ortancasi 0.05'ten buyuk / `1/dt` 20 Hz'in altinda | Dongu boguluyor. Sabit dt varsayan her kontrolcu dev daireler ve titreme uretir (2026-08-04 kok neden). `simtime_ratio` ile birlikte bak. Artik dogrudan kolon var: `dongu_hz_ort` ve `dt_asim` (bkz. 3.9). |
| `dt_asim` orani > %10 ya da `dongu_hz_ort` nominalin altinda | Gercek zaman kaybediliyor. `goruntulu.log`'un **stderr** kismindaki `*** DONGU YAVAS ***` satirlariyla dogrula; CPU yukunu azalt (video kaydi, GUI). |
| `hayatta_ttl` = -1 ya da 1'e dusuyor | Olu-adam anahtari tazelenemiyor: ya Redis cevapsiz ya dongu >1 s takiliyor. Karar verici bu durumda yetkiyi goruntuluye VERMEZ / geri alir -- ardindan gelen "komut kesildi" satirlari SEBEP degil SONUCTUR. |
| `ap_mod` `GUIDED` DEGIL | **Once buna bak.** Arac bizim setpointlerimizi dinlemiyordu; gudum kolonlarinin tamami yaniltir. `_olay.csv`'deki `ap_mod_degisti` satiri gecis anini verir (failsafe / RC ile mod degisimi / GCS mudahalesi). |
| `hb_yas_s` 2-3 s'in uzerine cikiyor | MAVLink baglantisi kopuk. O satirlardaki `pos_*`, `vel_*`, `roll/pitch/yaw` DONMUS onbellek degerleridir -- "arac hareketsiz duruyordu" diye okuma. |
| `butce_kesti` (mpc_tani) oraninin yuksek olmasi | Cozucu yakinsamadan cikiyor: Pi CPU'su yetmiyor ya da problem kotu kosullanmis. Komutlar optimal degil. `sure_ms` ve `dongu_hz_ort` ile birlikte oku. |
| `durum` uzun sure `suz` | Hedef kadraj disi. Komut olculen hiza sonumleniyor; arac duz uzuyor. Karar verici dwell'i dolunca konumluya donmeli. |
| `bbox_yas_s` surekli 0.5-0.7 bandinda | Tespit oraninda sinirdayiz; dedektor kareyi zor buluyor (kucuk bbox / uzak menzil). |
| `kadraj_kenar_px` < 50 ve dusuyor | Hedef kadrajdan cikmak uzere. Sonrasinda gelen `tespit_taze_to_tut` bunun sonucudur. |
| `px_sanal_y` kadraj disinda ama `px_ham_cy` icinde | Normal: sanal kadraj bir hesaptir, tasabilir. |
| `ey_deg` surekli buyuk negatif (hedef yukarida) | Kamera hedefin altini gosteriyor. **[GIMBAL DALI 2026-08-05]** Suclu artik `pitch_deg` DEGIL (gimbal onu telafi ediyor): TILT hatasi. Once `tilt_cmd_deg` vs `tilt_status_deg`'e bak -- ayrisiyorsa gimbal takip edemiyor/eklem sinirinda; ikisi de dogruysa standoff geometrisi hatalidir (`YILDIZ_TILT` != atan(down/back), bkz. `scripts/standoff_geom.sh`). |
| `kelepce_irtifa` terminal fazda surekli 1 | Kontrolcu sureklI alcalma komutluyor; taban olmasa yere carpardi. Dikey kanalda hata var. |
| `kelepce_yaw_slew` yuksek oranda 1 | Kontrolcu yaw'i cirpiyor (MPC'de sert FOV kisiti acikken 4 Hz'de +-16 dps olculdu). |
| `ref_kapanma_hizi_mps` isaret degistirdi | En yakin gecis oldu. Sonrasi `iska`dir. |
| `ref_karsilasma_tipi = kafa_kafaya` ve menzil kucuk | Yaklasma cok hizli, t_go kisa, kontrolcunun tepki penceresi dar. "MPC neden kaciyor gibi" sorusunun cevabi genelde burada. |
| `menzil_m` ile `ref_menzil_gercek_m` 5 m'den fazla ayristi | Estimator kaymis; gudum yanlis menzille calisiyor. |
| `vibe_max` > 50 **ve** `irtifa_m` ~0 | YER TEMASI (carpma DEGIL). |
| `vibe_max` dusuk ama menzil < 1 m | Muhtemelen GERCEK carpma: Gazebo iki SITL araci arasinda temas modellemiyor, vibe sicramaz. |
| `alan_px2` platoya oturmus, menzil sabit | Takip var ama kapanma yok -- standoff'ta asili kaldik. |
| `cmd_hiz_mps` hic 33 m/s'i gecmiyor (tavan 35) | 35 m/s turunun ana sorusu. Ya geometri (dik LOS -> dikey tavan bagliyor, bkz. `cmd_vz`), ya `q_ivme` (yatma cezasi) ya da sert FOV kisiti kapanmayi kirpiyor. `vz_alt_cbf`/`vz_ust_cbf` ve `u1` ile birlikte oku. |
| `durum=VURUS` ama `vurus` hep < 0.3 | 22 m'nin altina inildi ama 15 m'nin icine girilemedi: terminal faz baslamis, vurus tamamlanmamis. Tipik olarak ardindan ISKA gelir. |
| `sure_ms=0` satirlari VURUS'ta kumeleniyor | Kor suzulme calisiyor (bbox bayat). Kisa (< 1 s) ise TASARIMDIR; uzunsa dedektor terminal fazda hedefi tamamen kaybediyor demektir. |
| `pitch_deg` hizli salaniyor (ardisik satirlarda > 10 deg/s) | **[GIMBAL DALI 2026-08-05: bu satir ESKI GOVDEYE-SABIT KAMERA icin yazildi.]** O donemde pitch hizi = KADRAJ hiziydi: 1 deg/s ~ 17 px/s, 13 deg/s ~ 950 px/s -> gorsel titreme. Fiziksel tilt gimbaliyle bu carpim GECERSIZ (kamera dunya pitch'i max 0.65 deg olculdu); kadraj titremesinin dikey bileseni icin artik `tilt_status_deg` turevine bak. Pitch hala aracin ivmelenmesini anlatir (pitch ~ -atan(a/g), 5.84 deg per m/s^2). |
| `ey_deg` cok negatif (hedef ust kenarda) | Hedef UST kenardan cikiyor. `beta < -20.07` ise hedef FIZIKSEL olarak FOV disindadir -- bant acmak (VURUS) kurtaramaz. **[GIMBAL DALI 2026-08-05]** Eski care "ivmeyi/pitch'i kismak"ti; artik dogru care TILT: eksen `tilt_status_deg`, yeterince yukari bakmiyorsa standoff `down`/`back` ikilisi ya da tilt komut zinciri hatalidir. Asagidaki olcum ESKI govdeye-sabit, mount 0 doneminden: 2026-08-05 sim'de ust kayip %18.5 vs alt %6.0, bunun %10.9'u fiziksel FOV disiydi -- gimbal dalinda YENIDEN olculmeli. |

---

## 9. ARACLAR

| arac | ne yapar |
|---|---|
| `tools/kosu_anlat.py <deneme>` | **Once bunu calistir.** Loglari hizalar, insan dilinde zaman cizelgesi ve karsilasma tipi dagilimi cikarir. |
| `tools/karsilastir.py` | LOS/PID/MPC kosularini ayni odul tanimiyla yan yana koyar. |
| `tools/deneme_ozeti.py <deneme>` | Konumlu fazin + kameranin ozeti, kadraj geometrisi analizi. |

## 10. KOLON EKLERKEN

1. `goruntulu_temel.LOG_KOLONLARI` listesine **SONA** ekle.
2. `calistir()` icindeki `satir = [...]` listesine ayni sirada ekle. Uyusmazsa
   dongu ilk satirda "LOG UYARISI: kolonlar KAYMIS olabilir" basar.
3. Bu dosyaya birimi, isareti ve yorumuyla ekle.
4. Hedeften turetiliyorsa adi **`ref_` ile baslamali** ve kontrol yolu onu
   OKUMAMALI.
5. Ek maliyeti olc: 20 Hz dongude satir basi butce ~100 us mertebesindedir
   (olculdu: 69 kolon + geometri = 110 us/satir, dongunun %0.2'si).
6. Kolon bir DIS kaynaktan (Redis, MAVLink) her dongu ayri gidis-donusle
   geliyorsa **ORNEKLE**: 10 dongude bir oku, arada son degeri yaz
   (`GoruntuluDongu._hayatta_ttl` deseni). Kolon her satirda dolu kalir,
   dongu butcesi bozulmaz.
7. Yeni kolonlarin okundugu yer kontrol yolunun ALTINDA olsun -- komut
   gonderildikten sonraki "LOG" bloğunda. Boylece kolonun gudume
   sizmadigi kodun SIRASINDAN okunabilir.

## 11. DISKE YAZMA GARANTISI (2026-08-07)

Cakilmada acik dosya tamponu KAYBOLUR. Sureclerin hepsi artik periyodik
flush yapar; bunu bozma:

| dosya | flush |
|---|---|
| `goruntulu_*.csv` | 20 satirda bir (~1 Hz @ 20 Hz dongu) |
| `goruntulu_*_olay.csv` | HER olayda (olaylar seyrek) |
| `mpc_tani_*.csv` | 20 satirda bir |
| `gimbal_*.csv` | 20 satirda bir (~1.5 Hz @ 30 fps) |
| tutum CSV (`--tutum-log`) | 20 satirda bir |

Kayip pencere boylece en fazla ~1 s'dir (eskiden: dosya yalniz DUZGUN
kapanista bosaltiliyordu, yani SIGKILL/cakilmada son ~2 s hic yazilmiyordu
-- tam da analiz edilmek istenen kisim).
