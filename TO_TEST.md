# TO_TEST — MPC vuruş yasası: kök-neden temelli deney sırası (TUR-5+)

Oturum: 2026-08-05. Üç paralel analiz (koşu verisi adli dökümü + kod yapısal
taraması + literatür) + orkestratörün kendi çözücü/video doğrulaması sentezi.
Bağlam: önce `DEVAM.md` (tur-1..4 kampanyası) ve `guidance_allstar/LOG_SOZLUGU.md`.

> **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05]** Bu dosyanın tamamı **gövdeye-sabit
> kamera** varsayımıyla yazıldı. `gimbal` dalında sim'deki 5 drone'un kamerası
> kendini stabilize eden **fiziksel tek eksen (tilt) gimbalde**: uçuşta ölçüldü,
> gövde pitch'i −35.4…+35.2° savrulurken kamera dünya pitch'i **max |0.65°|**.
> Sonuç: **gövde pitch'i görüntüye yansımıyor**, **roll yansıyor** (tek eksen),
> **yaw'da gimbal yok**. Dikey ekseni artık `YILDIZ_TILT = atan(down/back)`
> belirliyor (`scripts/standoff_geom.sh`), yani **komut edilebilir**.
> Pitch'e dayanan her madde/ölçüm bu ışıkta yeniden okunmalı; madde 5 ve 6
> aşağıda ayrıca işaretlendi. Ayrıntı: `NOTLAR_GIMBAL.md`.

## ★ CANLI T3 HÜKMÜ — 2026-08-11, büyük-bbox kapılı tek LOS

T2'deki saf LOS ağır kuyruğunun kökü erken/uzak devirdi. Dört ağır ıskanın
başlangıç bbox ölçeği yalnız `%2.03–2.94` idi. Yeni kapı merkez ve telemetri
menzili istemeden art arda 5 karede alan ölçeğini `%3` üstünde arıyor
(`YILDIZ_GECIS_BUYUK_KARE=5`, `YILDIZ_GECIS_ALAN_PCT=3`). Aynı kural gerçek
donanım düğümünde `--buyuk-kare 5 --alan-pct 3` olarak uygulandı.

| kol | n | gerçek CPA medyan / p90 | `<5 m` | temas | ağır kuyruk |
|---|---:|---:|---:|---:|---:|
| LOS N=4 | 6 | **1.09 / 2.03 m** | **6/6** | **2** | 0 |
| LOS N=5 | 12 | 1.65 / 3.76 m | 11/12 | 0 | 1 (`8.94 m`) |

N=4 CPA'ları `0.42, 0.79, 2.60, 1.45, 1.39, 0.63 m`; iki ayrı yığın
başlangıcında `vurus_basarili` oluştu ve temas aracı düşürdü. N=5 CPA'ları
`0.48, 1.64, 1.50, 1.67, 2.93, 2.71, 2.60, 0.79, 3.85, 8.94, 1.20,
0.43 m`; temas yok. N=5'in ilk otomatik süreci HOME_POSITION yarışında öldü,
ölü-adam kapısı yanlış devri engelledi ve aynı sağlıklı uçuşta süreç yeniden
bağlanarak 12 angajman toplandı. Kontrolcü yokken biriken `292` karelik sayaç
da sonradan açılan sürece kredi olmaması için sıfırlanacak şekilde düzeltildi.

**Karar:** T2 hükmü değişti; sade donanım adayı N=4 tek LOS'tur. MPC yalnız
A/B/fallback. Sonraki değişken LOS kazancı değil, gerçek donanımda kademeli
doğrulamadır: salt izleme → gimbal/işaret/gecikme → hover dry-run → düşük hız
geofence uçuşu. Telemetrisiz geçiş tamamdır; telemetrisiz terminal menzil
henüz tamam değildir. Yaw kapatma ayrı bir emniyet/FOV A/B'sidir ve şampiyon
varsayılana karıştırılmayacaktır.

## ★ CANLI T0 HÜKMÜ — 2026-08-10 18:19, çıplak varsayılanlar, hedef_elips

Aktif koşu: `goruntulu_mpc_20260810_180540.csv` /
`mpc_tani_20260810_180540.csv`. Süreç durdurulmadan alınan anlık görüntü:
~10.5 dakika, 4.2k görüntülü satır, 23 ayrı yakın geçiş. Çalışan
`mpc_gudum.py` ortamında `YILDIZ_*` override yok; P+TGO, doyumlu eyleyici,
KOR_PN ve TUT_YAW yeni sevk varsayılanlarıyla açık.

### Yeni kök neden: terminalde ulaşılmaz 180° hız terslemesi

- Sistem hedefin arkasına **yerleşebiliyor**: `R<20 m` yaklaşım açısı
  ortancası **160.2°** (180° saf kuyruk). Sorun "arkaya hiç geçememek" değil.
- Fakat `R<8 m` içinde MPC'nin yatay hız komutu ile aracın gerçek hız yönü
  arasındaki açı ortanca **171.6°** (kapanan örneklerde); komut-gerçek hız
  farkı **51.9 m/s**. Komut hedefin uçuş yönüne de **175.7° ters**.
- Aynı bantta yaw komutu ortanca **88.6°/s**, gerçek roll yalnız **5.0°**.
  Örneklerin %91.4'ünde istenen hız mevcut hıza 90°'den fazla ters.
- MPC ufku sabit **2.4 s**; `R<8 m` kapanan örneklerde gerçek `t_go`
  ortancası **1.54 s**. Maliyetin önemli bölümü öngörülen CPA/çarpışma
  sonrasında kalıyor; `rbar` menzil tabanına çivilendikten sonra çözücü
  fiziksel olarak olmayan gelecek için geri dönüş setpoint'i seçiyor.
- Bu, kullanıcının gördüğü "bir kez dalıyor/geçiyor, kalkıp ikinci denemede
  vuruyor" davranışının yatay karşılığıdır: ilk geçişte setpoint tersleniyor,
  araç 5 m/s² ivme tavanıyla bunu zamanında uygulayamıyor, sonra ikinci tura
  giriyor.

### Dönüş ve dikey bulguları

- CPA ortancası: hedefin son 3 s'de `<4°/s` döndüğü düz fazlarda **9.96 m**;
  `>=8°/s` sert dönüşte **13.36 m**. Sert dönüşlü 8 geçişin hiçbirinde
  `<5 m` yok; düz fazda bir geçiş **2.58 m**.
- Sert dönüşte MPC zaten büyük yanal hız istiyor (`|u2|` ortanca **29.1
  m/s**) ve yaw'ı büyütüyor (**67.7°/s**), fakat gerçek roll ortanca yalnız
  **9.4°**; komut-gerçek hız farkı **21.7 m/s**. `apn_a` bütün satırlarda
  **0**: hedef eğriliği ufka hiç yayılmıyor.
- Dikey "dal-sonra-kalk" gerçek: 23 CPA'nın 20'sinde önceki 5 s içinde
  gerçek `vel_z` en az iki kez işaret değiştirdi. `R=8..15 m` bandında komut
  tırmanma rayında (`cmd_vz<=-8.5`) %16.6, alçalma rayında (`>=+4`) %12.1.
  Buna rağmen CPA `|dz|` ortancası ~1.2–1.7 m; dikey artık sorun ama sert
  dönüş ıskalarının birincil nedeni değil.
- GUI koşusunda çözücü bütçe-kesme oranı ~%90. Bu bilinen GUI bedelidir;
  aşağıdaki son kararlar headless tekrar ister. Ancak 171° tersleme birçok
  terminal örnekte sistematik olduğu için yalnız çözücü jitteri sayılamaz.

### Mimari karar

**MPC tamamen atılmayacak; saf terminal vurucu görevi elinden alınacak.**
MPC orta fazda FOV/irtifa/hız kısıtları ve arka-çeyreğe yerleşme için yararlı.
Terminalde ise erişilebilir bir **PN/ZEM çarpışma-konisi referansı** üretilecek;
MPC ya bu referansın kısıtlı dağıtıcısı olacak ya da `R<12..15 m` bandında
yerini doğrudan ivme-sınırlı terminal yasaya bırakacak. Yaw kamerayı tutar;
roll doğrudan komutlanmaz, istenen ataletsel yanal ivme ArduPilot hız/ivme
katmanına verilir. Doğrudan roll/attitude kontrolü güvenlik katmanlarını
atladığı için ilk seçenek değildir.

## ★ CANLI T1 HÜKMÜ — 2026-08-10 19:18, saf LOS ve hibrit elips uçuşları

T0 hipotezi canlı A/B ile doğrulandı. Yeni kollar:
`terminal_los_gudum.py` (saf görüntülü PN/LOS) ve `hibrit_gudum.py`
(18 m dışında MPC, içinde PN/LOS). Her iki kolda da ilk komut gerçek hızdan
ivme ve yön konisiyle erişilebilir kümeye izdüşürülüyor; dikey komut ayrıca
jerk ve mutlak hız sınırından geçiyor.

| kol | geçerli angajman | gerçek CPA'lar (m) | CPA ortanca | `<5 m` | fiziksel temas |
|---|---:|---|---:|---:|---:|
| uzun canlı saf MPC tabanı | 59 | — | 11.20 | 3/59 | ölçülmedi |
| saf terminal PN/LOS | 2 | 7.03, 6.48 | 6.76 | 0/2 | 0 |
| ilk hibrit, dikey sınırdan önce | 4 | 1.79, 3.47, 2.74, 7.27 | 3.11 | 3/4 | 0 |
| güvenli hibrit (`hibrit4`) | 3 | 3.22, 3.42, 4.00 | 3.42 | 3/3 | 0 |

`hibrit4`te `R<8 m` kapanan 103 karede komut–gerçek hız açısı p50/p90
**5.35°/6.72°**, `>90°` tersleme **0/103**; hız sıçraması p50/p90
**2.76/3.15 m/s**. İzdüşürülmüş dış MPC bandında (`18<R<35`) açı p90
**8.12°**, tersleme **0/232**. T0'daki 171.6° terminal tersleme böylece
yalnız hafifletilmedi, yapısal olarak kapatıldı.

### Sınırlar ve operasyonel bulgular

- Yakınlık başarıldı ama **çarpma henüz başarılmadı**: bütün hibrit geçişlerde
  titreşim 0.6–2.0 bandında kaldı, `vurus_basarili` oluşmadı. Sonraki hakem
  yalnız tahmini menzil değil, `ref_menzil_gercek_m + vibe/temas` olacak.
- İlk hibrit kolunda `cmd_vz=-9 m/s`, irtifa 78.9 m ve görsel faz sonrası
  kurtarma çakılması görüldü. Dikey sınırla `cmd_vz=-2.5..+0.70 m/s`, irtifa
  52.1–67.9 m oldu ve çakılma kalmadı; fakat koşu sonunda üç
  `ALTITUDE ABORT` kaldı. Dikey enerji hâlâ P0 emniyet işidir.
- Saf LOS ilk geçişten sonra yetki bırakmayıp kafa kafaya ikinci saldırıya
  dönüyordu; geçiş-sonrası bırakma eklendi. Saf LOS iyi bir A/B/fallback,
  seçilen ana mimari değil.
- Taze yığında kontrolcüden önce **20 s kestirim/irtifa yerleşme süresi**
  gerekiyor (`KONTROL_BEKLE_S=20`). Ayrıca senaryo temizleme regex'i alt
  çizgili kontrolcü adlarını öldürecek şekilde düzeltildi; ilk LOS koşusunun
  ilk ~66 saniyesi çift kontrolcü nedeniyle analizden çıkarıldı.

### T1 mimari kararı

**Seçilen yön hibrit:** konumlu güdüm hedefin arka slotuna getirir; görüntülü
devirden sonra MPC yalnız orta fazda (`R>18 m`) FOV/kısıt planlayıcısıdır;
PN/LOS `R<=18 m` terminal çarpışma yasasıdır; ikisinin çıkışı aynı
ulaşılabilirlik ve dikey emniyet katmanından geçer. Saf MPC terminalden,
saf LOS ise ana adaylıktan çıkarıldı. Doğrudan roll komutu eklenmeyecek:
ulaşılabilir yanal hız/ivme isteği ArduPilot tarafından zaten roll'a
çevriliyor; yaw yalnız sabit kameranın görüşünü tutuyor.

## ★ CANLI T2 HÜKMÜ — 2026-08-11, RoboFly/elips, uzun geçiş kampanyası

Amaç telemetri menziline bağlı MPC→LOS geçişini, yalnız görüntüden verilen
bir kararla değiştirmek ve saf LOS'u yeterli örnekle yeniden ölçmekti. Mevcut
menzil-hibrit davranışı varsayılan olarak donduruldu; deneysel görsel karar
`hibrit_gudum.py --gecis-kaynagi gorsel` arkasına eklendi. HSV dedektörü
confidence üretmediği için bu turda confidence yerine taze/geçerli tespit
kullanıldı.

Görsel kapı: normalize bbox ölçeği `100*sqrt(A/(1280*720)) >= %3.4`,
`|ex| <= 6°`, `|ey| <= 15°`, kesintisiz `0.30 s`. Eşikler önce mevcut iyi
uçuşun 22 angajmanlık replay'inde seçildi; ardından 600 s kapalı döngü uçuldu.

| kol | devir | LOS'a geçen | tüm devirlerde gerçek CPA medyan / p90 | `<5 m` | LOS'a geçenlerde medyan / p90 | abort / çöküş |
|---|---:|---:|---:|---:|---:|---:|
| önceki canlı şampiyon, menzil-hibrit | 22 | 16 | 1.97 / 26.06 m | 14/22 | **1.44 / 5.24 m** | 0 / 0 |
| görsel-geçişli hibrit, 600 s | 29 | 15 | 5.09 / 30.08 m | 14/29 | 1.50 / 6.95 m | 2 / 0 |
| saf LOS, 600 s | 19 | 19 | 1.95 / 39.13 m | **15/19** | aynı | 2 / 1 |
| eş-koşul menzil-hibrit, 600 s | 16 | 9 | 12.69 / 31.50 m | 7/16 | 1.91 / 19.17 m | 2 / 1 |

Saf LOS'un dört ağır kuyruğu `13.74, 20.35, 39.13, 48.74 m`; görüntü
devralmasını 29–49 m'de alıp MPC'nin hazırlama safhasını atladığı örnekler
var. Görsel hibritte LOS'a geçen 15 angajmanın 13'ü `<5 m`; yani terminal
yasası yine iyi. Fakat alan+merkez kapısı 14/29 angajmanda oluşmadı ve bazı
geçişleri gerçek `11.37 m`ye kadar geciktirdi. “Hedef tam merkezde olsun”
şartı dönüşte MPC'nin hazırlayamadığı durumları kilitliyor.

**Karar:** şampiyon hâlâ önceki menzil-hibrit; varsayılan
`--gecis-kaynagi menzil` korunacak. MPC terk edilmeyecek; orta fazda hedefi
FOV içinde terminal geometriye hazırlıyor. Saf LOS yalnız kontrol/fallback.
Görsel geçiş kodu kalacak fakat gerçek sensörde doğru sonraki tasarım ikili
eşik değil: confidence+dwell, bbox ölçeği ve merkez hatasına ek olarak LOS
açısal hızının düşük olması; varsa onboard menzil ile füzyon/uyuşmazlık
kapısı. Telemetri menzili hatalıysa tek başına ne geçişte ne de terminal PN
kazancı/t_go hesabında kullanılmalı.

**Sonraki tek-değişken A/B:** merkez kapısını gevşetmeden önce logdan
`lambda_dot` (bbox açı hızı) kapısı tasarla. MPC'deki küçük salınım için ayrı
bir koşuda komut dead-zone/histerezis denenebilir; geçiş deneyiyle aynı
koşuda karıştırılmayacak. Öncelik daha yüksek emniyet işi, RoboFly'ın iki
kontrol kolunda da tekrarlanan irtifa çöküşü: hız/yatma/dikey komut zarfını
daraltıp yer teması sıfırlanmadan gerçek uçuşa geçme.

## YENİ ÖNCELİK SIRASI — madde 28–33

### 28. Terminal tersleme replay'i ve yeni zorunlu metrikler — P0 önkoşul

**İş:** Bu canlı CSV'yi doğal sırayla besleyen sıcak-başlatmalı replay kur.
Her döngüde şu metrikleri logla: `angle(cmd_xy, vel_xy)`,
`angle(cmd_xy, hedef_vel_xy)`, `|cmd_xy-vel_xy|`, öngörülen ilk CPA zamanı,
ufkun CPA sonrası kalan oranı, gerçek/istenen yanal ivme ve eşdeğer bank.

**Kabul:** replay `R<8 m` bandında >150° komut terslemesini en az bir
angajmanda üretmeli. Üretemezse offline hakem değildir; madde 29 doğrudan
sim A/B olur. Mevcut 23 CPA taban dondurulur.

### 29. Terminal ulaşılabilirlik + çarpışma-konisi yasası — P0

Üç kol ayrı A/B/C yapılacak; tek yamada karıştırılmayacak:

1. **CPA'da kesilen ufuk:** `T_h <= k*t_go` (`k` taraması 0.6/0.8/1.0),
   maliyet/alan ödülü yalnız CPA öncesinde; çarpışma sonrası `rbar=6 m`
   kuyruğu maliyete girmeyecek.
2. **İlk-komut ulaşılabilir kümesi:** MPC'nin iç modeli doyumlu olsa da dışarı
   52 m/s sıçrama göndermesin. `|u0-v|` ivme×zaman ve jerk ile sınırlanacak;
   hız yönünün tek karede 90° üstüne dönmesi yasaklanacak.
3. **Terminal PN/ZEM referansı:** `a_lat = N*V_c*lambda_dot` + dikey eşleniği;
   amaç görüntü merkezini kovalamak değil ZEM/LOS hızını sıfırlamak. MPC bu
   ivme referansını kısıtlar içinde takip edecek. İlk tarama `N=2.5/3/4`.

**Birincil kabul:** `R<8 m` kapanırken komut–hız açı ortancası `<45°`,
`|cmd_xy-vel_xy| p90 <15 m/s`; CPA ortancası düşer ve düz fazda mevcut
2.58 m kuyruğu kaybolmaz. **Emniyet:** FOV taze oranı, irtifa tabanı ve
kurtarma sayısı gerilemez.

### 30. Arka-koni kapısı ve iki fazlı hız programı — kullanıcının amacı

**Amaç:** "arkasına takıl ve arkadan hızlanarak çarp" doğrudan durum
makinesi olsun.

- **YERLEŞ:** `R>12..15 m` veya kuyruk açısı `<165°` iken hedef hızına
  yaklaş, LOS hızını ve yanal göreli hızı sıfırla; sırf menzil kapansın diye
  tam gaz verme.
- **VUR:** kuyruk açısı `>=165°`, `|lambda_dot|` küçük ve koşul en az 0.4–0.6
  s kararlıysa ileri hız farkını +3/+5/+8 m/s taramasıyla aç.
- Koni bozulursa tekrar YERLEŞ; 8 m içinde 180° geri setpoint üretme.

**Kabul:** `R<10 m` yaklaşım açısı p10 `>=155°`, ortanca `>=165°`; kapanan
kare oranı yükselir, komut yönü hedef hızına ters düşmez. Elipsin düz
bacaklarında ve `hedef_duz`da regresyon olmayacak.

### 31. Constant-turn/arc hedef modeli — dönüş ıskasının P1'i

Raw APN türevi daha önce gürültü yüzünden elendi; aynı ham türevi yeniden
açmak yasak. `YILDIZ_MINRECT=1` ile işaretli dönük-bbox açısı + menzilden
arındırılmış LOS dik hızı, güven kapılı constant-turn/IMM durumuna girecek.
Önce mevcut sol-dönüş havuzu + **tek sağ elips koşusu** ile yön işareti
doğrulanacak. Sonra hedef hızı ufukta düz çizgi yerine CT yayıyla yayılacak;
PN'e hedef ivmesi feed-forward olarak yalnız güvenliyken eklenecek.

**Kabul:** `|hedef dönüş|>=8°/s` CPA ortancası **13.36→<9 m**, en az bir
`<5 m` geçiş; düz faz CPA ortancası 9.96 m'den kötüleşmez. Yanal komut-gerçek
hız farkı ve gerçek roll cevabı birlikte raporlanır.

### 32. Dikey tek-geçişli kritik sönüm — görsel dal/kalk probleminin P2'si

P+TGO hata büyüklüğünü azalttı ama yön değiştirme/chatter kaldı. `dz≈R*sin
(eps)` ve `dz_dot` ile kritik sönümlü CPA referansı kurulacak; `|dz|` küçükken
histerezis, `cmd_vz` jerk/slew sınırı ve bir angajmanda gereksiz ikinci işaret
değişimini engelleyen kapı A/B'lenecek.

**Kabul:** CPA öncesi 5 s'de gerçek `vel_z` işaret değişimi ortancası
**2→<=1**; 8–15 m dikey ray doluluğu toplamı **%28.7→<%12**; CPA `|dz|`
ortancası kötüleşmez ve yer/irtifa kapıları korunur.

### 33. Mimari karar deneyi — MPC'yi ne zaman bırakırız?

Madde 29 sonunda üç kol aynı başlangıç-geometrisi tabakalarında kıyaslanır:
tam MPC, orta-faz MPC + terminal PN/ZEM hibriti, saf PN/ZEM + emniyet
kelepçeleri. Elips + düz + wanderer, headless, en az 6 angajman/kol.

**MPC ancak** hibrit/saf kol hem CPA/temas hem tazelik hem kurtarma ölçülerinde
tam MPC'yi iki farklı rotada açıkça geçerse terminalden çıkarılır. Beklenen
hüküm: MPC orta-faz kısıt yöneticisi olarak kalır, terminali hibrit kazanır.

## HÜKÜM — neden her zaman başarılı olmuyor

Tek hata değil, **üç bağımsız yapısal katmanın zinciri**; ve baskın halka
kampanya boyunca yer değiştirdi (her tur bir halkayı kapattı, sonraki göründü,
toplam kapanma monoton eridi: ort. menzil hızı −3.47 → −0.68 m/s TUR1→TUR4).

1. **KÖK NEDEN — kapanma bir AMAÇ değil PRİM olarak kodlanmış.** Girdi seviye
   cezası `[0.0, r_hiz, r_hiz, r_yaw]` (`mpc_gudum.py:1586`) ileri kanalda
   bilinçli sıfır; ama `ivme_kos` `‖u−w‖²` demiri üç hız kanalına birden
   uygulanıyor (`:1620`), ileri dahil; kapanmayı isteyen tek terim (lineer alan
   ödülü) `/N` ile 20'ye bölünüyor (`:2018`). Sonuç: "hedefle yan yana uç"
   maliyetin geçerli bir çözümü. Veri: 24-32 m'de `u1` medyanı TUR1 +34.7 →
   TUR4 **−2.63**; o bantta hedef kadraj merkezinde, hız eşit, menzil donmuş,
   8 s formasyon uçuşu, sonra zaman aşımı. **12 ISKA zaman aşımının 12'si bu
   bantta; ≥45° kerteriz-rota açısıyla devralan 11 segmentin 11'i ıska.**
2. **Ufkun çoğu çarpışma SONRASINDA.** Ufuk sabit 2.4 s, t_go medyanı 0.61 s;
   VURUŞ'ta ufkun ort. %73'ü olmayan gelecekte, `rbar` 6 m'ye çivili, ×3
   terminal ağırlık orada. Kısıt ufku menzille ölçekleniyor (p50 0.45 s),
   maliyet ufku ölçeklenmiyor → 5 kat asimetri.
3. **Terminal körlük tasarımca üst üste biniyor.** Son 5 m'de döngülerin %78'i
   kör; segment sonu ~2.7 s %100 taze-değil; son taze ölçüm ort. 18 m. Dört
   kapı aynı fazda: bozucu donması (r<15 m), sert FOV bırakma (bayat bbox),
   kör süzülme (çözücü koşmaz), iskelet tut/süz (MPC hiç çağrılmaz, ~⅓ döngü).

### İki düzeltme (orkestratörün kendi kontrolü)
- **"FOV %96.8 bağlayıcı" YANLIŞ.** `fov_serbest=0` = "kısıt UYGULANDI", bağlayıcı
  değil (`mpc_gudum.py:2185`; `LOG_SOZLUGU.md:263` yanlış tanımlı). Gerçek
  aktiflik düşük. Bu metrik üzerinden alınmış ayar kararları şüpheli.
- **Çözücünün loglanan tıkanma anını tek-atış yeniden çözmesi GÜVENİLMEZ.** Tanı
  CSV'si sıcak başlatma/`u_onceki`/`guven`/kutu bozucuları loglamıyor; soğuk
  çözüm log komutunu tutturamıyor (fark 38 m/s görüldü). Bu başlı başına bulgu:
  uçan komut optimum değil, sıcak-başlatmaya kaymış ara nokta (iterasyon
  tavanına çarpma %46-70). **Sonuç: çözücü deneyleri KAPALI-DÖNGÜ replay ile
  yapılmalı, tek-atış yeniden çözümle değil.** (Madde 0 aşağıda.)

### Ölçüm uyarısı
TUR-4'ün "0 vuruş"u kontrol hatası DEĞİL: `vibe>15 ∧ menzil<3` yalnız saf kuyruk
geometrisinde ateşliyor; TUR-4'ün 0.94 m geçişi çapraz geometride, `vibe` 2.7'de
kaldı. TUR-4 CPA dağılımı kampanyanın en iyisi (15 segmentte 3'ü <3 m).

---

## ÇALIŞMA KURALLARI (her deneyde)
1. **Telemetriden yalnız MENZİL** (şimdilik; düşük güvenle). **Kameradan ÇIKARIM
   SERBEST** — bearing, bbox boyutu ve bunlardan kestirilen hedef kinematiği
   dahil (kullanıcı netleştirmesi 2026-08-05). Yerden tespit sistemi ayrı bir
   branch'te ele alınacak. `ref_` önekli loglama komuttan SONRA.
2. Sim koşarken ortak dosyalara dokunma. Aynı anda tek sim.
3. Ajan: yalnız Opus ya da orkestratör. Sonnet/Haiku YASAK (kullanıcı, 2026-08-05).
4. **İşe yarayanı tut, yaramayanı at, mükemmeleştir.** Her madde tek knob +
   ölçülebilir kabul ölçütü + offline→sim boşluğu açıkça işaretli.
5. Offline test bütçe KAPALI koşuyor (`mpc_test.py:596`) — uçuş rejimi değil;
   mutlak performans değil regresyon kilidi. Sim koşusu son söz.

---

## MADDE 0 — SADIK OFFLINE REPLAY HARNESS  (önkoşul, en yüksek zeka)
**Neden ilk:** Madde 1-3'ün hepsi "maliyet durağan mı, ufuk kesince ne olur"
sorusuna KAPALI-DÖNGÜ cevap ister; tek-atış yeniden çözüm güvenilmez (yukarı).
**İş:** `mpc_test.py`'nin senaryo altyapısı (düz rota, hedef 21.05 m/s, 3 devir ×
2 tohum) üzerine ÇAPRAZ geometri senaryosu ekle (kerteriz-rota ≥45°) ve tıkanmayı
offline üret. Üretemezse: gerçek `Olcum` dizisini logdan besleyen replay yaz
(sıcak başlatma doğal akışla geri gelsin).
**Kabul:** offline harness, 24-32 m'de `u1<hedef hız` tıkanmasını ≥1 tohumda
yeniden üretiyor (sim'deki %79.8'e nitel benzer). Üretemezse madde 1 doğrudan
sim'de test edilir, offline atlanır (ve bu NOT DÜŞÜLÜR).
**Zeka:** yüksek (orkestratör). **Bulk çıkarım:** Opus.

---

## ‼ EN KRİTİK AÇIK: GİMBAL STABİLİZE AMA **İZLEMİYOR** (2026-08-05 23:10)

`bbox_to_redis.py:1037` — `tilt_komutcu.hedef(eps_cmd)` **YALNIZCA BİR KEZ**,
başlangıçta çağrılıyor. Uçuş döngüsünde tilt'i güncelleyen HİÇBİR çağrı yok
(`grep '\.hedef('` → tek sonuç). Yani kamera dünya-sabit **+9.09°**'de duruyor
(= `atan(down/back)`, nominal standoff geometrisi), hedef nerede olursa olsun.

**Gimbal ±90° dönebiliyor** (SDF `tilt_joint` limitleri) — yani donanım
yeterli, komut yok.

### Geometrik sonuç: <8.2 m'de hedef kadraja SIĞMIYOR
Dikey standoff `down=4 m` iken hedefin görünen yükselişi `eps = asin(4/r)`:

| menzil | 25 m | 15 m | 10 m | **8.2 m** | 6 m | 5.5 m |
|---|---|---|---|---|---|---|
| eps | 9.2° | 15.5° | 23.6° | **29.2°** | 41.8° | 46.7° |

Kamera ekseni +9.09°, dikey yarı-FOV ±20.07° → hedef `eps > 29.16°` olunca
üst kenardan çıkar → **kritik menzil 8.21 m**.

**ÖLÇÜMLE DOĞRULANDI:** kadraj kaybı anındaki menzil ortancası
A kolu **8.1 m**, B kolu **5.5 m** (B daha derine giriyor çünkü daha hızlı
kapanıyor, kayıp anında ey = −25.2° yani hedef eksenin 25° üstünde).
Tahmin 8.2 m — ölçümle birebir.

**BU BİR GÜDÜM SORUNU DEĞİL, KOMUT EKSİKLİĞİ.** Hiçbir maliyet/ağırlık ayarı
bunu çözemez: hedef fiziksel olarak FOV dışında. Çözüm tilt'in `eps`'i
TAKİP ETMESİ (gimbal dalının "Faz C" olarak planladığı iş).

**Beklenen kazanç:** terminal bandın tamamı geri gelir (0-5 m tazelik
A %60 / B %42 → teorik ~%100), ve `vurus_hiza`'nın emekliye ayrılma
gerekçesi ("gimbal kamerayı döndürerek çözüyor") ancak o zaman geçerli olur —
şu anda tilt DÖNMÜYOR, yani gerekçe henüz karşılanmamış durumda.

## ★★ SİM A/B SONUCU (2026-08-05 22:37–22:55, hedef_duz, 360 s, GİMBALLİ)

İki koşu, aynı plan/süre/gimbal; tek fark iki env düğmesi.
A = `mpcA_baseline_duz_20260805_223745`, B = `mpcB_duzeltmeli_duz_20260805_224747`
B kolu: `YILDIZ_Q_ALAN_CARPANI=4 YILDIZ_UFUK_MENZIL_REF=60`

| ölçüt | A (baseline) | B (düzeltmeli) | hüküm |
|---|---|---|---|
| ISKA zaman aşımı | 2 | **0** | ✅ hedef buydu |
| en yakın menzil | 1.76 m | **1.30 m** | ✅ |
| CPA < 3 m (8 geçişte) | 2 | **4** | ✅ iki katı |
| kapanma `menzil_hizi` p50 | −3.23 | **−4.45** m/s | ✅ |
| kapanma ortalama | −2.25 | **−4.28** m/s | ✅ ~2× |
| `u1` p50 (ileri komut) | 25.1 | **33.9** m/s | ✅ tavana yaklaştı |
| kadraj kaybı olayı | 9 | 7 | ✅ hafif |
| **taze oran (genel)** | %72 | **%65** | ❌ bedel |
| **taze, 0-5 m** | %60 | **%42** | ❌ terminal kadraj |
| pitch hızı p50 | 11.4 | **14.2** °/s | ❌ titreme arttı |
| "menzil açılıyor" ISKA | 0 | 1 | ❌ hafif |

**HÜKÜM: düzeltme hedefini vurdu, bedeli beklenen yerden çıktı.** Zaman aşımı
tamamen bitti, kapanma ~2× arttı, metre-altı geçiş sayısı ikiye katlandı.
Bedel terminal kadrajda (0-5 m tazelik %60→%42) ve titremede — offline'da da
aynı yönde çıkmıştı (kadraj kaybı %18→%29), yani **offline→sim uyumu bu kez
tuttu**. Vuruş onayı (`vurus_basarili`) iki kolda da 0, ama o ölçüt çapraz
geometride kör (bkz. mpc_memory).

### İKİNCİ DOĞRULAMA — hedef_elips (23:12–23:27), aynı A/B
`mpcA2_baseline_elips_231245` / `mpcB2_duzeltmeli_elips_232030`

| ölçüt | DÜZ A→B | ELİPS A→B |
|---|---|---|
| ISKA zaman aşımı | 2 → **0** | 2 → 2 |
| en yakın menzil | 1.76 → **1.30** | 1.55 → **1.35** |
| CPA ortancası | 3.62 → 3.50 | **16.19 → 6.52** |
| CPA < 3 m | 2 → **4** | 3 → 3 |
| angajman sayısı | 8 → 8 | 9 → **14** |
| 0-5 m tazelik | %60 → %42 | %70 → %62 |

**İKİ ROTADA DA AYNI YÖN, FARKLI KANAL.** Düz rotada kazanç zaman aşımını
bitirmek + metre-altı geçişi ikiye katlamak; elipste ise CPA ortancasını
**2.5× iyileştirmek** (16.2 → 6.5 m) ve angajman sayısını artırmak.
Ortak: **en yakın menzil her iki rotada da iyileşti**, bedel her iki rotada
da terminal tazelik. Düzeltme rotadan bağımsız çalışıyor → **KABUL**.

Bedelin kaynağı ayrıca teşhis edildi ve bu düzeltmeyle ilgisiz:
sabit-tilt geometrisi (bkz. yukarıdaki "GİMBAL STABİLİZE AMA İZLEMİYOR").

**SIRADAKİ DARBOĞAZ ARTIK TERMİNAL KADRAJ** → TO_TEST madde 1c (PAMPC
image-plane velocity cezası) ve madde 5/6 bu bedeli geri almayı hedefler.

## ★ GİMBAL SONRASI YENİDEN ÖLÇÜM (2026-08-05 gece, offline n=32/grup)

**MPC kodunda gimbal geçişiyle değişen SADECE 2 parametre var** (eea214a→HEAD):
`pitch_baglasimi` True→False ve `vurus_hiza_kapatma` True→False (mekanizma
EMEKLİ). Güdüm yasasının çekirdeği (J, kısıtlar, çözücü, faz makinesi)
DEĞİŞMEDİ. Yani TO_TEST'teki hiçbir algoritmik düzeltme HENÜZ UYGULANMADI.

### Gimbalin tek başına etkisi (aynı MPC, farklı fizik)
| | kadraj kaybı | üst kenar | pitch hızı | **kapanma** | **zaman aşımı** |
|---|---|---|---|---|---|
| gövdeye-sabit fizik | %18.3 / %28.3 | %18.2 / %28.3 | 5.5 / 11.4 | %25 / %67 | %75 |
| **GİMBAL fiziği** | **%0.0 / %0.0** | **%0.0 / %0.0** | 5.5 / 4.1 | %25 / %67 | %75 |

**Gimbal kadraj kaybını TAMAMEN sildi** (%18-28 → %0) ve titremeyi yarıya
indirdi. **AMA kapanmaya ve zaman aşımına HİÇ dokunmadı** — 22-45 m tıkanması
aynen duruyor. İki arıza bağımsızdı, ölçüm bunu doğruladı.

### TO_TEST madde 1+3 gimbal fiziğinde HÂLÂ GEREKLİ ve HÂLÂ ÇALIŞIYOR
| kol | TIKANAN kapanma% | med | zaman aşımı% | KUYRUK kapanma% | pitch hızı |
|---|---|---|---|---|---|
| şimdiki (düzeltmesiz) | 25 | 24.7 | 75 | 67 | 9.1 |
| + q_alan ×4 | 75 | 3.2 | 41 | 67 | 4.9 |
| + ufuk ref=60 | 75 | 4.2 | 25 | 67 | 4.8 |
| **+ İKİSİ birden** | **100** | 3.7 | **0** | 67 | **4.4** |

Gimbalden ÖNCE de aynıydı (%25→%100, TO %75→%0). Yani düzeltme gimbalden
bağımsız çalışıyor; üstelik gimballi fizikte titremeyi de düşürüyor
(9.1 → 4.4 °/s) ve kadraj kaybı sıfır kalıyor. **Sıradaki sim koşusu bu.**

## ÖNCELİK REVİZYONU (kullanıcı, 2026-08-05 akşam)

Kullanıcı MPC kodunu okurken üç maddenin önceliğini YÜKSELTTİ. Yeni sıra:

| yeni sıra | madde | neden yükseldi |
|---|---|---|
| **1** | J FONKSİYONU İŞİ (madde 1 + yeni 1b/1c) | maliyet fonksiyonu kök nedendi; ödül şekli hiç literatürle doğrulanmadı |
| **2** | madde 7 — ÇÖZÜCÜ METRİĞİ (+ yeni: sabit-nokta artığı) | "çözdük mü" sorusunun ölçüsü YOK; iter tavanı %46-70 |
| **3** | madde 10 — EYLEYİCİ MODELİ | τ=1.0 iki yönde de yanlış; ıska zaman aşımı ulaşılamayan hıza kalibre |
| 4+ | eski sıra (2, 3, 4, 5, 6, 9) | — |

### 1b. ÖDÜL ŞEKLİ A/B: alan vs KAREKÖK alan  [YENİ, kullanıcı isteği]
Şu an ödül LİNEER ALAN (`A = w·h`, `_alan_guncelle` "karekok DEGIL" diyor).
Karekök kolu hiç ölçülmedi. Fark: `A ~ K/r²` iken teşvik `1/r³` ile büyür;
`sqrt(A) ~ 1/r` iken teşvik `1/r²` — yani karekök terminalde DAHA YUMUŞAK.
**Deney:** `_alan_odulu`'nda bağıl ödülü `bagil = (r0/rg)**2` yerine
`(r0/rg)` yapan bir kol (+ türev katsayısı buna göre) ve A/B koş.
**Kabul:** çapraz kapanma ve kuyruk CPA'sı q_alan×4 koluyla kıyaslanır;
terminal titreme (pitch hızı) ve kadraj kaybı yan-etki olarak okunur.
**Hipotez:** karekök daha az agresif → daha az kadraj kaybı ama daha geç
kapanma. Diz noktası aranacak. **Zeka:** yüksek (orkestratör). Offline.

### 1c. LİTERATÜR DOĞRULAMASI: J'ye iki ek terim  [YENİ]
Taramada çıkan ama HİÇ denenmemiş iki J değişikliği:
- **PAMPC image-plane velocity cezası** (arXiv 1804.04811): hedefin kadraj
  İÇİNDEKİ sürüklenme hızını cezalandır → kayıp OLMADAN önce bastırır.
  Bizim yeni darboğazımız (kadraj kaybı) tam bu.
- **ZEM-shaping terminal maliyeti** (Lee & Shin): kısa ufkun cost-to-go'su;
  ufuk t_go'dan kısayken "ufuk sonunda ne olacak" sorusunu cevaplar.
**Zeka:** yüksek (orkestratör). Offline test edilebilir.

## ÖNCELİK SIRALI DENEYLER

### 1. KAPANMA HIZINA MUTLAK TALEP  [KÖK NEDEN — OFFLINE DOĞRULANDI]
**Hipotez (revize):** İlk hipotez (`u1`'i `‖u−w‖²` demirinden muaf tut) OFFLINE
A/B'de ÇÜRÜDÜ — tek başına kapanmayı kırmıyor (min 23→13 m, zaman aşımı %75
sabit, kadraj kaybı %18→%25 arttı). **DOĞRU KOL: bbox ALAN ödülünü (`q_alan`)
büyütmek** — kapanma sinyalini menzilden değil BBOX ALANINDAN sürüyor
(kullanıcı felsefesiyle birebir: gözünle gör, kinematik çıkarma).
**Yer:** `mpc_gudum.py:2018` (alan ödülü /N), `MpcAyar.q_alan`.
**OFFLINE SONUÇ (mpc_test kapalı-döngü, capraz+yanal × duz+elips × 4 tohum;
kuyruk regresyon kontrolü ile):**
| kol | TIKANAN minR | zaman aşımı% | kadraj kaybı% | pitch hızı | KUYRUK minR |
|---|---|---|---|---|---|
| baseline | 23.0 | 75 | 18.2 | 5.5 | 1.8 |
| q_alan ×3 | 4.4 | 38 | 23.3 | 8.1 | 1.9 |
| **q_alan ×4** | **2.2** | **26** | ALT-METRE KUYRUĞU: sub-metre geçişler kayboldu, kaynağı belirsiz | **AÇIK** | düzeltilmemiş-KOR_PN havuzu (kpn1b/kpn2/kpnduz) vs düzeltilmiş (kpn3b/kpn4/d0kpn/kpn26c/kpn26d) | **0.69-0.92 m'lik geçişler YALNIZ düzeltmesiz-KOR_PN döneminde görüldü** (kör \|ex\| kadraj dışına, 28-47°'ye kaçarken). R3′ kelepçesi kaçağı kesince kuyruk da gitti: en iyi geçiş 0.69/0.73/0.75 → 1.08 (tavan 20) → 1.22 (tavan 26). Kelepçe 26 **medyanı** düzeltti (ortak katmanda CPA3B 5.99→3.50, yatay 4.66→2.77, <3m %14→%40) **ama kuyruğu getirmedi**. Yani sub-metre geçişleri üreten şey kelepçe genişliği DEĞİL. Adaylar: (i) kaçak ilerletmenin şansı (kadraj dışı hedefe körlemesine gidip tutturmak — tekrarlanamaz, güvenilmez), (ii) o dönemin farklı bir yan koşulu (dikey kol/profil), (iii) sub-metre CPA zaten şans kuyruğu ve n ile gelir. Ayırt etmek için: aynı konfigürasyonda n'i büyütmek (havuz 10→20+) ve <1.5 m geçişlerin devir geometrisini kıyaslamak |
| **27** | KURTARMA SIRASINDA KONTROLSÜZ YAW (500-800 deg/s) | **AÇIK — kazılacak** | kpn26c (149-505 deg/s), kpn26d (663, 813, 546, 356 deg/s) | HOLD/BRAKE kurtarması sırasında araç **500-800 deg/s** hızla dönüyor. **Komut değil:** HOLD'da güdüm setpoint göndermiyor (`send=False`, mod BRAKE) ve `ATC_SLEW_YAW`=180 deg/s — ölçülen 813, bunun **4.5 katı**. **Artefakt da değil:** değer `ATTITUDE.yawspeed`'den, yani doğrudan gyro (`mavlink_utils.py:110-115`), sayısal türev değil. Yani agresif manevradan/departure'dan artan **kontrolsüz açısal momentum**. HOLD'un `spinning` uzatması (RECOVERY_YAW_RATE_HOLD_DPS=40) tam bunun için var ve çalışıyor (14 kurtarmanın çoğu bu koldan uzadı). SORULAR: (i) momentum nereden — yatay fren mi, departure artığı mı, BRAKE modunun yaw'ı serbest bırakması mı; (ii) **kadraj/tespit bedeli ne** — 800 deg/s dönerken bbox kesin kayboluyor, bu kurtarma sonrası yeniden-edinim süresini uzatıyor olabilir; (iii) yaw otoritesini kurtarmada aktif kullanmak (sönümlemek) mümkün mü |
| **25** | 20.4 | 9.0 | **1.9** |
| q_alan ×6 | 4.4 | 12 | 23.3 | 10.0 | 2.0 |

**Diz noktası ×3-4:** tıkanma menzili 23→2.2 m, zaman aşımı %75→%25, **kuyruk
geometrisi BOZULMADI** (1.8→1.9 m). ×6'da min menzil geri yükseliyor (aşırı
kapanma / delip geçme). u1 demiri asıl kol DEĞİL.
**Kabul (sim):** 24-32 m tıkanma menzili düşer, zaman aşımı ≤%30, KUYRUK CPA
regresyonu yok. **OFFLINE→SİM BOŞLUĞU:** kadraj kaybı ve titreme yan-etkisi
offline'da az güvenilir (motor terminale düzleşmiş giriyor); min-menzil ve
zaman aşımı güvenilir. Sim'de doğrula: q_alan ×4 duvarı kırıyor mu, kadraj
kaybı/titreme baseline üstüne çıkmıyor mu.
**Risk:** aşırı agresif → delip geçme + kadraj kaybı; ×3-4 bandında kal.
**Zeka:** yüksek (orkestratör). **Durum:** OFFLINE OK, SİM BEKLİYOR (başka ajan
sim'de).

### 2. DEVİR KAPISINA GEOMETRİ ŞARTI  [en ucuz-en yüksek getiri]
**Hipotez:** `|kerteriz − rota| ≥ ~45°` iken devri geciktir / "önce kuyruğa geç".
11/11 ıska bu ayırıcıyla ÖNCEDEN biliniyor (<45°: 31 seg, min menzil ort. 4.5 m;
≥45°: 11 seg, hepsi ıska, ort. 23.7 m).
**Yer:** `bbox_to_redis.py` devir kapıları (ORTAK DOSYA — A/B tek koşuda).
**Kabul:** devralınan segmentlerin ≥45° oranı ~0; ıska zaman aşımı düşer.
**Risk:** devir sayısı düşer, angajman başına bekleme uzar (şu an ~17.7 s/devir).
**Zeka:** düşük-orta. **Yürütücü:** Opus (orkestratör kabul ölçer).

### 3. MALİYET UFKUNU MENZİLLE ÖLÇEKLE  [OFFLINE DOĞRULANDI]
**t_go DEĞİL, MENZİL.** `mpc_gudum.py:1765` notu: t_go ölçeklemesi ZATEN denenmiş
ve çapraz geometride ETKİSİZ çıkmış (kapanma ~0 → t_go sonsuz → ölçekleme hiç
devreye girmiyor). Kısıt ufku bu yüzden menzille ölçekleniyor (`:1767`); maliyet
ufku ölçeklenmiyordu — asimetri buradaydı.
**Uygulama:** `_adim_sureleri`'de `adim_s *= clip(r/ref, taban, 1.0)`, ref≈60 m.
**Hipotez ÇÜRÜDÜ / yeni mekanizma:** terminal tekrarlanabilirliği DÜZELTMEDİ
(tohumlar-arası saçılma 8.85→9.15). Gerçek etki: yakın menzilde ufku kısaltmak
MPC'yi MİYOP yapıyor; "yan yana uç" uzun-ufuk dengesi, alan ödülü ise ANLIK →
miyopluk kapanmayı serbest bırakıyor.
**OFFLINE SONUÇ (n=32/grup, 8 tohum):** tıkanan kapanma %25→%75, zaman aşımı
%75→%25, kuyruk BOZULMADI. ref=45 ve 60 çalışıyor, 35 ve 20 etkisiz.
**Durum:** OFFLINE OK, SİM BEKLİYOR.

### 1+3 BİRLİKTE — 2×2 FAKTÖRİYEL (n=32/grup, 8 tohum)  ★ ANA BULGU
| kol | TIKANAN kap% | med | zaman aşımı% | kadraj kaybı% | KUYRUK kap% |
|---|---|---|---|---|---|
| baseline | 25 | 22.2 | 75 | 18.3 | 67 |
| M1 (q_alan ×4) | 75 | 2.0 | 22 | 19.6 | 67 |
| M3 (ufuk ref=60) | 75 | 4.5 | 25 | 23.8 | 67 |
| **M1+M3** | **100** | 3.7 | **0** | **29.3** | 67 |

**TOPLANIYORLAR** (redundant değil). Koşu bitiş sebebi dökümü — asıl kanıt:
| kol | TIKANAN bitişleri | KUYRUK |
|---|---|---|
| baseline | ISKA_BIRAKTI 24, KADRAJ_KAYBI 8, **ÇARPIŞMA 0** | ÇARPIŞMA 16, ISKA 8 |
| M1+M3 | KADRAJ_KAYBI 16, ISKA_BIRAKTI 9, **ÇARPIŞMA 7** | ÇARPIŞMA 16, ISKA 8 |

**Yorum:** Düzeltme "pes et"i "angaje ol"a çeviriyor — çapraz geometride İLK
çarpışmalar (0/32 → 7/32). Kuyruk geometrisi hiç etkilenmiyor (16/16 çarpışma
her iki kolda), yer teması yok (min irtifa 47.2 m). **Bedel: kadraj kaybıyla
biten koşu 8→16.** Yani kadraj kaybı artışı bir REGRESYON değil, daha önce
"güvenle yan yana uçarken" hiç ödenmeyen angajman bedeli — ama YENİ DARBOĞAZ.
**Sıradaki mantıklı adım:** madde 5 (dikey ivmeyle pitch telafisi) + madde 6
(pitch gecikmesi) — agresifliği korurken kadrajı geri kazanmayı hedefler.
**[GİMBAL DALI GÜNCELLEMESİ 2026-08-05: bu "sıradaki adım" değişti. Madde 6
gimballe KAPANDI ve madde 5 İKİYE BÖLÜNDÜ (aşağıya bak). Kadraj kaybı
darboğazının dikey bileşeni artık pitch'le değil TILT'le çözülür — dinamik
tilt takibi (madde 5b) daha ucuz ve daha doğrudan bir kol. Yatay/roll
bileşeni açık kalıyor (gimbal tek eksen).]**

### 4. KÖR TERMİNAL: "son LOS hızını dondur, PN sürdür"
**Hipotez:** Kör süzülmeyi düz süzülme yerine son geçerli LOS hızını dondurup
PN'i açık-çevrim sürdürmeye çevir (JHU APL homing; IEEE 9066667). Son 18 m
görülmeden uçuluyor → doğrudan CPA.
**Yer:** `mpc_gudum.py:2741` (`_kor_komut`), `:2891` kör süzülme kapısı.
**Kabul:** kör segmentlerde CPA ortancası düşer; kör süre değişmez.
**Risk:** gürültülü son ölçüm donarak büyür → bbox güven/boyut kalite kapısı.
**Zeka:** düşük-orta. **Yürütücü:** Opus (orkestratör kalite kapısını tasarlar).

### 5. DİKEY İVMEYLE PİTCH SATIN AL
**Hipotez:** `θ≈atan(a_ileri/(g+a_yukarı))`; ölçülen 5.8°/(m/s²)≡1/g yani model
`a_yukarı=0` varsayıyor. `a_yukarı=+3` → burun 27°→21.3°. Aynı manevra iki modu
söndürür: burun kalkar + standoff altta olduğundan `eps=asin(down/r)` küçülür.
Literatür: StableTracker (arXiv 2509.14147).
**Yer:** `mpc_gudum.py:1826` (pitch=f(ivme)), dikey ivmeyi serbest bırak.
**Kabul:** üst-kenar fiziksel-dışı (%7.4) düşer; alt-kenar (%4.2) artmaz; dikey
hız tavanı (`WPNAV_SPEED_UP`) aşılmaz. **Risk:** dikey tavan modelde yoksa plan
uygulanamaz → modele gir. **Zeka:** yüksek (orkestratör). **Bulk:** Opus.

> **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05 — MADDE 5 İKİYE BÖLÜNDÜ]**
> Bu maddenin iki modundan **BİRİNCİSİ ÖLDÜ**: "burun kalkar → kamera kalkar"
> zinciri fiziksel tilt gimbaliyle koptu (gövde ±35° iken kamera max 0.65°).
> Pitch satın almanın kadraja hiçbir faydası kalmadı.
> **İKİNCİ mod AÇIK ama çaresi değişti:** `eps=asin(down/r)` yakın menzilde
> hâlâ büyüyor (standoff altta), ama artık bunu dikey ivmeyle değil **TILT'i
> dinamik komutlayarak** kapatmak gerekiyor:
> - **5a (ÖLDÜ):** dikey ivmeyle pitch telafisi — gimballe anlamsız.
> - **5b (YENİ, ucuz, yüksek getiri):** **DİNAMİK TILT TAKİBİ.** Tilt şu an
>   STATİK: `YILDIZ_TILT = atan(down/back)` bir kere komutlanıyor. Terminalde
>   `eps` bunu aşıyor. Kol: tilt'i menzille/ölçülen `ey` ile canlı komutla
>   (gimbal 6 rad/s izliyor, ölü bant ±0.17°).
>   **Kabul:** üst-kenar fiziksel-dışı kayıp düşer; tilt komut-durum farkı
>   <0.06 rad kalır; alt-kenar artmaz. **Risk:** tilt'i bbox'tan sürmek kapalı
>   döngü kurar → bayat/kayıp bbox'ta tilt'i DONDUR, standoff değerine dön.
> - Not: madde 1+3'ün yeni darboğazı olan "kadraj kaybı"nın DİKEY bileşeni bu
>   koldan çözülür; yatay bileşen yaw'da kalır (gimbal tek eksen).

### 6. beta'DAKİ 0.30 s PİTCH GECİKMESİNİ KALDIR
**Hipotez:** Pitch hızı p95 48°/s → LPF `beta`'da p95 ~14° hata (dikey yarı-FOV
20°). Sert FOV kısıtı "0.3 s önceki" kadrajı görüyor.
**Yer:** `mpc_gudum.py:839` (`pitch_lpf_tau_s`), `:2480` beta kurulumu.
**Kabul:** üst-kenar kaybı düşer; bant titremesi geri gelmez. **Risk:** LPF'yi
tam kaldırmak titretebilir → ham piksel kenar payını ayrıca kısıta besle.
**Zeka:** orta (orkestratör). **Bulk:** Opus.

> **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05 — MADDE 6 KAPANDI]**
> Bu maddenin varlık sebebi "kamera ekseni = montaj + gövde pitch"ti; pitch
> LPF'si o eksene 0.30 s gecikme bindiriyordu. Fiziksel tilt gimbaliyle pitch
> kamera ekseninden ÇIKTI, dolayısıyla LPF'nin `beta`ya kattığı gecikme de
> çıktı. **Yeni iş, ayarlama değil DÜZELTME:** `beta` kurulumu hâlâ pitch'ten
> besleniyorsa bu artık bir AYAR sorunu değil, YANLIŞ BİRİM — eksen
> `tilt_status` olmalı. (Kod `mpc_gudum.py`'de; bu dalga sırasında başkası
> çalıştığı için burada yalnız işaretlendi.) Kalan gerçek gecikme kaynağı:
> gimbal servo takibi (ölçüldü <0.06 rad) ve roll (tek eksen, telafi yok).

### 7. ÇÖZÜCÜ: ÖNCE METRİK, SONRA BÜTÇE  [ÖNCELİK 2 — yükseltildi]
**7a. OPTİMALLİK METRİĞİ EKLE (önce bu).** Şu an durma ölçütü
`max|ΔZ|·ağırlık < tolerans_mps` (`:2357`) — "adım küçüldü" der,
"optimumdayım" DEMEZ; FISTA takılsa da adım küçülür. Elimizde `sure_ms`,
`iter`, `maliyet` var ama **optimallik sertifikası yok**.
**Öneri:** sabit-nokta artığını logla:
    `art = ‖z − Π(z − (1/L)∇f(z))‖`
Bu, izdüşümlü gradyanın KKT artığıdır: 0 ise nokta optimaldir. Ucuz
(zaten hesaplanan `g` ve `Π` ile tek ek satır), ve "gerçekten çözdük mü"
sorusunu ilk kez cevaplar. `mpc_tani`'ye `kkt_artik` kolonu.
**Kabul:** `iter`, `sure_ms` ve `kkt_artik` birlikte okunabiliyor; tavana
çarpan döngülerde artığın gerçekten büyük olduğu (ya da olmadığı) görülüyor.
Bu ölçüm OLMADAN bütçe ayarı körlemesine yapılır.
**7b. BÜTÇE.** p95 13.4 ms / bütçe 13 ms, aşım %13-16, iter-tavan çarpma
%46-70 → komut sıcak-başlatmaya yanlı, madde 1'i güçlendiriyor. Blok/ufuk
düşür ya da bütçeyi ölçülen döngü hızına bağla.
**7c.** Offline testler bütçe KAPALI koşuyor (`mpc_test.py:596`) — yani
ayarlar uçuşta hiç görülmeyen bir çözücü rejiminde seçilmiş. Ayrı düzeltme.
**Yer:** `mpc_gudum.py:857`, `:888`, `:2340-2362`; `mpc_test.py:596`.
**Zeka:** orta-yüksek (orkestratör).

### 8. BEARING-ANGLE TMA — kameradan hedef kinematiği  [AÇIK, yüksek tavan]
**KURALIN NET HALİ (kullanıcı, 2026-08-05):** "Gözüyle görüp ÇIKARIM YAPABİLİR;
telemetriden şu an SADECE RANGE al." Yani **kameradan türetilen her şey serbest**
(bearing, bbox boyutu, bunlardan kestirilen hedef hızı/yönü); yasak olan
telemetriden/yerden tespitten kinematik almak. Menzil düşük güvenle kullanılıyor.
Yerden tespit sistemi AYRI BİR BRANCH'te hesaba katılacak (kullanıcı açacak).
**Hipotez:** bbox merkezi=kerteriz, bbox boyutu=gerilen açı (kamera dönüşüne
değişmez); 7-durumlu pseudo-linear KF hedef konum+hız+bilinmeyen fiziksel boyutu
birlikte kestirir (Ning et al. IJRR 2024). Klasik bearing-only TMA gözlemcinin
kerterize DİK manevrasını şart koşar — tam bizim kuyruk geometrimizde iflas eder;
bearing-angle bu diklik şartını kaldırır.
**Açtığı kapı:** madde 1'in tam sürümü (ZEM tabanlı terminal maliyet), madde 2'nin
tam geometri kapısı, madde 4'te daha iyi kör-terminal ekstrapolasyonu.
**Kabul:** offline harness'ta ground-truth'a karşı hedef hızı <%15 hata ve KUYRUK
geometrisinde yakınsıyor (klasik bearing-only'nin battığı yer).
**Risk:** görünen bbox genişliği aspect-angle ile 3-4x değişir (sabit kanat
yandan/önden) → menzil/boyut hatası. Azaltma: köşegen veya yükseklik kullan,
telemetri menzilini BAĞIMSIZ ölçüm olarak filtreye ver (menzil zaten serbest).
**Zeka:** yüksek (orkestratör). **Durum:** offline prototip sırada.
**Not:** Madde 1 bunsuz da çözüldü (kapanma bbox alanından); TMA artık bir
GEREKLİLİK değil, TAVAN YÜKSELTİCİ.

### 9. ÖLÇÜM ARAÇLARINI ONAR  [performans değil, görünürlük]
- `vurus_basarili` çapraz geometride kör → CPA + dikey ayrım kriteri ekle
  (`vibe`'a ek). `mpc_gudum.py:2493`.
- `fov_serbest` / `bos_sayac` sözlük tanımı yanlış. `LOG_SOZLUGU.md:263-264`.
- `r_ic` 3 m tabanlı → son 3 m durum makinesi kör. `mpc_gudum.py:2452`.
- Sözlük VURUŞ çarpanları tur-1'de kalmış (kod 1.0). `LOG_SOZLUGU.md:275`.
**Zeka:** düşük. **Yürütücü:** Opus.

### 10. EYLEYİCİ MODELİNİ İVME-SINIRLI YAP  [ÖNCELİK 3 — yükseltildi]
**Hipotez:** 1. mertebe τ=1.0 iki yönde yanlış (büyük komutta 2.7x iyimser,
küçükte 4x karamsar); model 35 m/s sanıyor, ölçülen max 29.9; `iska_zaman_asimi_s
=8` ulaşılamayan 13.9 m/s kapanmaya kalibre. Rate-limited modelle.
**Yer:** `mpc_gudum.py:829`, `:1715`; `:1231` zaman aşımı.
**Kabul:** model öngörülen kapanma ile ölçülen <%20 fark; zaman aşımı yeniden
temellenir. **Risk:** madde 1 olmadan tek başına sadece daha uzun yan-yana uçar.
**Zeka:** orta (orkestratör). **Bulk:** Opus.

---

## ÖNERİLEN TUR YAPISI
- **TUR-5a (ucuz paket, tek sim):** 2 + 4 + 9. Geometri kapısı + kör terminal +
  ölçüm onarımı. Tek koşuda A/B okunur.
- **TUR-5b (yapısal, tek başına):** 1 (madde 0 harness önce). Kapanma talebinin
  YAPISI değişiyor — etkisi izole okunmalı.
- **TUR-5c:** 3 (ufuk kesme), sonra 5-6-7 (pitch/gecikme/çözücü hijyeni).
  **[GİMBAL DALI GÜNCELLEMESİ: 5-6 yerine artık **5b (dinamik tilt takibi)**
  + 7. Madde 5a öldü, madde 6 kapandı — bkz. yukarısı.]**
- **AYRI PROJE:** 8 (bearing-angle TMA) — 1'in tam ZEM sürümü buna bağlı.

## ‼ MADDE 11 — DİKEY KANAL FAZ C'DEN SONRA KÖR (2026-08-07, AÇIK İŞ)

**Bu, şu anki en kritik açık.** Dikey standoff'u kapatan tek maliyet terimi
`q_ey` idi ve `beta = ey − ey_ref` üzerinde tanımlı (`_maliyet_satirlari`,
`mpc_gudum.py:~2148`). FAZ C ile `_kadraj_sabiti` (`~:2699`) `ey_ref`'i **canlı
gimbal tilt'ine** bağladı; tilt eps'i izlediği için `beta ≈ 0` → **maliyet dikey
hatayı GÖRMÜYOR**. Kalan tek dikey terim `sigma_el → 0` ve o, tanımı gereği
"dikey standoff'u **koru**" diyor.

**Ölçüm (DOWN=+4, düz):** hedef−kamera ekseni farkı −1.3° görünürken gerçek
dikey ayrım 4 m. `|dz|` ortancası 45→15 m boyunca **3.4-4.0 m sabit**;
`hiza_ref` aktif kare **0**. Elipste CPA 2.24 m'nin **2.02 m'si dikey artık**.
Elenen hipotezler: `‖u−w‖²` freni (dikey CBF kutusuna değme yalnız %11) ve
tırmanma tavanı (gereken ≈2.5 m/s, tavan 9.0).

**DENEY-1 (yapıldı, kodda duruyor):** `YILDIZ_DIKEY_TERMINAL=1` — dikey LOS hızı
referansına iki yanlı, menzille rampalı (45→25 m) bias. 6 koşu (düz ×2 çift,
elips ×1 çift, DOWN=+4). **Mekanizma çalışıyor** (`hiza_ref` %0→%65-83, terminal
eps 7.0→4.5, 20-8 m bandı `|dz|` 2.59→1.98), ISKA 5→0, regresyon yok.
**AMA CPA'daki dikey artık düşmedi** (havuz |dikey| ortanca 0.99→1.09); düzde
2/2 hafif kötüleşti, elipste net iyi (dikey 2.02→0.63, CPA 2.24→1.56).
**Varsayılan KAPALI kaldı.** Teşhis: türev sürülüyor ama hata terimi yok —
P'siz D kontrolü.

**DENEY-2 (yapıldı):** `YILDIZ_DIKEY_HATA` (çarpan, varsayılan KAPALI;
`q_dikey_hata=1.5`, ağırlık çözücü taramasıyla türetildi) — maliyete **eps
üzerinde doğrudan hata terimi** (6. satır bloğu, `n_satir 5N→6N`). `q_ey` ve
FOV/CBF canlı gimbal ekseninde kaldı; yalnız yeni satır "hedef hattı"nı
referans alıyor.

**SONUÇ — teşhis değişti: sorun artık HATA değil AŞMA.** P terimi dikey hatayı
gerçekten sıfırlıyor; `|dz|` profili **ilk kez monoton kapanıyor** (elips
DOWN+4: 25-35 m 3.83 → 8-15 m 0.92 → 0-8 m 0.67; kapalı kolda 3.29/2.07/1.51).
Ama CPA artığı düşmüyor çünkü **sıfıra büyük hızla varıyor**:

| kol | sıfır geçişi | dikey hız @CPA | CPA dikey |
|---|---|---|---|
| kapalı (düz, en iyi) | r=2.70 m, 0.25 s önce | **−0.35 m/s** | −0.13 |
| P q1.5 (düz) | r=2.28 m, 0.20 s önce | **−5.13 m/s** | −1.22 |
| P q1.5 (elips) | r=4.21 m, 0.20 s önce | **−4.54 m/s** | −1.10 |

İşaret bunu doğruluyor: kapalı **+2.02** (yetişemiyor), P kolu **−1.10/−1.22**
(geçip gidiyor). 0.2 s × ~4.5-5 m/s ≈ 0.9-1.0 m = kalan artığın tamamı.
**P+D çözmüyor:** `dikey_terminal_tau_s=1.5 s` sabit ve r<15 m'deki t_go'dan
(<1 s) uzun → erken fren, sıfıra hiç varmıyor (P+D elipste sıfır geçişi YOK).

**ROTA-ÖZGÜLÜK (önemli):** düz rotada taban CPA dikey **0.13/0.30 m** — ölçüt
zaten sağlanıyor; DOWN=0'da 0.18 m; elipste 2.02 m. Sorun **hedef dönerken**
çıkıyor. P kolu düz (0.13→1.22) ve DOWN=0 (0.18→1.93) rotalarında **regresyon**
yarattı. İki düğme de varsayılan KAPALI kaldı.

**DENEY-3 (yapıldı) — ★ ÖLÇÜT SAĞLANDI.** `YILDIZ_DIKEY_TGO` (çarpan,
varsayılan KAPALI; `dikey_tgo_k=2.0`). D kolunun zaman sabiti sabit `tau`
yerine **kalan zamana** bağlandı: `tau_eff = clip(t_go/k, 0.30, 1.50)`,
`sigma_el_ref = dikey_s·k·eps_fazla/t_go`. t_go kontrolcünün mevcut
`menzil_hizi`'nden (yeni kestirim yok); kapanma <1.0 m/s ise yok sayılır →
`tau_max`.

**Neden aşmayı kesiyor (kapalı form):** `de/dt = −k·e/(T−t)` → `e(t) =
e0·((T−t)/T)^k`; k>1 için **hem e hem ė** çarpışmada sıfıra gider. Sabit tau'da
`e = e0·exp(−t/tau)`: e küçülür ama ė asla sıfırlanmaz — ölçülen −4.5…−5.1 m/s
tam olarak budur.

**ZORUNLU İKİNCİ PARÇA:** eski sabit **8 dps** açısal tavan yasayı terminalde
boğuyordu (45 m'de 6.3 m/s = alçalma kutusunun ÜSTÜ, 15 m'de yalnız 2.1 m/s);
kırpılmış talep = sabit `eps_dot` = aşmanın kendisi. Bu düzeltme olmadan k'yi
büyütmek **hiçbir şey** değiştirmiyordu (dikey hız @CPA k=2.5 ile k=8 arası
−3.35'te sabit). Tavan artık fiziksel girdi kutusundan (tırmanma 9 / alçalma
4.5) yöne göre türetiliyor.

**A/B — CPA dikey artık [m]:**

| Rota / DOWN | kapalı (taban) | P (DENEY-2) | **P+TGO k=2** |
|---|---|---|---|
| Elips +4 (asıl açık) | +2.02 / +1.03 | −1.10 | **−0.41 / +0.15** |
| Düz +4 (regresyon kapısı) | +0.30 | −1.22 ❌ | **+0.06** |
| Elips 0 (regresyon kapısı) | +0.18 | −1.93 ❌ | **+0.09** |

P'nin iki regresyonu **tamamen onarıldı**. Yatay CPA elipste (0.96/1.84→0.24/0.21)
ve DOWN=0'da (2.45→0.65) **iyileşti**; ISKA 10→2-4. **Kampanyanın tek fiziksel
temasları bu kolda:** üç rotada üç `vurus_basarili` (vibe 234/40/24) — kapalı,
D, P, P+D, k=3 kollarının hiçbiri temas üretmedi.

**METODOLOJİK DERS:** offline tarama k≈2.6-2.8 dedi, **kapalı döngü 2.0 dedi**.
Offline'ın başarısızlık kipi *aşma* (büyük k ödüllenir), gerçek koşununki
*geç kalma*. **Kural: offline tarama yön verir, kararı kapalı döngü verir.**

**DENEY-4 (yapıldı) — üç kapının üçü de geçildi. HÜKÜM: AÇILSIN.**

**(A) Wanderer** — mekanizmanın **en temiz kazancı burada**. Kapalı kol dikey
standoff'u *hiç* kapatmıyor (bant profili düz: −2.34 → −2.18 m); ana kol monoton
kapatıyor. Terminal `|dz|`: 15-8 m **2.81 → 0.52**, 8-0 m **2.18 → 0.59**.
`irtifa_abort` 1 vs 1, ISKA 5 vs 5.
**AMA wanderer'da CPA dikey ÖLÇÜLEMİYOR:** angajmanlar iki kolda da 13-37 m'de
kopuyor. Sebep dikey değil → **madde 13** (kapanma arızası).

**(B) `irtifa_abort` — TANI TAMAM, ZARARSIZ.** Tetik
`simple_guided_follow.py:3171`: `alt_error = |pursuer_z − aim_z_nominal|` ve
`aim_z_nominal` (`:2975`) **konumlu SLOT'un z'si** — görüntülü kol o slotu
izlemek zorunda değil. 23 koşu / 67 abort sınıflandırıldı:

| sınıf | adet | yorum |
|---|---|---|
| angajman içi | 28/67 | **literal no-op** — `send_setpoint`/`set_mode` zaten `konumlu_yetkili` ile kapılı (67'nin yalnız 42'sinde `recovery → HOLD` satırı var) |
| devir sonrası ≤5 s | 27/67 | **27/27'sinde menzil açılıyordu** — pas bitmişti, fren doğru davranış |
| konumlu faz | 12/67 | menzil 96-795 m, **baseline'da da aynı** |
| kapanan yaklaşmayı kesen | **0/67** | — |

Sayının kendisi gürültü (aynı konfig tekrarları 1 vs 6, 2 vs 5, 3 vs 1); yeni
A/B çiftlerinde artış **yok** (wanderer 1-1, elips−3 3-3). Kök neden kayda
geçti: 2026-07-24'te LOS spear için `aim_z_nominal` istisnası konmuş, **görüntülü
devir hiç kapsanmamış** → madde 14 (düzeltme `simple_guided_follow.py`'de,
bugün gerekmiyor).

**(C) DOWN=−3 asimetrisi — HİPOTEZ ÇÜRÜTÜLDÜ.** k=3'teki ters aşma (+1.75)
**k=2'de tekrarlamıyor**: işaretli dz 8-0 m **+0.09 vs kapalı +0.92**.
"Alçalma kutusu (4.5) tırmanmanın (9) yarısı" iddiası üç katmanda ölçüldü,
**hiçbiri bağlayıcı değil**: maliyet referansı tavanı alçalma dalında
**674 karenin 0'ı kırpıldı (%0)**; `cmd_vz`'nin +4.5'e değmesi %0.0-0.7; CBF
dikey dilimi DOWN'a özel değil (−3'te %21, 0'da %20). Yöne göre ayırma **no-op**
olurdu → kod değişikliği yapılmadı.

**AÇILMA GEREKÇESİ:** işaretli terminal dikey artık **4/4 hücrede** iyileşiyor;
ISKA her hücrede düşüyor; kampanyanın **3 fiziksel temasının 3'ü** bu kolda
(vibe 234/41/40); DOWN +4/0/−3'te regresyon yok; abort bedeli zararsız kanıtlandı.

**DÜRÜST KAYIT:** ölçütün *harfi* wanderer'da sağlanmıyor (CPA dikey 2.51) — ama
orada ölçüt dikey kanalı ölçmüyor (kapalı kolun 1.86'sı da kabul dışı).
Ve **k=2 vs k=3 ÇÖZÜLMEDİ**: bant metriğinde k=3 daha tutarlı (4/4), k=2 iki uçlu
(aynı konfigürasyonun iki koşusu 0.19 ve 2.78); k=2'yi seçtiren fiziksel temaslar.
**Konfigürasyon içi yayılım, hücreler arası farktan büyük** → n≥3 şart.

**DENEY-5 (yarıda kesildi — kullanıcı sahaya çıktı).** `YILDIZ_COZUCU_BOL`
(iter 26→40, bütçe 13→18 ms, **sim ölçüm kalitesi için**) ile 11 koşu yapıldı;
`BOL=0` (sevkiyat ayarı) doğrulaması 2 koşuda kaldı.

**ÖNEMLİ — DÜRÜST DÜZELTME:** DENEY-5 koşuları DENEY-3'ün manşetinden
(elips k=2: 0.41/0.15) **belirgin kötü** çıktı. CPA'daki |dikey| artık:

| rota | k=2 | k=3 |
|---|---|---|
| elips (n=3) | 1.39 / **0.09** (VURUŞ, vibe 72) / 1.59 | 1.74 / 0.13 / 1.50 |
| düz (n=2) | 0.11 / 1.80 | 1.10 / 0.97 |
| **havuz ortanca** | 1.39 (ort 1.00) | 1.10 (ort 1.09) |

**Sonuç: DENEY-3'ün "CPA |dikey| < 0.4 m" manşeti tekrar ÜRETİLEMEDİ.** Aynı
konfigürasyonun kendi içindeki yayılım (0.09→1.80) kollar arası farktan büyük,
yani **k=2 vs k=3 ayrışmıyor** (madde 11d açık kalır) ve tek koşuluk CPA
değerleriyle hüküm verilemez.

**Ama mekanizmanın işe yaradığı SAĞLAM** — güvenilir sinyal CPA tek noktası
değil, **bant metrikleri**: wanderer'da kapalı kol dikey standoff'u *hiç*
kapatmıyor (profil düz −2.34→−2.18), ana kol monoton kapatıyor (terminal
2.18→0.59); ISKA her hücrede düşüyor; kampanyanın 3 fiziksel temasının 3'ü bu
kolda. **Doğru ifade:** "dikey kanal artık görüyor ve kapatıyor" ✅ /
"her koşuda 0.4 m altına iniyor" ❌.

**SEVKİYAT KARARI (2026-08-07, kullanıcı):** irtifa düzeltmesi **uçuşta
kullanılacak**, ama **ENV ile açılarak** — varsayılan 0 kaldı:

```
YILDIZ_DIKEY_HATA=1 YILDIZ_DIKEY_TGO=1   # ucusta ACIK
```

`dikey_tgo_k=2.0` korundu (temas üreten kol; veri k'yi ayırmaya yetmedi).
`YILDIZ_DIKEY_TERMINAL` (eski D kolu) ve `YILDIZ_COZUCU_BOL` kapalı
(ikincisi Pi 5'te bütçe zaten dar).

**MADDE 11e — NEDEN VARSAYILAN DEĞİL (açık iş):** varsayılanı 1 yapınca
`mpc_test` **77/86**'ya düşüyor. Kalan 9 testin 8'i dikey kanalın **ESKİ**
tasarımını doğruluyor — `hiza_ref`'in 8 dps tavanda doyması, `eps<0`'da bias
verilmemesi (tek yanlı), `vurus_hiza` karışımı, `--vurus_hiza_kapatma`.
Kampanya bunların **hepsini bilerek değiştirdi** (tavan artık fiziksel hız
kutusundan; bias iki yanlı, çünkü irtifa-agnostiklik isteniyor). Yani testler
eskimiş, kod yanlış değil — ama 8 test beklentisini uçuş öncesi aceleyle
yeniden yazmak doğru olmadığı için **sim'de doğrulanan kurulumun birebir
aynısı** sevk edildi (tüm A/B koşuları env ile yapıldı). 9. test yük duyarlı
zamanlama ölçümü (20.85 µs vs 20 µs eşiği).
**Varsayılanı açmadan önce:** bu 9 testi yeni tasarıma göre güncelle.

## ★ İRTİFA VARYANTI BULGUSU (2026-08-07, düz rota, n=1/varyant)

`--down` işareti: **pozitif = avcı hedefin ALTINDA** (`simple_guided_follow.py:1798`).
Mevcut varsayılan `DOWN=+4` yani alttan takip.

| | +4 (alttan, mevcut) | **0 (aynı irtifa)** | −3 (üstten) |
|---|---|---|---|
| Gerçek CPA | 2.53 m | **0.67 m** | 1.42 m |
| CPA dikey artık | **+0.92 m** | −0.36 m | +0.64 m |
| Devir menzili (medyan) | 23.7 m | **21.1 m** | 28.0 m |
| <6 m kapanan angajman | 2/8 | **7/13** | 1/10 |
| Yakın bant (<25 m) tespit | 63.5% | 69.0% | **86.8%** |
| ISKA | 3 | 6 | 8 |

**Öneri:** `DOWN=0`; kadraj kaybı korkusu ağır basarsa −1..−2 (hafif üstten).
Alttan takip terminal faza uçulmamış dikey artık bırakıyor ve hedefin görünen
yükselişini 72°'lik yaya sokuyor (hız-limitli gimbal yetişemiyor).
**Not:** `tools/senaryo.sh:18`'deki "DOWN'u ezersen kamera ekseni bozulur!"
uyarısı **artık geçersiz** — FAZ C tilt takibi geometri değişimini soğuruyor
(üç varyantta gimbal hatası aynı: medyan 0.162-0.168°, p95 ~0.41°).

## PENCERE 45 → 25 KARE (2026-08-07, UYGULANDI)

Karar penceresi **kare** cinsindendir, süre değil. Sim'de kamera 30 fps
(45 kare = 1.5 s) ama gerçekte YOLO ~20 Hz → aynı 45 kare **2.25 s** eder,
devir sessizce gecikirdi. 25 kare: 30 fps'te 0.83 s, 20 Hz'de 1.25 s; oran
%80'de kaldı (20/25). Env: `YILDIZ_PENCERE_KARE` / `YILDIZ_PENCERE_ORAN`.
**Yan etki (ölçüldü):** düz rota baseline CPA 2.53 m → **0.19/0.35 m** (biri
gerçek temas). Düzde iyileştirilecek alan pratikte kalmadı; açık elipste.

## MADDE 12 — DURAN/YAVAŞ HEDEFTE KAPANMA DURUYOR (2026-08-07)

Asılı hedef koşusunda **devir hiç olmadı**, menzil 150 m'de kaldı. Kök neden
görüntülü tarafta değil: `simple_guided_follow.py:~3380`
`d_allow = min(d_allow, cmd_speed_xy * budget_s + budget_margin_m)` —
`cmd_speed_xy` **HEDEFİN** hızı. Hedef durunca `d_allow = 0*2+20 = 20 m`,
ArduCopter iç P terimi ~3.4 m/s veriyor (hareketli hedefte 20 m/s idi).
İkinci bileşen: iki kopterli dünyada simtime oranı 0.44.
**Öneri (UYGULANMADI):** bütçeyi hedefin değil **avcının** hız kabiliyetiyle
ölçekle (`max(|slot_vel_xy|, |pursuer_vel_xy|)`) ya da uzak menzilde
`COMMAND_POSITION_BUDGET_MARGIN_M`'i menzille büyüt; yakın fazdaki 20 m
koruması (2026-07-31'de bilerek kondu) korunmalı.
**Hızlı tekrar:** `HEDEF_MESAFE=400 SURE=420`.

## ÇÖZÜCÜ BÜTÇESİ — SAHA UYARISI (2026-08-07)

`butce_kesti` kolonu eklendi ve ilk işinde bozulmayı gösterdi: sim koşularında
**%65-80**, `iter` p50 = 26 = `iterasyon_tavani`, `sure_ms` p95 ≈ 13 ms =
`sure_butcesi_ms`. Yani çözücü döngülerin çoğunda **yakınsadığı için değil
bütçe bittiği için** duruyor; komut optimal-altı. Simde vuruş yine de üretiliyor
(arıza kanıtı değil), ama Raspberry Pi 5'te daha kötü olacak. **Sahada ilk
bakılacak kolon budur.** Azaltıcılar hazır: `--ufuk` küçült, `sure_butcesi_ms`,
`--no-yaw`.

## DURUM İZLEME
| # | madde | durum | koşu | sonuç |
|---|---|---|---|---|
| 0 | replay harness | **BİTTİ** | offline | mpc_test kapalı-döngü tıkanmayı üretiyor: capraz/yanal 23 m'de ıska, kuyruk 1.6 m kapanır |
| 1 | kapanma (q_alan ↑) | **İKİ ROTADA DOĞRULANDI ✅✅** | mpcB_duzeltmeli_duz_224747 | zaman aşımı 2→0, kapanma 2×, CPA<3m 2→4; bedel: 0-5 m tazelik %60→%42 |
| **1b** | ödül şekli: alan vs KAREKÖK | BEKLİYOR (öncelik 1) | — | karekök kolu hiç ölçülmedi; teşvik 1/r³ → 1/r² yumuşar |
| **1c** | J literatür terimleri (PAMPC img-vel, ZEM) | BEKLİYOR (öncelik 1) | — | kadraj kaybı darboğazını hedefler |
| **7a** | çözücü KKT artığı metriği | BEKLİYOR (öncelik 2) | — | optimallik sertifikası YOK; iter tavanı %46-70 |
| 2 | geometri kapısı | BEKLİYOR | — | — |
| 3 | ufuk MENZİLLE ölçekleme | **SİMDE DOĞRULANDI ✅** | mpcB (1+3 birlikte) | tek başına sim'de ayrıştırılmadı; 1+3 paketi kazandı |
| **1+3** | **birlikte** | **SİMDE DOĞRULANDI ★✅** | A/B 22:37–22:55 | ISKA zaman aşımı 2→0, kapanma −2.25→−4.28 m/s, minR 1.76→1.30 m |
| 4 | kör terminal PN | BEKLİYOR | — | — |
| 5a | dikey ivme pitch | **[GİMBAL DALI: ÖLDÜ]** | — | pitch artık kamera eksenine girmiyor (gövde ±35° → kamera 0.65°) |
| **5b** | **dinamik TILT takibi** [YENİ] | BEKLİYOR | — | eps=asin(down/r) terminalde statik tilt'i aşıyor; gimbal 6 rad/s izliyor |
| 6 | pitch gecikme | **[GİMBAL DALI: KAPANDI]** | — | pitch kamera ekseninden çıktı; kalan iş beta'yı tilt'ten beslemek (ayar değil düzeltme) |
| 7 | çözücü bütçe | BEKLİYOR | — | — |
| 8 | bearing-angle TMA | **AÇIK** (kameradan çıkarım serbest) | — | offline prototip sırada; tavan yükseltici, gereklilik değil |
| 9 | ölçüm onarımı | BEKLİYOR | — | — |
| 10 | eyleyici modeli | BEKLİYOR | — | — |
| **11** | **dikey kanal FAZ C'den sonra kör** | **ÖLÇÜT SAĞLANDI ★ (DENEY-3)** — varsayılan hâlâ KAPALI | DENEY-1: 6, DENEY-2: 12, DENEY-3: ~14 koşu (2026-08-07) | `P+TGO k=2`: elips 2.02→0.41/0.15, düz 0.30→0.06, DOWN=0 0.18→0.09; **kampanyanın tek 3 fiziksel teması**; ISKA 10→2-4. DENEY-4 (wanderer + abort tanısı + DOWN=−3) koşuyor |
| **11b** | rota-özgülük: dikey artık yalnız DÖNEN hedefte | AÇIK (yeni) | düz vs elips 2026-08-07 | düz taban 0.13/0.30 m (ölçüt sağlanıyor), elips 2.02 m — hipotez: dönüşteki d_ey bozucusu PN'i besliyor, "paralel seyir" dikey ofseti kilitliyor |
| **11c** | `butce_kesti` %52-82 TÜM kollarda | **DENEY-5 koşuyor** | tüm 2026-08-07 koşuları | FISTA koşuların 2/3'ünde yakınsamadan çıkıyor — ölçülen her A/B farkının üstüne binen gürültü; `YILDIZ_COZUCU_BOL` (iter 26→40, bütçe 13→18 ms) test ediliyor |
| **11d** | k=2 vs k=3 seçimi | **AÇIK** (n≥3 gerekiyor) | elips+4 n=2/kol | konfigürasyon içi yayılım (0.19-2.78) hücreler arası farktan büyük; 11c kırılmadan karar verilemez |
| **13** | wanderer'da KAPANMA arızası (dikey değil) | **AÇIK (yeni)** | wanderer A/B 2026-08-07 | avcı 15.5 / hedef 15.0 m/s, hız kelepçesi %0 → MPC kapanmayı seçmiyor ("yan yana uç" nüksü, dönen hedefte); angajman 13-37 m'de kopuyor, ISKA 4/5'i "menzil açılıyor" |
| **14** | `irtifa_abort` referansı yanlış büyüklüğü ölçüyor | AÇIK (zararsız, hijyen) | 67 abort sınıflandırması | `simple_guided_follow.py:3171` konumlu slot z'sine bakıyor; görüntülü yetkideyken ve devir sonrası ~5 s silahsızlandırılmalı (2026-07-24'te LOS spear için yapılanın aynısı) |
| **15** | eps ölçüm gürültüsü terminal tabanı koyuyor | AÇIK | r<20 m karelerin %23-44'ü | bbox eps'i ile gerçek yükseliş farkı >2° (15 m'de 0.52 m); kabul eşiği 0.5 m tam bu tabanda — ölçüt sıkılaşacaksa önce bbox dikey merkezi düzeltilmeli |
| **12** | duran/yavaş hedefte kapanma duruyor | AÇIK (konumlu tarafta) | asılı hedef 2026-08-07 | `d_allow` hedefin hızıyla ölçekleniyor → 20 m kelepçe → 3.4 m/s |
| **16** | APN: hedef yanal ivmesini kestir + ufka yay (`YILDIZ_APN`) | **PARK — bayrak KAPALI kalacak** (kod duruyor) | apn1_elips_20260808_183316, apn1t_elips_20260808_185351 | `a_dik` TÜREV terimi GÜRÜLTÜ çıktı. Gerçek referans `v·ω`'dan türetildi (`ref_hedef_ax/ay` kolonları BOZUK, max 0.13 m/s²): kestirim-gerçek korelasyon **+0.05 / +0.11**; \|a_dik\| kestirim p50 **1.3-1.8** vs gerçek **0.01-0.02** m/s²; DÜZ fazda gerçek \|a\|>0.5 olan kare **%0.0** iken `apn_a` **%70-73** aktif; aktiflerin yalnız **%38-43'ü doğru işaretli** (yazı-turadan kötü). Kök neden: `v_dik` RMS hatası 6.5-8.8 m/s, 0.6 s LPF'li türevi ≈10 m/s² gürültü → ±6 kelepçeye dayanıyor. Birinci mertebe terim SAĞLAM (v_dik korelasyon +0.62/+0.75). Bayrak kapalıyken davranış BİT-AYNI (500 rastgele girdide `array_equal`, Gam/Hessian değişmiyor) |
| **17** | `tut` kolunda yaw sürdürme (`YILDIZ_TUT_YAW`) | **İZOLE TEST EDİLİYOR** | tyaw1_elips_20260808_190812 (+ tyawacc, düz regresyon) | apn1t'de APN ile karışıktı, ayrıştırılıyor. Mekanizma çalışıyor: `tut` karelerinin **%100'ünde** yaw komutu gidiyor (öncesi %0), yawsız kare DÜZ'de **%18.0 → %7.7**. Sönümlü (τ=1.0 s) + süre sınırlı (1.5 s), `suz` koluna dokunmuyor |
| **18** | WPNAV_ACCEL araca GEÇMEMİŞ (dosya 500, araç 250) | **ÖLÇÜLDÜ → restorasyon deneyi** | MAVLink 14551 param okuma 2026-08-08 | `params/swarm_copter.parm:55` **500** yazıyor, araçta **250** okundu; `WPNAV_ACCEL_Z` de 500→250. `WPNAV_SPEED` 3500 uyuyor. Overfit değil, kayıp parametrenin restorasyonu (bkz. memory "eeprom parm dosyasını eziyor") |
| **25** | TAKLA/DEPARTURE: kurtarma zamanlayıcıyla çıkıyordu → **HOLD irtifa kapısı UYGULANDI (default-ON)** | **UYGULANDI ✅ — uçuş doğrulaması sırada (kpn26c)** | kpn1/kpnduz2/kpn26/kpn26b (4 takla) vs 8 sağlam koşu, 2026-08-09 | **BULGU: 12/12 ayıran tek ölçü SÜREKLİ İRTİFA HATASI** — alt_err p95 **56-57 m** (taklalı) vs **19-33 m** (sağlam); alt_err>15 m oranı **%66-91 vs %9-18**. Bütün koşular bir noktada 15 m'yi aşıyor; fark aşmak değil **toparlanamamak**. **MEKANİZMA (kaçak döngü):** irtifa hatası → agresif dikey+yatay talep → büyük yatma (takla öncesi tilt **ortancası 67°**) → cos kaybı (54°'de dikey bileşen 0.59) → gaz zaten karelerin **%10-23'ünde TAM DOYGUN** (askı 0.35 iken ThO=1.00) → irtifa tutulamaz → hata büyür → departure. **KÖK NEDEN (kod):** `ALTITUDE ABORT` yatayı doğru kesiyor (`send=False, speed_cap=0`) **ama HOLD'dan çıkış YALNIZ ZAMANLAYICIYDI** (`hold_s`=2 s; uzatma sadece `spinning`). kpn26: t=25.8 HOLD alt_err 15.6 → t=30.8 REENGAGE **12.9** (13 m hatayla hız geri) → t=34.8 CHASE **20.2**; takla t=32.3. Ayrıca abort `:342`'de **kendini silahsızlandırıyor** (yeniden silahlanma `alt_err ≤ 3 m` ister, taklalı koşularda hiç olmuyor). **DÜZELTME:** `simple_guided_follow.py` HOLD çıkışına `irtifa_bozuk` şartı — `spinning` ile aynı desen, aynı `hold_max_s`=5 s tavanı, aynı fail-open, **yeni eşik yok** (`alt_abort_arm_m` zaten 'irtifa toparlandı' ölçütü). `YILDIZ_HOLD_IRTIFA=0` eski davranış. Birim sınaması: alt_err None/0.5/2.9 → 2.0 s (**eski ile birebir**), 3.1/20 → 5.0 s (uzatma, tavanda kilitli). **ÇÜRÜTÜLEN YAN HİPOTEZ:** MOT_THST_HOVER/EEPROM-silme suçlu DEĞİL — ThH her yerde 0.390-0.407 (2026-08-01 koşularında da), gerçek askı gazı 0.28-0.35, yani ThH zaten doğru; **0.68 assert'i eklenseydi ileri beslemeyi 2× şişirip zararlı olurdu**. **AÇIK ADAY (bugün YAPILMADI, 'ikinci düğme no-op' dersi):** `RECOVERY_HOLD_MAX_S` 5 → 8 s. Tek değişken ölçülmeden ikinci düğmeye dokunulmayacak |
| **24** | DİKEY AYRIM (dz<0): dört ayar düğmesi de ÇÜRÜTÜLDÜ → **YAPISAL tasarım turu gerekiyor** | **AÇIK — yeni tasarım turu** | eyl2, d0kpn, dv1 (+ kpn3b/kpn4 tabanı), 2026-08-08/09 | **DESEN:** CPA'da avcı hedefin ALTINDA — dz<0 **%76 (26/34)** düzeltilmiş KOR_PN havuzunda; ham havuzda ve DOWN=0'da da aynı. **DENENEN VE ÇÜRÜTÜLEN DÖRT DÜĞME:** **(1) dikey doyumlu eyleyici** (`YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5`, eyl2): abort/angajman 0.69→**1.11**, `|cmd_vz|` tavan %5.4→%20.3 — kötüleştirdi, varsayılanlar nötrlendi. **(2) DOWN=0 standoff** (d0kpn): |dz| tgoK20d0 bandına (0.04-0.11) İNMEDİ, tüm CPA |dz| p50 1.62 = değişmedi, **ve dz<0 yine 9/10** → alttan kaçırma standoff'tan GELMİYOR. **(3) dikey rampa penceresini öne alma** (`YILDIZ_DIKEY_RAMPA_BAS=80/SON=45`, dv1): **NO-OP olduğu KANITLANDI** — imza ölçümü (`dikey_hata`'nın ilk sıfırdan-farklı olduğu menzil) dv1 28.5 m vs kpn3b 29.6 / kpn4 26.1 m, yani DEĞİŞMEDİ; aritmetik sebep: `s=clip((BAS−r)/(BAS−SON),0,1)` ve **devir menzili R0 p50 ~25 m**, o noktada 45→25 penceresi de 80→45 penceresi de zaten **s=1.00 (tam doyum)**. Angajman, pencerenin doymuş ucunda başlıyor — rampa hiç limitleyici değilmiş. **(4) (1)+(3) paketi** (dv1): bekçi tetiklendi (abort/angajman 1.11 = eyl2 imzası), CPA |dz| p50 3.59 (havuzun en kötüsü), dz<0 %78. **KALAN KÖK NEDEN (analiz ajanı, hâlâ geçerli):** dikey kanal ölü zamanı ~1.5 s (ölçülen τ_z 2.16 s, model 1.0; CPA öncesi talep icrası %11) + eps bir **AÇI** olduğu için dz sabitken bile r küçüldükçe talep son saniyede doğup tavana vuruyor. **SONUÇ: bu ayar düğmesiyle çözülmez.** Aday yapısal yönler: (i) dikey için ayrı **lead/öngörü** terimi (talebi eps'ten değil, kestirilen dz ve t_go'dan üret — açısal doğum gecikmesini baypas eder); (ii) **devir menzilini büyütmek** — dikey işin tamamı ~25 m'de başlıyor, çünkü görüntülü yetkiyi orada alıyor; devir eşiği (~%6 kapsama) gevşerse dikey bütçe de büyür; (iii) dikey kanalı MPC ufkunda ayrı ağırlıkla ele almak. Hepsi yeni bir tasarım turu ister |
| **22** | KÖR SÜZÜLMEDE PN SÜRDÜRME (`YILDIZ_KOR_PN`) | **UYGULANDI + DÜZELTİLDİ; n artıyor** | kpn1b/kpn2/kpnduz (ham), kpn3b/kpn4 (R3′+menzil kapısı) | Kök neden: son 5 m'de döngülerin ~%78'i bbox'sız ve o körlükte **komut donuyordu** (`_kor_komut` son komutu aynen tekrarlıyor, çözücü hiç koşmuyor); oysa menzil kanalı bbox'tan bağımsız ve taze. Kol: ex/ey'yi içeriden **ölü hesapla ilerlet** (`c2=KDEG/r` kapanırken büyür) ve çözücüyü koşmaya devam ettir — komutu değil **yasayı** sürdür. Raylar R1 süre 1.0 s, R2 d_ex `exp(-t/0.7)` ile sıfıra sönüm, R3 `|ex0|+5°`, R4 menzil tazelik, R5 yaw'a dokunma. **HAM SÜRÜM DÖNÜŞTE REGRESYON YAPTI** (kapanma +3.87 → −7.93/−2.89/+0.28, taze %74→%46-58): ölü hesap hedefi kadraj dışına sürüklüyordu (kör \|ex\| MAX 28.3/46.9/36.4 vs kadraj yarı-açısı 20.07) çünkü R3'ün tabanı "son gerçek \|ex\|" idi ve dönüşte o zaten 30°+. **DÜZELTME (R3′ + menzil kapısı):** `\|ex\| ≤ min(\|ex0\|+5, kor_pn_ex_mutlak_deg=20)` (`YILDIZ_KOR_PN_EX_MUTLAK`) + `r_ic ≤ kor_pn_menzil_m=12` (`YILDIZ_KOR_PN_MENZIL`). Sonuç (kpn3b): kör \|ex\| MAX **20.0** (bir kez bile aşılmadı), kor_pn aktifliği %100→%52 ve kalanlar terminalde (r_ic p50 3.1 m), **DÖNÜŞ kapanma +2.36 / açılan %31.5** (pt1b'den bile iyi), ALTITUDE ABORT 6 (günün en düşüğü), `\|cmd_vz\|` tavan %4.2, tüm CPA \|dz\| p50 1.64. Bit-aynılık (bayrak kapalı) max\|fark\| **0.000e+00**; mpc_test bayraksız 86/86. **AÇIK BEDEL:** kpn3b'de sub-metre geçişler kayboldu (en yakın 0.73/0.75 → 1.47) — kpn4 replikasyonu bunu ayırt edecek. **AYAR ADAYI (yarın, koşulmadı):** mutlak kelepçeyi **20 → 26°** esnet (`YILDIZ_KOR_PN_EX_MUTLAK=26`). Gerekçe: eski kazançlar 20-28° bandındaki kör ilerletmeden geliyordu (kpn1b MAX 28.3 ile minR 0.73 yaptı); asıl zarar 30-47° kaçaklarıydı. 26 kaçağı hâlâ keser, terminaldeki meşru taşmalara izin verir. Menzil kapısını gevşetmek İKİNCİL aday |
| **23** | ÖLÇÜM ALTYAPISI: plan-uyum kapısı | **UYGULANDI ✅** | tools/plan_uyum.py + senaryo.sh:148-168 | **İKİ DENEME SESSİZCE YANDI**: `PLAN=` yalnızca `YENIDEN_BASLAT=1` dalında araca yükleniyor; `YENIDEN_BASLAT=0` (GUI'yi korumak için standart kuralımız) ile `PLAN` **hiçbir şey yapmıyor** ve hedef, yığın açılışındaki görevi (varsayılan elips) uçuyordu. Üstelik ekran `">>> hedef ucak: AUTO, duz rota"` yazıyordu — etiket `PLAN_AD`'den geliyordu, araca yüklenenden değil. Kurbanlar: `tyawaccduz_duz_20260808_193448` (dünkü "E", **"düz regresyon GEÇTİ" hükmü geri çekildi**) ve `kpnduz_duz_20260809_123228`; ikisi de ELİPS uçtu (hedef konum yayılımı 1177/696 ve 986/695 m; gerçekten düz koşularda D-B **4-8 m**). 16 eski düz koşu (b5dk*, dt*, mpc*, tgo*) SAĞLAM — onlar `YENIDEN_BASLAT=1` döneminden. Kapı: yüklü görevi MAVLink'ten indirip **geometriden** karşılaştırır (K-G kapsamı, D-B kapsamı, DO_JUMP) — waypoint sayısı kullanılmaz (yüklemede home item değişebiliyor). Uyumsuz → **EXIT 1** + çözüm satırı; `SENARYO_PLAN_KONTROL=0` atlar; okunamazsa uyarıp etiketi "(dogrulanmadi)" yapar. Canlı test: düz→1 UYUMSUZ, elips→0 UYUMLU |
| **21** | KESİŞME yaklaşımı (görüntülü faz için) | **RET** — düğme kalsın, varsayılan `slot` | KES1 (kes1_elips_20260808_231825), `YAKLASIM=kesisme` | **Ön şartı SAĞLADI ama temas ÜRETMEDİ — öğretici bir ret.** Kesişme devri gerçekten daha hızlı ve çapraz geliyor: kapanma0 p50 **6.92 m/s** (slot ~4.4), |aspekt0| p50 **135°** (slot ~171). Buna rağmen minR p50 **14.57 m** (slot kolları 6.4-12.6), <3m 1/14, en iyi 2.66 m, ve **4 angajman "menzil açılıyor" ile koptu** (slot'ta 0-1). Kesişme fren yapmadığı için görüntülü yasa aşımı toparlayamıyor. **ÇIKARIM: "yüksek devir kapanması → iyi temas" korelasyonu slot geometrisi İÇİNDE geçerliymiş; çapraz geometriyle nedensel olarak zorlanınca kırılıyor.** YAN KAZANIMLAR (kayda değer): (1) **düz-faz taze% 97.7 — kampanya rekoru** (slot 63.8-67.9), ama dönüşte tersi: taze %62.2, |ex| p50 21.1 (slot 7-11), kapanma −4.28 (açılıyor). (2) Konumlu faz TEK BAŞINA **CPA 1.33 m** yaptı (vibe max 5.80, temas yok) → darboğaz TERMİNAL DÖNÜŞÜM. (3) Aşım/toparlanma sağlıklı: 9 geçiş çevrimi, 8 yeniden angajman, aşım→yeni angajman p50 12.4 s; hedef arkada kalıp tur beklemiyor. ALTYAPI: `tools/senaryo.sh:187-199,208` **YAKLASIM passthrough** eklendi (boşsa bit-aynı; `--yaklasim` zaten `simple_guided_follow.py:1985`'te vardı). İLERİSİ: **hibrit "kesişme-yaklaş / slot-devir"** düşünülebilir — kesişmenin düz-faz tazelik ve kapanma avantajını alıp terminale slot geometrisiyle girmek; şimdi değil |
| **20** | DOYUMLU EYLEYİCİ modeli (`YILDIZ_EYLEYICI`) | **UYGULANDI, SİM'DE n=1 GEÇERLİ ÖLÇÜM** | eyl1_elips_20260808_202707 (eyl1b GEÇERSİZ: roll salınımı → 6× ALTITUDE ABORT → yere temas) | Kök neden ölçüldü: MPC `hiz_gecikme_tau_s`=1.00 s + **sınırsız ivme** varsayıyor; gerçek araç ivme sınırlı (yatay plato **~4 m/s²**), çalışılan bantta (|e|=10-20 m/s) fiili tau **5.5-6.8 s** → MPC ulaşılamaz komut planlıyor. Düzeltme: doyumun ardışık doğrusallaştırması, `tau_eff_k = max(tau_lin, |u_nom_k−w_nom_k|/a_max)`, tau_lin 1.7 s / a_max 4.0 m/s², **yalnız yatay** (dikey eski tau'da → `_cbf_sinirlari` dokunulmamış). Ölçülen etki: `u_doyum` dönüş %40.8→%14.9, düz %36.4→%11.0. Çözücü maliyeti **+%3.5** p50. Bit-aynılık: 400 girdide `array_equal`. **NOT: WPNAV_ACCEL 250↔500 platoyu DEĞİŞTİRMEDİ** — GUIDED hız-setpoint yolu PSC'den geçer, `WPNAV_*` waypoint kontrolcüsünün parametresidir. **DİKEY GENİŞLETME DENENDİ ve GERİ ALINDI (eyl2, 2026-08-08 gece-2):** yatay-yalnız uygulama dikey kanalı yapay olarak UCUZ bıraktı ve çözücü talebi oraya kaydırdı (`|cmd_vz|` p90 6.38→9.00 tavan, ALTITUDE ABORT/angajman 0.36→0.69). Dikey de ölçüldü (**tau_lin_z 2.16 s, plato 5.25 m/s² = WPNAV_ACCEL_Z 500**; yani dikey yataydan DAHA YAVAŞ, model onu 1.0 ile daha HIZLI sanıyordu) ve simetrik uygulandı — **ama İŞE YARAMADI, KÖTÜLEŞTİRDİ**: abort/angajman **1.11** (kampanya rekoru), `|cmd_vz|` tavanda kare oranı %5.4→**%20.3**. CBF suçlu değil (zorla-alçalma %7.6→%2.2'ye düştü, kutu açıldı, komut yine tavanda). **Çıkarım: dikey talebi kuran şey eyleyici modeli değil, maliyet/geometri tarafı (dikey standoff + eps kovalaması) — AYRI BİR TURUN işi.** Varsayılanlar nötrlendi (`tau_lin_z`=1.0, `a_max_z`=∞ ⇒ dikey doyum kapalı), yani `YILDIZ_EYLEYICI=1` = kanıtlı YATAY-YALNIZ kol; ölçülen değerler `YILDIZ_TAU_LIN_Z=2.2 YILDIZ_A_MAX_Z=5` ile açılır. **ÖLÇÜ TUZAĞI (kayda geçsin): `|u3|` dikey talebi ÖLÇMEZ** — u3, LOS üçayağındaki e3 bileşenidir ve `e3=(sin ε cos ex, sin ε sin ex, cos ε)`, ε büyüdükçe e3'ün yatay payı büyür. Dikey talep için **`cmd_vz` (NED)** kullan |
| **19** | 8 s ISKA zaman aşımı KAPANIRKEN kesiyor | **KAPANDI — uzayan süre TEMAS ÜRETMİYOR (2 senaryoda kanıt). Kod duruyor, bayrak varsayılan KAPALI** | H1b (olcag1b, saat kapalı) vs H2b (olcag2b, saat açık), 2026-08-08 | **Kol çalıştı ama HİÇ ATEŞLENEMEDİ: durgunluk saati p50 0.46 / p90 1.94 / MAX 4.14 s (eşik 8); 6 s üstü kare 0/1092.** H1b 2/13 → H2b 0/12 zaman aşımı farkı kola AİT DEĞİL, angajman süre dağılımından: H1b'de >8 s olan 4/13, H2b'de 1/12. **Karşı-olgu kolonu iki kolda da tam 0.00 m** (minR@8s → nihai; minR her zaman ilk 8 s içinde oluşuyor) — kesin kanıt. Neden: kolun kanıt tabanı tur-1'di (15 zaman aşımının 4'ü kapanırken kesilmiş), o koşular ESKİ profildeydi ve angajmanlar duvar saatine kadar koşuyordu; yeni profil+eyleyici kombinasyonunda angajmanlar 6-8 s'de kendiliğinden bitiyor, yani **hedeflenen patoloji bu kurulumda YOK**. Kol yanlış değil, senaryo onu sınamıyor. Tasarım notu: ilerleme ölçüsü **best-so-far iyileşme hızı** (monoton → salınıma bağışık); anlık ve pencereli ham menzil sürümleri `r=25+2·sin(2t)` ile kandırılıp atıldı **[KAPANIŞ, KES1 2026-08-08]** Kesişme senaryosunda kol ARTIK ATEŞLENDİ (durgunluk max 8.02 s, >6 s kare %3.4, bir ISKA `zaman asimi/durgunluk (8.0 s > 8; angajman 10.7 s)`) — yani "senaryo sınamıyor" mazereti kalktı. **Karşı-olgu kazancı YİNE 0.00 m** (uzayan 2/14: minR@8s 19.62→19.62, 22.53→22.53). İki bağımsız senaryoda (slot H2b + kesişme KES1) aynı sonuç: minR her zaman ilk 8 s içinde oluşuyor, uzayan süre temas bandına iniş sağlamıyor. Madde KAPANDI. | |
| 19a | (19'un tasarim/kanit kaydi) | — | tur-1 havuzu (taban8+apn1+apn1t, 34 angajman) + sentetik | KANIT TABANI: 15 zaman asiminin **4'u (%27)** arac HALA kapanirken kesilmisti (son-2s egim -4.11/-3.59/-2.11/-1.34 m/s; r = 8.8/21.6/13.6/25.6 m); kalan 11'inde menzil gercekten aciliyordu (+0.36...+7.31 m/s). SENTETIK DOGRULAMA: guclu/kesikli kapanma 8.1 -> 22.1 s uzuyor, durgun 8.05 s ve **salinimli-duragan 8.25 s**'de kesiliyor. YAN BULGU (madde 19'dan bagimsiz, hala gecerli): minR ile en guclu iliski **\|aspekt kaymasi devir->CPA\|**, korelasyon **-0.577** -- en yakin gecisler kuyruk takibinden degil, buyuk yaw ile hedefin yanina savrulup oradan kapamaktan geliyor |
| — | pencere 45→25 kare | **UYGULANDI ✅** | düz baseline 2026-08-07 | CPA 2.53 → 0.19/0.35 m; 20 Hz YOLO için de doğru süre |
| — | irtifa varyantı (down 0 / ±) | **ÖLÇÜLDÜ ✅** | 3 koşu 2026-08-07 | DOWN=0 en iyi (CPA 0.67 m); alttan takip dikey artık bırakıyor |
