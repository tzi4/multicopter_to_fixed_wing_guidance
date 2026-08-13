# mpc_memory.md — MPC vuruş yasası: kalıcı hafıza (2026-08-05 oturumu)

Bu dosya "ne öğrendik + hangi kararı neden aldık" hafızasıdır. Yanındaki iki
dosya: `TO_TEST.md` (deney planı + durum tablosu, CANLI) ve `DEVAM.md` (tur-1..4
kampanya güncesi). Log okuma: `guidance_allstar/LOG_SOZLUGU.md`. Ortamı sürme:
`NOTLAR_MPC.md`.

---

## 1. KÖK NEDEN (neden her zaman çarpmıyor) — üç katmanlı zincir

Tek hata değil; her tur bir halkayı kapattı, sonraki göründü, **toplam kapanma
monoton eridi** (ort. menzil hızı −3.47 → −0.68 m/s, TUR1→TUR4).

1. **KAPANMA PRİM OLARAK KODLANMIŞ (kök).** İleri hız kanalında seviye cezası
   bilinçli 0 (`mpc_gudum.py:1586` `r_lvl=[0.0,...]`) ama `ivme_kos` `‖u−w‖²`
   demiri ileri kanala da uygulanıyor (`:1620`), kapanmayı isteyen tek terim
   (lineer bbox alan ödülü) `/N`=20'ye bölünüyor (`:2018`). Sonuç: **"hedefle yan
   yana uç" maliyetin geçerli çözümü.** Kanıt: 24-32 m'de u1 medyanı TUR1 +34.7 →
   TUR4 −2.63; 12 ISKA zaman aşımının 12'si bu bantta; kerteriz-rota açısı ≥45°
   ile devralan 11 segmentin 11'i ıska.
2. **Ufkun çoğu çarpışma SONRASINDA.** Ufuk sabit 2.4 s, t_go medyanı 0.61 s;
   VURUŞ'ta ufkun ort. %73'ü olmayan gelecekte. Kısıt ufku menzille ölçekleniyor
   (`:1767`), maliyet ufku ölçeklenmiyordu → 5x asimetri.
3. **Terminal körlük tasarımca üst üste biniyor.** Son 5 m'de döngülerin %78'i
   kör; dört kapı aynı fazda (bozucu donması r<15 m, sert FOV bırakma, kör
   süzülme, iskelet tut/süz'de MPC hiç çağrılmaz ~⅓ döngü).

### İki yanlış inanç düzeltildi (orkestratör kendi kontrolü)
- **"FOV %96.8 bağlayıcı" YANLIŞ.** `fov_serbest=0` = "kısıt UYGULANDI", bağlayıcı
  değil (`:2185`; `LOG_SOZLUGU.md:263` yanlış tanımlı). Bu metrikle alınan ayar
  kararları şüpheli.
- **Çözücünün loglanan anı tek-atış yeniden çözmesi GÜVENİLMEZ** (sıcak başlatma
  loglanmıyor; soğuk çözüm log komutunu 38 m/s tutturamadı). Çözücü deneyleri
  KAPALI-DÖNGÜ replay ile → `mpc_test.py` `Benzetim`/`senaryo_kos`.

---

## 2. OFFLINE DENEY ALTYAPISI (madde 0 — BİTTİ)

`mpc_test.py` içinde `class Benzetim` + `senaryo_kos()` TAM kapalı-döngü:
nokta-kütle avcı + gerçek sanal gimbal + hedef + BİREBİR eyleyici zinciri
(LPF τ=0.35 → |v|≤35 kelepçe → ivme-sınırlı hız döngüsü WPNAV_ACCEL 5 m/s²).
Hız tavanı `cevre_hiz_tavani()`=35. Devir geometrileri hazır: `kuyruk`(r0=30,β=0),
`capraz`(45,40°), `yanal`(55,80°). Metrikler: `min_menzil`, `bosa_gudum_s`,
`iska_sebep`, `tavan_degme_pct`, `pitch_hizi_med`, `kayip_dongu`, `bitis`.

**Tıkanma offline'da ÜRETİLİYOR** (madde 0 kabul): baseline'da capraz min 30.7 m
(duz)/18.1 m (elips), yanal 26.3 m — hepsi ISKA zaman aşımı; kuyruk 1.6 m'ye
kapanır. Sim'deki tabloyla birebir.

Deney yöntemi: `MpcCozucu.__init__/coz/_adim_sureleri`'ni bellek-içi monkeypatch
(ORTAK DOSYAYA YAZMADAN — başka ajan sim'de). Kabul için ≥8 tohum: sonuçlar
iki-kutuplu (ya ~2 m kapanır ya ~23 m takılır), az tohumda medyan yanıltıcı;
"kapanma oranı" (min<10 m %) daha sağlam metrik.

---

## 3. DOĞRULANMIŞ DÜZELTMELER (offline; sim bekliyor)

### Madde 1 — bbox ALAN ödülünü büyüt (`q_alan ×3-4`)
İlk hipotez (u1'i ivme demirinden muaf tut) ÇÜRÜDÜ — tek başına kapanmayı kırmaz
(min 23→13 m, TO %75 sabit, kadraj kaybı artar). **Doğru kol: `q_alan`** —
kapanmayı menzilden değil BBOX ALANINDAN sürüyor (kullanıcı felsefesiyle birebir).
Diz noktası ×3-4: tıkanan kapanma %25→%75, TO %75→%25, **kuyruk bozulmadı**.
×6'da min menzil geri yükseliyor (delip geçme). Yer: `MpcAyar.q_alan`, `:2018`.

### Madde 3 — maliyet ufkunu MENZİLLE ölçekle (t_go DEĞİL)
`mpc_gudum.py:1765` notu: t_go ölçeklemesi zaten denenmiş, çapraz geometride
etkisiz (kapanma~0 → t_go∞). Menzille ölçekle (ref≈60 m, `_adim_sureleri`'de
`adim_s *= clip(r/ref, taban, 1)`). Terminal tekrarlanabilirliği DÜZELTMEDİ;
gerçek mekanizma: yakın menzilde ufku kısaltmak MPC'yi miyop yapıp anlık alan
ödülünü öne çıkarıyor, "yan yana uç" uzun-ufuk dengesini bozuyor. Tıkanan
kapanma %25→%75, TO %75→%25.

### ★ 1+3 BİRLİKTE (2×2 faktöriyel, n=32/grup) — TOPLANIYORLAR
| kol | çapraz kapanma% | TO% | kadraj kaybı% | kuyruk kapanma% |
|---|---|---|---|---|
| baseline | 25 | 75 | 18.3 | 67 |
| q_alan ×4 | 75 | 22 | 19.6 | 67 |
| ufuk ref=60 | 75 | 25 | 23.8 | 67 |
| **1+3** | **100** | **0** | **29.3** | 67 |

Bitiş sebebi (asıl kanıt): baseline çapraz **çarpışma 0/32** (24'ü "yan yana
uçtum pes ettim"); 1+3 → **çarpışma 7/32**. Kuyruk her iki kolda 16/16 çarpışma
(bozulmadı), yer teması yok (min irtifa 47.2 m). **Bedel: kadraj kaybı bitişi
8→16** — regresyon değil, angajman bedeli, ama YENİ DARBOĞAZ → madde 5+6 hedefi.

**OFFLINE→SİM BOŞLUĞU:** min-menzil/zaman aşımı güvenilir (yatay kapanma offline
iyi modelleniyor). Kadraj kaybı/titreme az güvenilir (offline motor terminale
düzleşmiş giriyor). Sim'de doğrula.

---

## 4. KURALIN NET HALİ (kullanıcı, 2026-08-05)

"Gözüyle görüp ÇIKARIM YAPABİLİR; telemetriden şimdilik SADECE RANGE al (düşük
güvenle)." → **Kameradan türetilen her şey serbest** (bearing, bbox boyutu, hedef
kinematiği kestirimi dahil). Yasak: telemetriye/yerden tespite yaslanmak. Yerden
tespit sistemi AYRI BRANCH'te ele alınacak. Birincil sinyal bbox alanı.
→ Bearing-angle TMA (madde 8) KURALA UYGUN, rafa kaldırılmadı; ama madde 1
kapanmayı bunsuz çözdüğü için TMA artık gereklilik değil, TAVAN YÜKSELTİCİ.

DİKKAT: bu kuralı bir ara fazla dar yorumlayıp TMA'yı yanlışlıkla rafa
kaldırdım; kullanıcı düzeltti. Kuralı daraltmadan önce sor.

---

## 5. SIRADAKİ (bkz. TO_TEST.md durum tablosu)

- **SİM KUYRUĞU (başka ajan bitince):** `q_alan ×4 + ufuk ref=60` A/B. İki satır
  kod, geri alması kolay. Kabul: çapraz duvar kırılıyor mu, kadraj kaybı/titreme
  baseline üstüne çıkmıyor mu.
- **OFFLINE DEVAM:** madde 5 (dikey ivmeyle pitch telafisi: 5.8°/(m/s²)≡1/g,
  `a_yukarı`+3 → burun 27→21°) + madde 6 (beta'daki 0.30 s pitch gecikmesi).
  İkisi de yeni darboğazı (kadraj kaybı) hedefler, saf güdüm kodu, offline test
  edilir.
- Sonra: madde 4 (kör terminal PN), madde 7 (çözücü bütçe), madde 2 (geometri
  kapısı — ORTAK DOSYA, sim gerekir), madde 9/10.

## 6. ORTAM/PROSEDÜR NOTLARI (bu oturumda düzeltilenler)
- `tools/gorev_baslat.py` standoff ipucu düzeltildi: eski YILDIZ_MOUNT=30 kopyası
  `--down 13` öneriyordu; artık `standoff_geom.sh` kaynaklanıyor → `--down 4`.
- NOTLAR_MPC.md (eski NOTLAR.md): yol bloğu (her şey depo kökünden) + `[--plan]` köşe-parantez uyarısı
  (opsiyonel gösterimi, komutun parçası değil).
- Yerdeki yaw: SIMSTATE disarm 0.00 sabit; drone yerde DÖNMÜYOR, kalkışta ±6°
  normal quad davranışı. Model legacy `drone_with_camera` ile fiziksel birebir.
