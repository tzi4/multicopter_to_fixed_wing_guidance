# NOTLAR_TAKIP — ArduPilot FOLLOW yöntemi: testleri kendin koşmak

> Bu dosya **`ardupilot-takip` dalındaki** `guidance_allstar/takip_gudum.py`
> kolunun çalıştırma notudur. MPC kolu için: **`NOTLAR_MPC.md`**.
> Yöntemin tasarım gerekçesi, ArduPilot kaynak analizi ve ölçümler:
> **`ARDUPILOT_TAKIP.md`** (bu dosya onun "nasıl koşarım" özeti).
>
> Yığın, görev başlatma, montaj ve ortam düğmeleri **MPC ile birebir aynıdır** —
> yalnız görüntülü güdüm süreci değişir.
>
> **[GİMBAL DALI GÜNCELLEMESİ 2026-08-05]** Bu not `ardupilot-takip` dalı
> içindir; **`gimbal` dalında dikey geometri başkadır**. Orada kamera gövdeye
> sabit değil, kendini stabilize eden **fiziksel tek eksen (tilt) gimbalde**
> (gövde ±35° savrulurken kamera dünya pitch'i max 0.65°). "Montaj açısı" diye
> bir düğme yok; dikey ekseni `YILDIZ_TILT = atan(down/back)` belirler
> (`scripts/standoff_geom.sh`, araç `tools/tilt_ayarla.py`). Aşağıdaki
> `--back/--down` ve `--ofs-geri/--ofs-asagi` sayıları gövdeye-sabit dönemin
> değerleridir; iki dal birleşirse dikey kanal ve pitch'e dayanan her ölçüt
> yeniden türetilmeli. Ayrıntı: `NOTLAR_GIMBAL.md`.

```bash
# ============================ NEREDE ÇALIŞIR ====================
cd /path/to/savasan_iha_yildizlar_goruntulu_gudum     # TÜM komutlar depo kökünden
git checkout ardupilot-takip            # bu yöntem BU DALDA

# ============================ YIĞIN ============================
./yildizlar_gudum.sh --headless    # --iris ve --hummingbird modelleri için flag eklenmesi gerekmektedir.
# GUI istersen --headless'sız
./yildizlar_gudum.sh --stop

# ==================== ARAÇLARI GÖREVE SOK ======================
# --plan OPSİYONEL (varsayılan hedef_elips). KÖŞE PARANTEZ YAZMA.
python3 tools/gorev_baslat.py --drones 1 --drone-alt 60
python3 tools/gorev_baslat.py --drones 1 --drone-alt 60 --plan missions/hedef_duz.plan

# ======================= KONUMLU GÜDÜM =========================
cd guidance_allstar && python3 simple_guided_follow.py --no-kill-mode --yaw-lock --back 25 --down 6

# ================== GÖRÜNTÜLÜ GÜDÜM (TAKIP) ====================
# Konumluyla BİRLİKTE ayağa kalkar, Redis 'komut_yetkisi'='goruntulu'
# olana kadar komut GÖNDERMEZ.
cd guidance_allstar && python3 takip_gudum.py
# Portlar MPC ile AYNI (14654/14604) -> AYNI ANDA MPC VE TAKIP KOŞMAZ.

# ===================== TAM DENEME (tek komut) ==================
SURE=360 GORUNTULU="takip_gudum.py" PLAN=missions/hedef_sonsuz.plan tools/senaryo.sh

# ----- ablasyonlar (ARDUPILOT_TAKIP.md bölüm 4 ve 6) -----
GORUNTULU="takip_gudum.py --hiz-kaynagi p"    # saf mode_follow
GORUNTULU="takip_gudum.py --fren ap"          # AP 'yanında dur' freni
GORUNTULU="takip_gudum.py --yasa poscon"      # Copter >= 4.5 yön yasası
GORUNTULU="takip_gudum.py --ivme-sekil 0"     # şekillendirme kapalı
GORUNTULU="takip_gudum.py --iska-kaynak alan" # GÖRSEL HAKEM (menzilsiz ıska)

# ================= OFFLINE TESTLER (sim gerekmez) ==============
cd guidance_allstar && python3 takip_test.py       # 55/55
```

## Bu kolun kendine has düğmeleri

| düğme | ne yapar |
|---|---|
| `--yasa klasik\|poscon` | FOLLOW yön yasası sürümü |
| `--kp` | konum hatası kazancı |
| `--fren kapali\|ap\|menzil` | terminal fren politikası |
| `--hiz-kaynagi tavan\|p` | hız komutu kaynağı (tavan = vuruş kolu) |
| `--ivme-sekil` | ivme şekillendirme katsayısı |
| `--ofs-geri / --ofs-asagi` | standoff ofseti (MPC'nin back/down karşılığı) |
| `--yaw-p`, `--no-yaw` | yaw kanalı |
| `--iska-kaynak menzil\|alan` | ıska hakemi: telemetri menzili mi, bbox alanı mı |
| `--iska-zaman-kaynak duz\|ilerleme` | ilerleme saati (kapanırken zaman aşımı çalmaz) |
| `--iska-zaman-asimi` | ıska zaman aşımı [s] |

## Tanı logu

`guidance_allstar/logs/takip_tani_*.csv` — kolonlar **bilinçli olarak
`mpc_tani` ile örtüşür** (`durum`, `vurus`, `menzil`, `menzil_hizi`,
`en_iyi`, `cmd_*`, `vibe`, `vuruldu`), böylece iki yöntem aynı araçlarla
kıyaslanır: `tools/karsilastir.py`, `tools/deneme_ozeti.py`.

## Ortak tuzaklar (MPC ile aynı)

- **Görüntülü süreci başlatmayı unutma.** Unutursan ölü-adam anahtarı
  (`goruntulu_hayatta`) geçişi engeller ve bbox.log'a uyarı düşer. Bu kapı
  olmadan araç komutsuz kalıyordu (2026-08-05 arızası, `mpc_memory.md`).
- **Aynı anda tek görüntülü yöntem** — MPC ve TAKIP aynı portları kullanır.
- **Koşu sırasında MAVLink portlarına ikinci istemci bağlanmaz.**
- **Aynı anda tek simülasyon**; koşuları sıraya koy.
- Koşu sağlığı: `simtime_ratio` 1.00 olmalı; "SIMULASYON GERIDE" = koşu geçersiz.
