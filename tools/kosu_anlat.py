#!/usr/bin/env python3
"""kosu_anlat.py - bir kosunun loglarini INSAN DILINDE zaman cizelgesine cevirir.

NICIN VAR (2026-08-05): loglarimiz "hangi sayilar vardi"yi anlatiyordu, "ne
oldu"yu degil. Kullanici bir videoda "hedef bana dogru geliyordu ama MPC
kaciyor gibi" dedi; bunu dogrulamak icin hedefin gidis yonu ile kerteriz
arasindaki aci ELLE hesaplanmak zorunda kalindi, cunku hicbir logda
"bu karsilasma kuyruk takibi mi, kafa kafaya mi" yazmiyordu. Bu arac o
soruyu -- ve yanindaki on soruyu -- saniyeler icinde cevaplar.

KULLANIM:
    python3 tools/kosu_anlat.py run/denemeler/mpc_duz_20260804_191600
    python3 tools/kosu_anlat.py --goruntulu guidance_allstar/logs/goruntulu_mpc_*.csv
    python3 tools/kosu_anlat.py <klasor> --tam     # her olayi yaz (ozetleme)

NE YAPAR
  1. Kosuya ait dort logu bulur ve ORTAK ZAMANA hizalar:
       goruntulu_<metot>_<damga>.csv   (goruntulu gudum, t = time.monotonic())
       guided_follow_<damga>.csv       (konumlu gudum, wall_time = monotonic)
       mpc_tani_<damga>.csv            (MPC ic tanilari, t = monotonic)
       bbox.log                        (dedektor metni, ZAMAN DAMGASI YOK)
     Ilk uc dosya AYNI CLOCK_MONOTONIC saatini kullanir (Linux'ta bu saat
     surecler arasinda ortaktir), yani dogrudan hizalanirlar. Yeni goruntulu
     CSV'lerde ayrica t_unix vardir; o da mutlak saate baglanmayi saglar
     (bbox.log ve video dosyalari icin tek koprü budur).
  2. Hedefin durumunu (konum/hiz/ivme) ve karsilasma geometrisini cikarir.
     Yeni CSV'lerde bunlar ref_* kolonlarinda HAZIR durur. Eski kosularda
     yoktur; o zaman konumlu logdan (meas_x/y/z) turetilir -- bu sayede arac
     GECMIS kosularda da calisir.
  3. Zaman cizelgesini Turkce cumlelerle basar.

ONEMLI KURAL HATIRLATMASI: ref_* kolonlari ve bu aracin urettigi her sey
YALNIZ ANALIZ icindir. Gudum hedefin telemetrisinden yalniz MENZILI kullanir
(kullanici kurali, 2026-08-03). Bu arac gudume hicbir sey geri beslemez.
"""

import argparse
import csv
import glob
import math
import os
import re
import sys
from datetime import datetime
from pathlib import Path

KOK = Path(__file__).resolve().parent.parent
LOG_DIZIN = KOK / 'guidance_allstar' / 'logs'
ANSI = re.compile(r'\x1b\[[0-9;]*m')

# Karsilasma tipi esikleri -- goruntulu_temel.py ile AYNI olmali.
TIP_KAFA_KAFAYA_DEG = 60.0
TIP_KUYRUK_DEG = 120.0

TIP_METIN = {
    'kafa_kafaya': 'KAFA KAFAYA',
    'kuyruk': 'KUYRUK TAKIBI',
    'capraz': 'CAPRAZ',
    'durgun': 'HEDEF DURGUN',
    '': 'bilinmiyor',
}


# --------------------------------------------------------------- yardimcilar

def _f(satir, anahtar):
    """CSV hucresini float'a cevirir; bos/bozuk/eksik ise None."""
    ham = satir.get(anahtar, '')
    if ham in ('', None):
        return None
    try:
        d = float(ham)
    except (TypeError, ValueError):
        return None
    return d if math.isfinite(d) else None


def _oku(yol):
    with open(yol, newline='') as f:
        return list(csv.DictReader(f))


def _yon(deg):
    """Dereceyi pusula adiyla birlikte yazar: '090 (dogu)'."""
    if deg is None:
        return '---'
    adlar = ['kuzey', 'kuzeydogu', 'dogu', 'guneydogu',
             'guney', 'guneybati', 'bati', 'kuzeybati']
    return f"{deg:03.0f} ({adlar[int((deg % 360) / 45 + 0.5) % 8]})"


def _tip_belirle(yaklasim_deg, hedef_hiz):
    if hedef_hiz is not None and hedef_hiz < 1.0:
        return 'durgun'
    if yaklasim_deg is None:
        return ''
    if yaklasim_deg < TIP_KAFA_KAFAYA_DEG:
        return 'kafa_kafaya'
    if yaklasim_deg > TIP_KUYRUK_DEG:
        return 'kuyruk'
    return 'capraz'


def _sure(s):
    return f"{s:6.1f} s"


# ------------------------------------------------------------- dosya bulucu

def dosyalari_bul(deneme_dir, goruntulu_arg=None):
    """Kosuya ait log dosyalarini bulur. Doner: dict."""
    d = {'deneme': deneme_dir, 'goruntulu': None, 'konumlu': None,
         'mpc_tani': None, 'bbox': None, 'olay': None}

    if goruntulu_arg:
        d['goruntulu'] = Path(goruntulu_arg)
    elif deneme_dir is not None:
        # EN GUVENILIR BAG: goruntulu surec kendi CSV yolunu stdout'a basar
        # ve senaryo.sh bunu goruntulu.log'a yonlendirir. Ad tahmini yapma.
        gl = deneme_dir / 'goruntulu.log'
        if gl.exists():
            m = re.search(r'\] log: (\S+\.csv)',
                          ANSI.sub('', gl.read_text(errors='replace')))
            if m and Path(m.group(1)).exists():
                d['goruntulu'] = Path(m.group(1))
        if d['goruntulu'] is None:
            # Yedek: klasor adindaki damgadan SONRAKI ilk goruntulu CSV.
            m = re.search(r'([a-z0-9]+)_.*_(\d{8}_\d{6})$', deneme_dir.name)
            if m:
                metot, damga = m.group(1), m.group(2)
                aday = sorted(LOG_DIZIN.glob(f'goruntulu_{metot}_*.csv'))
                sonra = [a for a in aday
                         if re.search(r'(\d{8}_\d{6})', a.name).group(1) >= damga]
                if sonra:
                    d['goruntulu'] = sonra[0]

    if d['goruntulu'] is not None:
        m = re.search(r'(\d{8}_\d{6})', d['goruntulu'].name)
        if m:
            tani = LOG_DIZIN / f"mpc_tani_{m.group(1)}.csv"
            if tani.exists():
                d['mpc_tani'] = tani
        olay = Path(str(d['goruntulu']).replace('.csv', '_olay.csv'))
        if olay.exists():
            d['olay'] = olay

    if deneme_dir is not None and (deneme_dir / 'bbox.log').exists():
        d['bbox'] = deneme_dir / 'bbox.log'

    # Konumlu log: monotonic araliklari ORTUSEN aday secilir (ad tahmininden
    # daha saglam; iki kosu ayni dakikada baslamis olabilir).
    if d['goruntulu'] is not None:
        d['konumlu'] = _ortusen_konumlu(d['goruntulu'])
    if d['konumlu'] is None and deneme_dir is not None:
        # Goruntulu surec hic CSV uretmemis olabilir (or. pid_elips
        # 20260803_175339: HOME_POSITION alinamadi ve surec dustu). Kosu yine
        # de anlatilabilir -- konumlu faz tek basina cok sey soyler.
        d['konumlu'] = _damgadan_konumlu(deneme_dir)
    return d


def _damgadan_konumlu(deneme_dir):
    """Klasor adindaki damgadan sonra baslayan ILK konumlu logu sec."""
    m = re.search(r'(\d{8}_\d{6})$', deneme_dir.name)
    if not m:
        return None
    damga = m.group(1)
    aday = []
    for y in sorted(LOG_DIZIN.glob('guided_follow_*.csv')):
        d2 = re.search(r'(\d{8}_\d{6})', y.name)
        if d2 and d2.group(1) >= damga:
            aday.append((d2.group(1), y))
    return aday[0][1] if aday else None


def _zaman_araligi(yol, anahtar):
    try:
        with open(yol, newline='') as f:
            r = csv.DictReader(f)
            ilk = son = None
            for satir in r:
                v = _f(satir, anahtar)
                if v is None:
                    continue
                if ilk is None:
                    ilk = v
                son = v
        return (ilk, son) if ilk is not None else None
    except Exception:
        return None


def _ortusen_konumlu(goruntulu_yol):
    g = _zaman_araligi(goruntulu_yol, 't')
    if g is None:
        return None
    en_iyi, en_iyi_ortusme = None, 0.0
    for aday in sorted(LOG_DIZIN.glob('guided_follow_*.csv')):
        k = _zaman_araligi(aday, 'wall_time')
        if k is None:
            continue
        ortusme = min(g[1], k[1]) - max(g[0], k[0])
        if ortusme > en_iyi_ortusme:
            en_iyi, en_iyi_ortusme = aday, ortusme
    return en_iyi


# ------------------------------------------------------------ zaman hizalama

class Hizalayici:
    """Farkli loglari ORTAK zamana oturtur.

    t_mono : time.monotonic() -- goruntulu CSV 't', konumlu CSV 'wall_time',
             mpc_tani CSV 't'. Linux'ta CLOCK_MONOTONIC surecler arasi ORTAK,
             o yuzden bu uc dosya dogrudan hizalanir.
    t_unix : mutlak saat. Yeni goruntulu CSV'de kolon olarak var; eski
             kosularda dosyanin degistirilme zamanindan geriye dogru
             kestirilir (+-1 s hassasiyet; video/bbox eslemesi icin yeter).
    t_rel  : kosunun BASINDAN itibaren saniye (cikti bunu kullanir).
    """

    def __init__(self, t0_mono, mono_to_unix_ofset):
        self.t0 = t0_mono
        self.ofset = mono_to_unix_ofset     # unix = mono + ofset

    def rel(self, t_mono):
        return None if t_mono is None else t_mono - self.t0

    def unix(self, t_mono):
        if t_mono is None or self.ofset is None:
            return None
        return t_mono + self.ofset

    def saat(self, t_mono):
        u = self.unix(t_mono)
        return '--:--:--' if u is None else \
            datetime.fromtimestamp(u).strftime('%H:%M:%S')


def hizalayici_kur(gor_satir, kon_satir, gor_yol):
    """Kosunun t0'ini ve monotonic->unix ofsetini belirler."""
    mono_baslar = []
    if kon_satir:
        v = _f(kon_satir[0], 'wall_time')
        if v is not None:
            mono_baslar.append(v)
    if gor_satir:
        v = _f(gor_satir[0], 't')
        if v is not None:
            mono_baslar.append(v)
    t0 = min(mono_baslar) if mono_baslar else 0.0

    ofset = None
    for s in gor_satir[:50]:                      # YENI format: kolon var
        tm, tu = _f(s, 't'), _f(s, 't_unix')
        if tm is not None and tu is not None:
            ofset = tu - tm
            break
    if ofset is None and gor_satir and gor_yol:   # ESKI format: mtime'dan
        son = _f(gor_satir[-1], 't')
        if son is not None:
            ofset = os.path.getmtime(gor_yol) - son
    return Hizalayici(t0, ofset)


# ------------------------------------------------- hedef durumu + geometri

def _turetilmis_hedef(kon_satir):
    """Konumlu logdan hedefin hizini SAYISAL TUREVLE cikarir.

    ESKI kosular icin. Konumlu log hedefin OLCULEN konumunu (meas_x/y/z)
    yaziyor ama hizini yazmiyor; hiz ~0.5 s'lik merkezi farkla bulunur.
    Doner: [(t_mono, pos3, vel3|None), ...]
    """
    ham = []
    for s in kon_satir:
        t = _f(s, 'wall_time')
        p = [_f(s, k) for k in ('meas_x', 'meas_y', 'meas_z')]
        if t is None or any(v is None for v in p):
            continue
        ham.append((t, p))
    cikti = []
    for i, (t, p) in enumerate(ham):
        v = None
        j = i
        while j > 0 and t - ham[j][0] < 0.5:
            j -= 1
        k = i
        while k < len(ham) - 1 and ham[k][0] - t < 0.5:
            k += 1
        dt = ham[k][0] - ham[j][0]
        if dt > 0.05:
            v = [(ham[k][1][a] - ham[j][1][a]) / dt for a in range(3)]
        cikti.append((t, p, v))
    return cikti


def geometri(pos, vel, hpos, hvel):
    """goruntulu_temel._karsilasma_geometrisi'nin bagimsiz kopyasi.

    Ayni tanimlar (isaret konvansiyonlari LOG_SOZLUGU.md'de): yaklasim_deg
    0 = hedef tam uzerimize geliyor, 180 = biz onun kuyrugundayiz.
    """
    g = {'menzil_m': None, 'kerteriz_deg': None, 'yukselis_deg': None,
         'yaklasim_deg': None, 'tip': '', 'kapanma_mps': None, 'tgo_s': None,
         'cpa_m': None, 'cpa_s': None}
    if pos is None or hpos is None:
        return g
    r = [hpos[i] - pos[i] for i in range(3)]
    menzil = math.sqrt(sum(x * x for x in r))
    g['menzil_m'] = menzil
    yatay = math.hypot(r[0], r[1])
    g['kerteriz_deg'] = math.degrees(math.atan2(r[1], r[0])) % 360.0
    g['yukselis_deg'] = math.degrees(math.atan2(-r[2], max(yatay, 1e-6)))
    if vel is None or hvel is None or menzil < 1e-6:
        return g
    u = [x / menzil for x in r]
    vb = [hvel[i] - vel[i] for i in range(3)]
    g['kapanma_mps'] = -sum(vb[i] * u[i] for i in range(3))
    if g['kapanma_mps'] > 0.1:
        g['tgo_s'] = menzil / g['kapanma_mps']
    n2 = sum(x * x for x in vb)
    if n2 > 1e-6:
        tc = -sum(r[i] * vb[i] for i in range(3)) / n2
        if tc < 0.0:
            g['cpa_s'], g['cpa_m'] = 0.0, menzil
        else:
            g['cpa_s'] = tc
            g['cpa_m'] = math.sqrt(sum((r[i] + vb[i] * tc) ** 2 for i in range(3)))
    hh = math.sqrt(sum(x * x for x in hvel))
    if hh < 1.0:
        g['tip'] = 'durgun'
        return g
    kos = sum((hvel[i] / hh) * (-u[i]) for i in range(3))
    g['yaklasim_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, kos))))
    g['tip'] = _tip_belirle(g['yaklasim_deg'], hh)
    return g


def yetki_araliklari(gor, bosluk_s=1.0):
    """Goruntulu gudumun yetkili oldugu [bas, son] zaman araliklari.

    KRITIK: iki surec AYNI ANDA log yazar -- konumlu surec yetkiyi kaybettikten
    sonra da olcup loglamaya devam eder (yalniz setpoint gondermeyi keser).
    Bu yuzden iki CSV'yi zamana gore ic ice dizmek "her dongude yetki el
    degistiriyor" yanilsamasi verir. Dogru kural: GORUNTULU CSV yalniz yetkili
    oldugunda satir yazar; satirlar arasindaki 1 s'den buyuk bosluk yetkinin
    konumluya dondugu anlamina gelir (karsilastir.py ile ayni tanim).
    """
    araliklar = []
    bas = onceki = None
    for s in gor:
        t = _f(s, 't')
        if t is None:
            continue
        if onceki is None or t - onceki > bosluk_s:
            if bas is not None:
                araliklar.append((bas, onceki))
            bas = t
        onceki = t
    if bas is not None:
        araliklar.append((bas, onceki))
    return araliklar


class Ornekler:
    """Kosunun her aninda 'ne oluyordu' sorusunu cevaplayan birlesik tablo.

    Her ornek bir sozluk: t (mono), kaynak ('konumlu'/'goruntulu'), menzil,
    geometri, kadraj bilgisi, durum. Yetkili olan hangi surecse o anin
    ornegi ONDAN alinir (bkz. yetki_araliklari).
    """

    def __init__(self, gor, kon, hiz):
        self.hiz = hiz
        self.g = gor
        self.k = kon
        self.araliklar = yetki_araliklari(gor)
        self.ornek = []
        self._birlestir()
        self._menzil_duzelt()

    def _yetkide(self, t):
        for bas, son in self.araliklar:
            if bas <= t <= son:
                return True
        return False

    def _birlestir(self):
        # Eski format tespiti: ref_hedef_x kolonu yoksa hedefi konumludan turet
        yeni_format = bool(self.g) and 'ref_hedef_x' in self.g[0]
        # Hedefin hizi konumlu logda YOK (yalniz olculen konumu var). KONUMLU
        # faz icin her zaman turetilir; GORUNTULU faz icin yalniz eski
        # kosularda (yeni CSV'de ref_hedef_v* hazir).
        turetilmis = _turetilmis_hedef(self.k)

        for s in self.k:
            t = _f(s, 'wall_time')
            if t is None or self._yetkide(t):
                continue          # o anda yetki goruntuludeydi: onu kullan
            self.ornek.append(self._konumlu_ornek(t, s, turetilmis))

        for s in self.g:
            t = _f(s, 't')
            if t is None:
                continue
            self.ornek.append(self._goruntulu_ornek(t, s, yeni_format,
                                                    turetilmis))
        self.ornek.sort(key=lambda o: o['t'])

    def _menzil_duzelt(self):
        """Menzili ve kapanma hizini DUZGUNLESTIR.

        Menzil iki ayri kaynaktan gelir (konumlu: true_range_m; goruntulu:
        IMM menzili) ve ikisi de gurultuludur. Kapanma hizini anlik hiz
        vektorlerinden degil, ~0.5 s'lik menzil farkindan hesaplamak hem
        gurultuyu keser hem de iki kaynagi tutarli kilar -- 'en yakin gecis'
        gibi isaret degisimine bakan olaylar aksi halde onlarca kez tetikler.
        """
        for i, x in enumerate(self.ornek):
            if x['menzil'] is None:
                x['menzil_duz'] = x['kapanma_duz'] = None
                continue
            j, k = i, i
            while j > 0 and x['t'] - self.ornek[j]['t'] < 0.3:
                j -= 1
            while k < len(self.ornek) - 1 and self.ornek[k]['t'] - x['t'] < 0.3:
                k += 1
            pencere = [y['menzil'] for y in self.ornek[j:k + 1]
                       if y['menzil'] is not None]
            x['menzil_duz'] = (sorted(pencere)[len(pencere) // 2]
                               if pencere else x['menzil'])
            r0, r1 = self.ornek[j]['menzil'], self.ornek[k]['menzil']
            dt = self.ornek[k]['t'] - self.ornek[j]['t']
            x['kapanma_duz'] = ((r0 - r1) / dt
                                if None not in (r0, r1) and dt > 0.05 else None)

    def _konumlu_ornek(self, t, s, turetilmis):
        pos = [_f(s, k) for k in ('pursuer_x', 'pursuer_y', 'pursuer_z')]
        vel = [_f(s, k) for k in ('pursuer_vx', 'pursuer_vy', 'pursuer_vz')]
        hpos = [_f(s, k) for k in ('meas_x', 'meas_y', 'meas_z')]
        pos = None if any(v is None for v in pos) else pos
        vel = None if any(v is None for v in vel) else vel
        hpos = None if any(v is None for v in hpos) else hpos
        _, hvel = self._en_yakin_hedef(t, turetilmis)
        g = geometri(pos, vel, hpos, hvel)
        if g.get('kapanma_mps') is None:
            g['kapanma_mps'] = _f(s, 'closing_velocity')
        if g.get('tgo_s') is None:
            g['tgo_s'] = _f(s, 't_go_s')
        return {
            't': t, 'kaynak': 'konumlu',
            'menzil': _f(s, 'true_range_m') or _f(s, 'range_m'),
            'geo': g,
            'pos': pos, 'vel': vel, 'hpos': hpos, 'hvel': hvel,
            'ex': None, 'ey': None, 'kapsama': None, 'durum': '',
            'kurtarma': s.get('recovery_state', ''),
            'vibe': _f(s, 'vibe_max'),
            'kelepce': {},
        }

    def _goruntulu_ornek(self, t, s, yeni_format, turetilmis):
        pos = [_f(s, k) for k in ('pos_x', 'pos_y', 'pos_z')]
        vel = [_f(s, k) for k in ('vel_x', 'vel_y', 'vel_z')]
        pos = None if any(v is None for v in pos) else pos
        vel = None if any(v is None for v in vel) else vel
        if yeni_format:
            hpos = [_f(s, k) for k in ('ref_hedef_x', 'ref_hedef_y', 'ref_hedef_z')]
            hvel = [_f(s, k) for k in ('ref_hedef_vx', 'ref_hedef_vy', 'ref_hedef_vz')]
            hpos = None if any(v is None for v in hpos) else hpos
            hvel = None if any(v is None for v in hvel) else hvel
            g = {'menzil_m': _f(s, 'ref_menzil_gercek_m'),
                 'kerteriz_deg': _f(s, 'ref_kerteriz_deg'),
                 'yukselis_deg': _f(s, 'ref_yukselis_deg'),
                 'yaklasim_deg': _f(s, 'ref_yaklasim_acisi_deg'),
                 'tip': s.get('ref_karsilasma_tipi', ''),
                 'kapanma_mps': _f(s, 'ref_kapanma_hizi_mps'),
                 'tgo_s': _f(s, 'ref_tgo_s'), 'cpa_m': _f(s, 'ref_cpa_m'),
                 'cpa_s': _f(s, 'ref_cpa_s')}
        else:
            hpos, hvel = self._en_yakin_hedef(t, turetilmis)
            g = geometri(pos, vel, hpos, hvel)
        return {
            't': t, 'kaynak': 'goruntulu',
            'menzil': _f(s, 'menzil_m'), 'geo': g,
            'pos': pos, 'vel': vel, 'hpos': hpos, 'hvel': hvel,
            'ex': _f(s, 'ex_deg'), 'ey': _f(s, 'ey_deg'),
            'kapsama': _f(s, 'kapsama_pct'),
            'durum': s.get('durum', ''), 'kurtarma': '',
            'vibe': _f(s, 'vibe_max'),
            'bbox_yas': _f(s, 'bbox_yas_s'),
            'kelepce': {k: _f(s, k) for k in
                        ('kelepce_hiz', 'kelepce_irtifa', 'kelepce_yaw_slew')
                        if s.get(k) not in ('', None)},
        }

    @staticmethod
    def _en_yakin_hedef(t, turetilmis):
        """Eski format: konumlu logdan zamanca en yakin hedef ornegini al."""
        if not turetilmis:
            return None, None
        en_iyi = min(turetilmis, key=lambda x: abs(x[0] - t))
        if abs(en_iyi[0] - t) > 1.0:
            return None, None
        return en_iyi[1], en_iyi[2]


# ---------------------------------------------------------------- anlatici

def anlat(kaynaklar, tam=False):
    gor = _oku(kaynaklar['goruntulu']) if kaynaklar['goruntulu'] else []
    kon = _oku(kaynaklar['konumlu']) if kaynaklar['konumlu'] else []
    if not gor and not kon:
        raise SystemExit(
            "okunacak log bulunamadi. Beklenenler:\n"
            "  <deneme>/goruntulu.log icinde '] log: <yol>.csv' satiri, ya da\n"
            "  guidance_allstar/logs/goruntulu_<metot>_<damga>.csv / "
            "guided_follow_<damga>.csv\n"
            "Goruntulu gudum hic calismadiysa (or. HOME_POSITION alinamadi) "
            "yalniz konumlu log anlatilir; o da yoksa kosu goruntulu-oncesi "
            "donemden olabilir.")
    if not gor:
        print("[not] goruntulu CSV yok -- yalniz KONUMLU faz anlatiliyor.")
    hiz = hizalayici_kur(gor, kon, kaynaklar['goruntulu'])
    tablo = Ornekler(gor, kon, hiz)
    o = tablo.ornek

    ad = (kaynaklar['deneme'].name if kaynaklar['deneme']
          else Path(kaynaklar['goruntulu']).stem)
    print("=" * 78)
    print(f"KOSU ANLATIMI: {ad}")
    print("=" * 78)
    print("kaynaklar:")
    for etiket, anahtar in (('goruntulu', 'goruntulu'), ('konumlu ', 'konumlu'),
                            ('mpc tani ', 'mpc_tani'), ('olaylar  ', 'olay'),
                            ('bbox     ', 'bbox')):
        y = kaynaklar.get(anahtar)
        print(f"  {etiket}: {y if y else '(yok)'}")
    n_gor, n_kon = len(gor), len(kon)
    yeni = bool(gor) and 'ref_hedef_x' in gor[0]
    print(f"  satir     : goruntulu {n_gor}, konumlu {n_kon}   "
          f"CSV bicimi: {'YENI (ref_* var)' if yeni else 'ESKI (hedef durumu konumludan turetildi)'}")
    if o:
        bas, son = o[0]['t'], o[-1]['t']
        print(f"  zaman     : t=0 -> {hiz.saat(bas)} "
              f"(unix {hiz.unix(bas):.0f})   sure {son - bas:.1f} s"
              if hiz.unix(bas) else
              f"  zaman     : sure {son - bas:.1f} s (mutlak saat bilinmiyor)")
    print()
    print("--- ZAMAN CIZELGESI ---")

    satirlar = []                      # (t_rel, sira, metin)
    onceki_durum = None
    kayip_basi = None
    esik_gecildi = set()
    en_yakin = {'m': float('inf'), 't': None, 'ornek': None}
    kelepce_acik = {}
    kelepce_sure = {}
    terminal_yazildi = False

    def ekle(t, metin, sira=1):
        satirlar.append((t, sira, metin))

    if o:
        x0 = o[0]
        ekle(hiz.rel(x0['t']),
             f"KOSU BASLADI -- {_faz_metni(x0, x0['menzil'], x0['geo'])}", 0)

    # --- DEVIR ANLARI: yetki araliklarindan (bkz. yetki_araliklari) ---
    for bas, son in tablo.araliklar:
        if son - bas < 0.3:
            continue                    # bir kac dongu suren tereddut
        ilk = _en_yakin_ornek(o, bas, 'goruntulu')
        if ilk is not None:
            ekle(hiz.rel(bas), f">>> DEVIR ALINDI (goruntulu gudum). "
                               f"{_devir_metni(ilk)}", 0)
        sonx = _en_yakin_ornek(o, son, 'goruntulu')
        ekle(hiz.rel(son), f"<<< yetki KONUMLUYA dondu "
                           f"(menzil {_m(sonx['menzil_duz'] if sonx else None)}, "
                           f"goruntulu faz {son - bas:.1f} s surdu).", 2)

    for i, x in enumerate(o):
        t = hiz.rel(x['t'])
        g = x['geo']
        mz = x['menzil_duz']

        # --- goruntulu fazin ic durumu: taze / tut / suz ---
        if x['kaynak'] == 'goruntulu':
            d = x['durum']
            if d and d != onceki_durum:
                if d in ('tut', 'suz') and onceki_durum == 'taze':
                    kayip_basi = t
                elif d == 'taze' and kayip_basi is not None:
                    if t - kayip_basi >= 0.4:
                        ekle(t, f"    hedef {t - kayip_basi:.1f} s kadrajdan "
                                f"CIKMISTI, geri geldi (menzil {_m(mz)})")
                    kayip_basi = None
                onceki_durum = d
            for k, v in x['kelepce'].items():
                acik = bool(v)
                if acik and not kelepce_acik.get(k):
                    kelepce_acik[k] = t
                elif not acik and kelepce_acik.get(k):
                    kelepce_sure[k] = kelepce_sure.get(k, 0.0) + (t - kelepce_acik[k])
                    kelepce_acik[k] = None

        # --- menzil esikleri (ilk gecis) ---
        if mz is not None:
            for esik in (500, 200, 100, 50, 30, 20, 10, 5):
                if mz <= esik and esik not in esik_gecildi:
                    esik_gecildi.add(esik)
                    if esik <= 100:
                        ekle(t, f"    menzil {esik} m altina indi "
                                f"({_gel2(x)})")
            # "en yakin" HAM menzille olculur (duzgunlestirme hizli gecisleri
            # yumusatip 6.3 m gibi yanlis bir dip bildirir; gercek 2.6 m'ydi).
            if x['menzil'] is not None and x['menzil'] < en_yakin['m']:
                en_yakin = {'m': x['menzil'], 't': t, 'ornek': x}

        if (not terminal_yazildi and mz is not None and mz < 20.0
                and x['kaynak'] == 'goruntulu'):
            terminal_yazildi = True
            ekle(t, f"    TERMINAL FAZ (menzil {_m(mz)}, {_gel2(x)})")

    # --- EN YAKIN GECISLER: menzil serisinin BELIRGIN yerel minimumlari ---
    # Anlik kapanma isaretine bakmak onlarca sahte tetik uretiyordu; burada
    # bir minimum ancak iki yaninda da menzil belirgin olcude yukseliyorsa
    # "gecis" sayilir.
    for x in _belirgin_minimumlar(o):
        dip_m = x['menzil']
        # Karsilasma tipini gecisten 2 s ONCEKI ornekten oku (gecis aninda
        # yaklasim acisi 0'dan 180'e savrulur; sorulan sey NASIL yaklastigimiz).
        onceki = _en_yakin_ornek(o, x['t'] - 2.0) or x
        ekle(hiz.rel(x['t']),
             f"*** EN YAKIN GECIS: {_m(dip_m)} -- yaklasirken {_gel2(onceki)}", 3)
        for y in o:
            if y['t'] > x['t'] and y['menzil_duz'] is not None and \
                    y['menzil_duz'] > max(2.0 * dip_m, dip_m + 15.0):
                ekle(hiz.rel(y['t']),
                     f"    ISKA: en yakin {dip_m:.1f} m idi, "
                     f"simdi {_m(y['menzil_duz'])} -- menzil aciliyor.", 4)
                break

    for t, _, metin in sorted(satirlar, key=lambda s: (s[0], s[1])):
        print(f"t={t:7.1f} s  {metin}")

    # --------------------------------------------------------------- ozet
    print()
    print("--- SONUC ---")
    if en_yakin['ornek'] is not None:
        x, g = en_yakin['ornek'], en_yakin['ornek']['geo']
        hukum = ('VURUS' if en_yakin['m'] <= 3.0 else
                 'YAKIN' if en_yakin['m'] <= 8.0 else 'ISKA')
        print(f"  hukum              : {hukum}  (en yakin {en_yakin['m']:.2f} m, "
              f"t={en_yakin['t']:.1f} s, kaynak={x['kaynak']})")
        # Karsilasma tipini GECIS ANINDA degil, gecisten 2 s ONCE oku: gecis
        # aninda aci hizla doner (yandan gecerken 90 dereceden gecer), oysa
        # sorulan sey "nasil yaklastik" -- kuyruktan mi, karsidan mi.
        onceki = _en_yakin_ornek(o, x['t'] - 2.0)
        gy = (onceki or x)['geo']
        print(f"  karsilasma tipi    : {TIP_METIN.get(gy.get('tip'), '?')}"
              + (f"  (yaklasim acisi {gy['yaklasim_deg']:.0f} deg, "
                 f"gecisten 2 s once)"
                 if gy.get('yaklasim_deg') is not None else ''))
        print(f"  kapanma hizi       : "
              f"{_ms((onceki or x).get('kapanma_duz'))} m/s (gecisten 2 s once)")
        if x['hvel'] is not None:
            hh = math.sqrt(sum(v * v for v in x['hvel']))
            hr = math.degrees(math.atan2(x['hvel'][1], x['hvel'][0])) % 360.0
            print(f"  hedef              : {hh:.1f} m/s, rota {_yon(hr)}")
        if x['vel'] is not None:
            bh = math.sqrt(sum(v * v for v in x['vel']))
            br = math.degrees(math.atan2(x['vel'][1], x['vel'][0])) % 360.0
            print(f"  biz                : {bh:.1f} m/s, rota {_yon(br)}")
        if g.get('yukselis_deg') is not None:
            print(f"  hedefin yukselisi  : {g['yukselis_deg']:+.1f} deg "
                  f"({'yukarida' if g['yukselis_deg'] > 0 else 'asagida'})")

    # Karsilasma tipi dagilimi: "yakinda kac kez kafa kafayaydik" sorusunun
    # cevabi -- kullanicinin 2026-08-04'te ELLE hesaplamak zorunda kaldigi sey.
    yakin = [x for x in o if x['kaynak'] == 'goruntulu'
             and (x['menzil'] or 1e9) < 60 and x['geo'].get('tip')]
    if yakin:
        sayim = {}
        for x in yakin:
            sayim[x['geo']['tip']] = sayim.get(x['geo']['tip'], 0) + 1
        print(f"  KARSILASMA DAGILIMI (goruntulu faz, menzil<60 m, n={len(yakin)}):")
        for tip, n in sorted(sayim.items(), key=lambda kv: -kv[1]):
            print(f"      {TIP_METIN.get(tip, tip):15s} %{100*n/len(yakin):5.1f} "
                  f"({n} ornek)")

    gor_ornek = [x for x in o if x['kaynak'] == 'goruntulu']
    if gor_ornek:
        taze = sum(1 for x in gor_ornek if x['durum'] == 'taze')
        if taze:
            print(f"  kadrajda kalma     : %{100*taze/len(gor_ornek):.1f} "
                  f"({taze}/{len(gor_ornek)} dongu 'taze')")
        vb = [x['vibe'] for x in gor_ornek if x['vibe'] is not None]
        if vb:
            print(f"  vibe tepe          : {max(vb):.1f}  "
                  f"(YER temasi >50; Gazebo hedefe carpmayi MODELLEMIYOR)")
    if kelepce_sure or any(kelepce_acik.values()):
        print("  kisit/kelepce toplam suresi:")
        for k in set(list(kelepce_sure) + list(kelepce_acik)):
            sure = kelepce_sure.get(k, 0.0)
            print(f"      {k:20s} {sure:5.1f} s")

    if kaynaklar['bbox']:
        _bbox_ozeti(kaynaklar['bbox'])
    if kaynaklar['olay']:
        _olay_ozeti(kaynaklar['olay'], hiz, tam)


def _en_yakin_ornek(ornek, t, kaynak=None):
    aday = [x for x in ornek if kaynak is None or x['kaynak'] == kaynak]
    if not aday:
        return None
    return min(aday, key=lambda x: abs(x['t'] - t))


def _belirgin_minimumlar(ornek, en_az_yukselis=8.0, oran=0.5):
    """Menzil serisindeki 'gercek' en yakin gecisleri bulur.

    Bir yerel minimum ancak SONRASINDA menzil belirgin olcude aciliyorsa
    gecis sayilir (min(en_az_yukselis, oran*menzil) kadar). Bu, gurultunun
    urettigi yuzlerce sozde minimumu eler; kalanlar gercekten "yaklastik,
    gectik, aciliyoruz" anlarina karsilik gelir.
    """
    seri = [x for x in ornek if x['menzil_duz'] is not None]
    if len(seri) < 5:
        return []
    # DIP <-> TEPE ALMASIK TARAMASI. Sadece "dipten esik kadar yukseldi" demek
    # yetmez: bir kez tetikledikten sonra menzil ACILMAYA devam ederken her
    # adim yeni bir "gecis" gibi gorunur (ilk denemede 12 sahte gecis cikti).
    # Yeni bir gecis ancak arada GERCEK bir tepe olup tekrar yaklasilirsa
    # sayilir.
    def esik(v):
        return max(en_az_yukselis, oran * v)

    sonuc = []
    mod, dip, tepe = 'dip', seri[0], seri[0]
    for x in seri:
        v = x['menzil_duz']
        if mod == 'dip':
            if v < dip['menzil_duz']:
                dip = x
            elif v - dip['menzil_duz'] >= esik(dip['menzil_duz']):
                sonuc.append(dip)
                mod, tepe = 'tepe', x
        else:
            if v > tepe['menzil_duz']:
                tepe = x
            elif tepe['menzil_duz'] - v >= esik(v):
                mod, dip = 'dip', x
    # KOSU SONU: acilma tamamlanmadan log bitmis olabilir. En yakin gecis
    # anlatimin EN ONEMLI satiri -- global minimum her halukarda yazilir.
    kucuk = min(seri, key=lambda y: y['menzil_duz'])
    if kucuk not in sonuc:
        sonuc.append(kucuk)
    # DUZGUNLESTIRME BULUR, HAM VERI OLCER: 38 m/s'lik bir gecis 0.6 s'lik
    # medyan penceresini 11 m yayar; dip ARANIRKEN duzgun seri (sahte tetik
    # olmasin), dip RAPORLANIRKEN +-1 s icindeki HAM en kucuk menzil kullanilir.
    return sorted((_ham_dip(seri, s) for s in sonuc if s['menzil_duz'] < 100.0),
                  key=lambda y: y['t'])


def _ham_dip(seri, dip, pencere_s=1.0):
    yakin = [y for y in seri if abs(y['t'] - dip['t']) <= pencere_s
             and y['menzil'] is not None]
    return min(yakin, key=lambda y: y['menzil']) if yakin else dip


def _m(v):
    return '---' if v is None else f"{v:.1f} m"


def _ms(v):
    return '---' if v is None else f"{v:.1f}"


def _gel2(x):
    """Karsilasmanin bir cumlelik ozeti: tip + (duzgunlestirilmis) kapanma."""
    g = x['geo']
    parca = []
    if g.get('tip'):
        parca.append(TIP_METIN.get(g['tip'], g['tip']))
        if g.get('yaklasim_deg') is not None:
            parca[-1] += f" [{g['yaklasim_deg']:.0f} deg]"
    kap = x.get('kapanma_duz')
    if kap is None:
        kap = g.get('kapanma_mps')
    if kap is not None:
        parca.append(f"kapanma {kap:+.1f} m/s")
        if kap > 0.3 and x.get('menzil_duz') and x['menzil_duz'] / kap < 30.0:
            parca.append(f"t_go {x['menzil_duz'] / kap:.1f} s")
    return ', '.join(parca) if parca else 'geometri yok'


def _faz_metni(x, mz, g):
    p = ['KONUMLU faz' if x['kaynak'] == 'konumlu' else 'GORUNTULU faz',
         f"menzil {_m(mz)}"]
    if x['hvel'] is not None:
        hh = math.sqrt(sum(v * v for v in x['hvel']))
        hr = math.degrees(math.atan2(x['hvel'][1], x['hvel'][0])) % 360.0
        p.append(f"hedef {hh:.1f} m/s rota {_yon(hr)}")
    p.append(_gel2(x))
    return ', '.join(p)


def _devir_metni(x):
    """Devir aninin tam tarifi: bu satiri okuyan devri GORMELI."""
    p = [f"menzil {_m(x['menzil_duz'])}"]
    # ey isareti: Olcum sozlesmesi -> ey_deg + = hedef ASAGIDA.
    # NEYE GORE "kadrajin altinda/ustunde": kadraj merkezi = kameranin OPTIK
    # EKSENI. Gimbal dali (2026-08-05): bu eksen artik govde pitch'i degil,
    # gimbalin dunya elevasyonu (tilt); govde savrulsa da eksen sabit kalir
    # (olculdu: govde +-35 deg iken kamera max 0.65 deg). Yani buradaki
    # "hedef kadrajin X deg ALTINDA" ifadesi dogrudan TILT hatasidir.
    if x['ey'] is not None:
        p.append(f"hedef kadrajin {abs(x['ey']):.1f} deg "
                 f"{'ALTINDA' if x['ey'] > 0 else 'USTUNDE'}")
    if x['ex'] is not None:
        p.append(f"{abs(x['ex']):.1f} deg "
                 f"{'SAGINDA' if x['ex'] > 0 else 'SOLUNDA'}")
    if x['kapsama'] is not None:
        p.append(f"kapsama %{x['kapsama']:.2f}")
    p.append(_gel2(x))
    return ', '.join(p)


def _bbox_ozeti(yol):
    """bbox.log ZAMAN DAMGASIZ: yalniz toplu istatistik verilebilir.

    Hizalama icin tek koprü, yeni goruntulu CSV'deki px_ham_cx/px_ham_cy
    kolonlaridir (ayni ham merkezler oradadir, zaman damgasiyla).
    """
    metin = ANSI.sub('', Path(yol).read_text(errors='replace'))
    kutu = re.findall(r'merkez=\((\d+),(\d+)\).*?cov=([0-9.]+)%', metin)
    ozet = re.findall(r'kare=(\d+)\s+fps=([0-9.]+)\s+tespit_orani=%([0-9.]+)', metin)
    print()
    print("--- DEDEKTOR (bbox.log; ZAMAN DAMGASI YOK, toplu istatistik) ---")
    if ozet:
        print(f"  islenen kare       : {ozet[-1][0]} (ort {sum(float(x[1]) for x in ozet)/len(ozet):.1f} fps)")
        print(f"  kumulatif tespit   : %{ozet[-1][2]}")
    if kutu:
        ys = sorted(int(k[1]) for k in kutu)
        cov = sorted(float(k[2]) for k in kutu)
        n = len(kutu)
        print(f"  tespit sayisi      : {n}")
        # Kadraj ortasi (360 px) = kameranin optik ekseni. GIMBAL DALI: bu
        # eksen tilt komutuyla belirlenir, govde pitch'iyle DEGIL; sistematik
        # bir sapma tilt/standoff geometrisi hatasidir (standoff_geom.sh).
        print(f"  kadraj y ortanca   : {ys[n//2]} (kadraj ortasi 360 = kamera "
              f"optik ekseni/tilt; kucuk = hedef YUKARIDA)")
        print(f"  kapsama ortanca    : %{cov[n//2]:.2f}  tepe %{cov[-1]:.2f}")


def _olay_ozeti(yol, hiz, tam):
    satir = _oku(yol)
    if not satir:
        return
    print()
    print("--- OLAY LOGU (goruntulu iskeletin ayrik olaylari) ---")
    for s in (satir if tam else satir[:60]):
        t = _f(s, 't')
        print(f"t={hiz.rel(t):7.1f} s  {s['olay']:28s} "
              f"menzil={s.get('menzil_m', ''):>8s}  {s.get('detay', '')}")
    if not tam and len(satir) > 60:
        print(f"  ... {len(satir)-60} olay daha (--tam ile hepsini gor)")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('deneme', nargs='?',
                   help='run/denemeler/<deneme> klasoru')
    p.add_argument('--goruntulu', help='dogrudan goruntulu CSV yolu')
    p.add_argument('--tam', action='store_true',
                   help='olay logunu kisaltmadan bas')
    a = p.parse_args()
    if not a.deneme and not a.goruntulu:
        p.error("bir deneme klasoru ya da --goruntulu CSV verin")
    d = Path(a.deneme) if a.deneme else None
    if d is not None and not d.exists():
        raise SystemExit(f"klasor yok: {d}")
    anlat(dosyalari_bul(d, a.goruntulu), a.tam)


if __name__ == '__main__':
    main()
