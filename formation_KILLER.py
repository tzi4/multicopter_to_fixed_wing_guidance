import os
import re
import sys
import time
import threading
import math
import json
import queue
import logging
import subprocess
import termios
import tty
import select
from logging.handlers import RotatingFileHandler
from enum import Enum
from pymavlink import mavutil
import redis
import cv2
import numpy as np
import config

# guidance_config sadece CANLI irtifa ofseti (ALT_OFFSET_M) icin okunur.
# Bulunamazsa saldiri guduumu ofsetsiz (0.0 m) calisir; surec ASLA cokmez.
try:
    import guidance_config as _guidance_cfg
    _GUIDANCE_CFG_IMPORT_ERR = None
except Exception as _exc:  # pragma: no cover - guidance_config yoksa
    _guidance_cfg = None
    _GUIDANCE_CFG_IMPORT_ERR = _exc  # logger henuz yok; _make_alt_offset uyarir


# DIKKAT: apply_global_speed_limit() bu sabitlerden WPNAV_SPEED'i ARACA YAZAR
# (kalkistan once, her kosuda). Yani elle Mission Planner'dan girilen WPNAV_SPEED
# bir sonraki kosuda SESSIZCE EZILIR. 2026-07-30 ucusunda WPNAV_SPEED=1700 olarak
# kaydedilmesinin sebebi budur: NAV_SPEED=17.0 idi. Bu sabitleri araca girilen
# degerle AYNI tut, yoksa kod pilotun ayarini geri alir.
MAX_SPEED = 25.0   # _set_speed() icindeki DO_CHANGE_SPEED tavani (24'u kirpmasin)
NAV_SPEED = 24.0   # -> WPNAV_SPEED = 2400 cm/s; araca girilen deger ile ayni
NAV_WPNAV_ACCEL_CMS = 300.0 # sqrt(WPNAV_ACCEL * mesafe) yani 4 m/s^2 ile 20m/s hiza 50 metrede ulasir

RTL_SPEED = 15
DRONE_IDS            = [1, 3, 4, 5] #[1, 2, 3, 4, 5]   [2, 3, 4, 5, 6]
LEADER_ID            = 3

ENABLE_KILL_STRATEGY = True
SUCCESSION_ORDER = [3, 1, 4, 5] # [3, 2, 5, 1, 4]  [4, 2, 5, 3, 6]

DIAMOND_TOP_GAP_M       = 20.0
DIAMOND_BOTTOM_GAP_M    = 20.0
DIAMOND_LATERAL_STAGGER = 3.0   # sağ/sol dikey kaydırma
DIAMOND_FORE_AFT_STAGGER= 3.0   # üst/alt ileri-geri kaydırma

# Zorunlu sıra: COAST < POS_LOITER < HB_TIMEOUT
LEADER_COAST_S       = 1.5   
LEADER_POS_LOITER_S  = 6.0
LEADER_HB_TIMEOUT_S  = 10.0 

assert LEADER_COAST_S < LEADER_POS_LOITER_S < LEADER_HB_TIMEOUT_S, \
    "Eşikler artan sırada olmalı: COAST < POS_LOITER < HB_TIMEOUT"


MSG_RATE_HZ = {
    "RAW_IMU":      50.0,   # id 27  — birincil darbe dedektörü (ivme sıçraması)
    "VIBRATION":    10.0,   # id 241 — doğrulayıcı titreşim sıçraması
    "SYS_STATUS":    5.0,   # id 1   — sensör sağlığı + batarya
    "SERVO_OUTPUT_RAW": 10.0, # id 36 — motor çıkışı doyumu (hasar öz-kontrolü)
}
IMPACT_ACCEL_G      = 3.5    # darbe bayrağı için |ivme| eşiği [g]
IMPACT_MIN_MS       = 50.0   # eşiğin üstünde bu kadar süre kalmalı [ms]

ENABLE_STATUSTEXT_LOGGING = True
STATUSTEXT_DEDUP_S = 3.0    # aynı drone'dan aynı metni bu süre içinde tekrar loglama

ATTACKER_PRIORITY   = [5, 1, 4, 3]  # Mute savaşı taktiği ;)   [3, 2, 5, 6]

ATTACK_SPEED_MPS  = 24.0 #mavlink do_change_speed komutu gondermek icin
CRUISE_SPEED_MPS  = 24.0   # formasyona dönünce geri yüklencek hiz

COMMIT_RANGE_M = 140.0
# --- Saldırı iptal / serbest bırakma politikası ------------------------------
ATTACK_TIMEOUT_S = 14.0
ATTACK_ABORT_ARM_M = 60.0
ATTACK_ABORT_OPEN_M = 30.0
# DİKKAT: ATTACK_ABORT_MAX_M 50 YAPILMAMALI! Saldırgan ~120-150 m menzilde
# göreve atanıyor; 50 m'lik mutlak sınır İLK TİKTE iptal ettirir. Gerçek
# "ıskaladık" sinyali, yaklaştıktan SONRA menzilin tekrar AÇILMASIDIR —
# bunu ATTACK_ABORT_ARM_M + ATTACK_ABORT_OPEN_M ikilisi yakalar.
ATTACK_ABORT_MAX_M  = 250.0  # mutlak: hedef tamamen kaçtıysa iptal [m]
ATTACK_COOLDOWN_S   = 10.0   # başarısız denemeden sonra bu drone'u tekrar seçme [s]
ATTACK_TERMINAL_M   = 45.0   # bu menzilin içinde kısa-öngörülü saf takibe geç [m]
ATTACK_MAX_TGT_SPD  = 35.0   # bu değerin üstündeki hedef hız tahminlerini reddet [m/s]
ATTACK_VEL_LPF      = 0.35   # hedef hız tahmini için alçak geçiren filtre katsayısı

ATTACK_WPNAV_SPEED_CMS = 2400.0
ATTACK_WPNAV_ACCEL_CMS = 400.0
ATTACK_OVERSHOOT_M = 50.0


ENABLE_ATTACK_EGRESS   = False    # False → eski davranış (düz beeline, sapma yok)
EGRESS_CLEAR_RADIUS_M  = 4.0    # bu yarıçap içinde komşu varsa nişan saptırılır [m]
EGRESS_DEFLECT_GAIN    = 1.0     # dışa/yukarı sapma şiddeti (büyük = daha keskin dönüş)
EGRESS_CEIL_PAD_M      = 5.0     # tavanın (max_alt) bu kadar altında kal [m]
ENABLE_TARGET_SIDE_SELECTION = True  # F: saldırganı hedef tarafındaki drone'dan seç

# --- Terminal (KILL) safhası: hız ram-through — WPNAV_ACCEL'e DOKUNMADAN hız ---
# ATTACK_TERMINAL_M içindeyken konum hedefi yerine SAF HIZ vektörü komutlanır;
# ArduCopter S-eğrisi hedefte sıfıra frenlemediği için drone hedefin İÇİNDEN tam
# hızla geçer (daha yüksek çarpma hızı, daha hızlı 'kill').
ENABLE_VELOCITY_TERMINAL = True  # False → terminalde de konum hedefi (eski davranış)

ENABLE_ATTACK_YAW_LOCK    = False 
ATTACK_YAW_UNLOCK_SPEED_MPS  = 8.0 
ATTACK_YAW_LOCK_TIMEOUT_S    = 15.0 
ATTACK_YAW_RECHASE_ANGLE_DEG = 60.0 
ATTACK_YAW_RECHASE_CONFIRM_S = 0.5 
# WP_YAW_BEHAVIOR araçtan okunamazsa kilit açılırken yazılacak yedek değer
# (ArduCopter fabrika varsayılanı = 2 / "ileriye bak").
DEFAULT_WP_YAW_BEHAVIOR      = 2.0

ATTACKER_SLOT_KEY   = "attacker_slot_ned"
ATTACKER_STATE_KEY  = "attacker_state_ned"
LEADER_STATE_KEY    = "leader_state_ned"    # liderin kendi güdümü için yayınlanan durum


POSITION_ONLY_MASK = 3576
# SET_POSITION_TARGET type_mask — yalnız HIZ (konum/ivme/yaw yok sayılır).
# bit0..2 (konum) + bit6..8 (ivme) + bit10,11 (yaw/yaw_rate) = 7+448+3072 = 3527.
VELOCITY_ONLY_MASK = 3527

# --- Yaw KİLİDİ maskeleri ---
# Yaw kilidi devredeyken sabit bir baş açısı KOMUTLANIR; bunun için yaw bitini
# (bit10 = 1024) maskeden DÜŞÜRÜRÜZ (bit temiz = "bu alanı kullan"). yaw_rate
# (bit11 = 2048) yok sayılmaya devam eder — dönüş hızı değil, mutlak açı veririz.
POSITION_YAW_MASK = POSITION_ONLY_MASK - 1024   # 2552: konum + sabit yaw
VELOCITY_YAW_MASK = VELOCITY_ONLY_MASK - 1024   # 2503: hız  + sabit yaw


def diamond_body_slots(n_followers, offset, top_gap, bottom_gap,
                       lateral_stagger, fore_aft_stagger):
    top    = (-fore_aft_stagger, 0.0,     -top_gap)          # geride + yukarıda
    right  = (0.0,               offset,  -lateral_stagger)  # sağda + biraz yukarıda
    left   = (0.0,              -offset,   lateral_stagger)  # solda + biraz aşağıda
    bottom = (fore_aft_stagger,  0.0,      bottom_gap)       # önde + aşağıda
    ladder = [top, right, left, bottom]
    if n_followers >= 4:
        return ladder[:4]
    if n_followers == 3:
        return [top, right, left]
    if n_followers == 2:
        return [(-fore_aft_stagger,  offset * 0.5, -top_gap),
                ( fore_aft_stagger, -offset * 0.5,  bottom_gap)]
    if n_followers == 1:
        return [(-fore_aft_stagger, 0.0, -top_gap)]
    return []


def solve_intercept_time(rel_pos, tgt_vel, atk_speed):
    """
        |rel_pos + tgt_vel * t| = atk_speed * t

        (|Vt|^2 - Sa^2) t^2 + 2 (R . Vt) t + |R|^2 = 0
    """
    vt2 = tgt_vel[0]**2 + tgt_vel[1]**2 + tgt_vel[2]**2
    a = vt2 - atk_speed * atk_speed
    b = 2.0 * (rel_pos[0]*tgt_vel[0] + rel_pos[1]*tgt_vel[1] + rel_pos[2]*tgt_vel[2])
    c = rel_pos[0]**2 + rel_pos[1]**2 + rel_pos[2]**2

    if abs(a) < 1e-6:
        # Hedef hızı == saldırgan hızı: denklem birinci dereceye iner.
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t > 1e-3 else None

    disc = b*b - 4.0*a*c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    roots = [(-b + sq) / (2.0*a), (-b - sq) / (2.0*a)]
    positive = [t for t in roots if t > 1e-3]
    return min(positive) if positive else None


def next_leader(succession_order, alive_ids, current_leader, skip_ids=None):
    skip = set(skip_ids or set())
    for did in succession_order:
        if did == current_leader:
            continue
        if did in skip:
            continue
        if did in alive_ids:
            return did
    return None



# Loglama   
def setup_logging(log_dir="logs", log_file="swarm.log", level=logging.INFO):
    os.makedirs(log_dir, exist_ok=True)
    logger = logging.getLogger("Swarm")
    logger.setLevel(level)
    logger.propagate = False

    if logger.handlers:
        for h in list(logger.handlers):
            logger.removeHandler(h)

    fmt = logging.Formatter(
        fmt="%(asctime)s | %(levelname)s | %(threadName)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    fh = RotatingFileHandler(
        os.path.join(log_dir, log_file),
        maxBytes=5_000_000,
        backupCount=3,
        encoding="utf-8",
    )
    fh.setLevel(level)
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    return logger


logger = setup_logging()

_EVENTS_LOG_DIR  = "logs"
_EVENTS_LOG_PATH = os.path.join(_EVENTS_LOG_DIR, "events.jsonl")
_events_lock     = threading.Lock()


def log_event(event_type: str, **fields):
    """
    logs/events.jsonl dosyasına tek satırlık bir JSON kaydı ekler.
    Örn: log_event("slot_reassignment", follower_id=3, previous_slot=1, new_slot=2)
    """
    record = {"ts": time.time(), "event": event_type}
    record.update(fields)
    try:
        os.makedirs(_EVENTS_LOG_DIR, exist_ok=True)
        with _events_lock:
            with open(_EVENTS_LOG_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception:
        logger.exception("Olay kaydı (JSONL) yazılırken hata.")


# =========================
# CANLI irtifa ofseti (guidance_config.ALT_OFFSET_M)
# =========================
class LiveAltOffset:
    """Uçuş SIRASINDA değiştirilebilen SAYISAL parametre okuyucusu.

    Değeri guidance_config.py içindeki ``<name> = <sayı>`` satırından okur;
    ``name`` varsayılan olarak ``ALT_OFFSET_M``'dir (komut irtifası ofseti:
    POZİTİF = daha YÜKSEK uç; çağıran taraf bunu komutlanan NED z'den çıkarır,
    NED'de z aşağı-pozitiftir), ama herhangi bir sayısal parametre için
    kullanılabilir -- bu dosyada ikinci bir örnek FORMATION_OFFSET_M'yi canlı
    okur. NOT: simple_guided_follow*.py içindeki kardeş kopyalar yalnızca
    ALT_OFFSET_M sürümünü taşır; bu kopya onların ÜST KÜMESİDİR (varsayılan
    argümanlarla davranış birebir aynıdır).

    Yeniden başlatma YOK: her ``poll_s`` saniyede bir dosyanın mtime'ına bakar
    ve yalnızca dosya değiştiyse ilgili satırı tek bir regex ile yeniden
    ayrıştırıp modüle geri yazar.

    Bilerek ``importlib.reload()`` KULLANILMAZ: reload (a) GUI'nin bellekte
    değiştirdiği tüm diğer parametreleri de ezerdi ve (b) tam o anda yarı
    kaydedilmiş bir dosyayı çalıştırabilirdi — hem de araç o sayılarla
    uçarken. Tek satırlık regex bunların ikisini de yapamaz: bozuk, eksik ya
    da kaydedilme anında yakalanmış bir satır sadece ÖNCEKİ değeri korur.

    Regex sayının ardından GERÇEK bir satır sonu (isteğe bağlı yorumla) arar;
    böylece kaydedilme anında yakalanıp GEÇERLİ bir sayıya kırpılan bir yazma
    -- "ALT_OFFSET_M = 25.0"un "ALT_OFFSET_M = 2" olarak okunması -- 2 m diye
    uçulmak yerine reddedilir.

    mtime'ı değişmiş ama ayrıştırılamayan bir dosya ``_MAX_REPARSE_TRIES``
    yoklama boyunca yeniden denenir (yarım yazma milisaniyelerde tamamlanır),
    sonra ``on_error`` ile pes edilir: satırı yorum satırı yapılmış bir config
    yüzünden dosyanın kontrol döngüsünde sonsuza dek yeniden okunması olmaz.

    Aralık dışı bir değer (yanlış tuş) kırpılır, uygulanmaz: varsayılan olarak
    +-``max_abs_m``, ya da ``min_m``/``max_m`` verildiyse o aralığa (ofset gibi
    simetrik olmayan büyüklükler için: formasyon ofseti 0'a inerse dronelar üst
    üste biner). ``slew_mps`` canlı bir değişimi basamak yerine rampa yapar.

    BİLİNEN SINIR: algılama mtime iledir; zaman damgası korunarak geri yüklenen
    bir config (cp -p, rsync -t, tar -x) GÖRÜLMEZ. Dosyaya `touch` atın.
    """

    _MAX_REPARSE_TRIES = 3

    @staticmethod
    def _build_pattern(name):
        return re.compile(
            r"^[ \t]*" + re.escape(name) + r"[ \t]*=[ \t]*"
            r"([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)"
            r"[ \t]*(?:#[^\n]*)?\n",
            re.MULTILINE,
        )

    def __init__(self, module=None, path=None, poll_s=0.5, max_abs_m=30.0,
                 slew_mps=0.0, default=0.0, on_change=None, on_error=None,
                 name="ALT_OFFSET_M", min_m=None, max_m=None):
        self._name = str(name)
        self._pattern = self._build_pattern(self._name)
        self._min_m = None if min_m is None else float(min_m)
        self._max_m = None if max_m is None else float(max_m)
        self._module = module
        if path is None and module is not None:
            path = getattr(module, "__file__", None)
        self._path = str(path) if path else None
        self._poll_s = max(0.0, float(poll_s))
        self._max_abs = abs(float(max_abs_m))
        self._slew = max(0.0, float(slew_mps))
        self._on_change = on_change
        self._on_error = on_error
        self._reparse_tries = 0
        self._value = self._clamp(self._read_module(default))
        self._applied = self._value
        # Modül varsa değeri zaten elimizde; yalnızca SONRAKİ dosya değişimi
        # ilgilendirir. Modül yoksa (import başarısız, yalnız-dosya yedeği)
        # elimizde hiçbir şey yok; mtime'ı boş bırak ki İLK yoklama dosyayı
        # gerçekten ayrıştırsın — aksi hâlde biri config'i yeniden kaydedene
        # kadar ofset sessizce 0.0'da kalırdı.
        self._mtime = self._stat() if module is not None else None
        self._next_poll = 0.0
        self._last_t = None

    def _clamp(self, v):
        lo = -self._max_abs if self._min_m is None else self._min_m
        hi = self._max_abs if self._max_m is None else self._max_m
        try:
            v = float(v)
        except (TypeError, ValueError):
            # Bozuk/eksik değer: asimetrik aralıkta 0.0 geçerli OLMAYABİLİR
            # (formasyon ofseti 0 = dronelar üst üste), o yüzden tabana düş.
            return 0.0 if self._min_m is None else lo
        if v != v:  # NaN
            return 0.0 if self._min_m is None else lo
        return max(lo, min(hi, v))

    def _read_module(self, default):
        if self._module is None:
            return default
        return getattr(self._module, self._name, default)

    def _stat(self):
        if not self._path:
            return None
        try:
            return os.path.getmtime(self._path)
        except OSError:
            return None

    def _poll(self, now):
        if now < self._next_poll:
            return
        self._next_poll = now + self._poll_s
        mtime = self._stat()
        if mtime is not None and mtime != self._mtime:
            parsed = None
            try:
                with open(self._path, "r", encoding="utf-8", errors="replace") as fh:
                    text = fh.read()
            except OSError:
                text = None
            if text is not None:
                match = self._pattern.search(text)
                if match is not None:
                    try:
                        parsed = self._clamp(float(match.group(1)))
                    except ValueError:
                        parsed = None
            if parsed is not None:
                # mtime YALNIZCA başarılı ayrıştırmadan sonra işlenir. Dosyayı
                # kaydedilme anında yakaladıysak mtime aynı kalır ve bir
                # sonraki yoklamada tekrar okuruz; aksi hâlde mtime
                # çözünürlüğü kaba olan bir bağlama noktasında (NFS, bazı
                # FUSE/paylaşılan klasörler) tamamlanan yazma aynı mtime'ı
                # taşıyabilir ve düzenleme belirti vermeden yutulurdu.
                self._mtime = mtime
                self._reparse_tries = 0
                if self._module is not None:
                    setattr(self._module, self._name, parsed)
                else:
                    self._file_value = parsed
            else:
                # Okunamadı / ayrıştırılamadı. Birkaç yoklama yeniden dene
                # (yarım yazma milisaniyelerde tamamlanır), sonra bu mtime'ı
                # kabul et ve okumayı bırak: aksi hâlde ALT_OFFSET_M satırı
                # yorum yapılmış bir config, dosyayı saniyede iki kez sonsuza
                # dek okuturdu -- üstelik bu yeniden denemenin var olma sebebi
                # olan ağ bağlama noktalarında her okuma bloklayan bir syscall.
                self._reparse_tries += 1
                if self._reparse_tries >= self._MAX_REPARSE_TRIES:
                    self._mtime = mtime
                    self._reparse_tries = 0
                    if self._on_error is not None:
                        try:
                            self._on_error(self._path, self._value)
                        except Exception:
                            pass
        if self._module is not None:
            new = self._clamp(self._read_module(self._value))
        else:
            new = self._clamp(getattr(self, "_file_value", self._value))
        if new != self._value:
            old, self._value = self._value, new
            if self._on_change is not None:
                try:
                    self._on_change(old, new)
                except Exception:
                    pass

    def target(self):
        """Rampa uygulanmadan ÖNCEKİ hedef ofset [m]."""
        return self._value

    def reset(self):
        """Rampayı anlık değere sabitler (yeni bir koşu/saldırı başlarken)."""
        self._applied = self._value
        self._last_t = None

    def value(self, now=None):
        """Şu an uygulanacak ofset [m]: kırpılmış, taze, gerekiyorsa rampalı."""
        now = time.monotonic() if now is None else float(now)
        self._poll(now)
        if self._slew <= 0.0:
            self._applied = self._value
            self._last_t = now
            return self._applied
        if self._last_t is None:
            self._last_t = now
            self._applied = self._value
            return self._applied
        dt = min(max(now - self._last_t, 0.0), 1.0)
        self._last_t = now
        delta = self._value - self._applied
        step = self._slew * dt
        if abs(delta) <= step:
            self._applied = self._value
        else:
            self._applied += step if delta > 0.0 else -step
        return self._applied


def _guidance_cfg_path(tag):
    """guidance_config.py'nin diskteki yolu (modül import edilemediyse yedek)."""
    if _guidance_cfg is not None:
        return None  # LiveAltOffset yolu modülün __file__'ından alır
    candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "guidance_config.py")
    path = candidate if os.path.exists(candidate) else None
    # SESSİZ KALMA: canlı parametrenin sessizce varsayılana düşmesi uçuşta
    # fark edilmez; tek belirtisi logdaki değerdir.
    logger.error(
        f"[{tag}] guidance_config IMPORT EDILEMEDI ({_GUIDANCE_CFG_IMPORT_ERR!r}). "
        + (f"Dosya yedeği kullanılıyor: {path}" if path else
           "Dosya da bulunamadı -> canlı parametre varsayılanda kalacak! "
           "guidance_config.py'yi bu betiğin yanına koyun.")
    )
    return path


def _make_alt_offset():
    """Saldırı güdümü için tek bir canlı irtifa ofseti okuyucusu kurar."""
    path = _guidance_cfg_path("AltOffset")
    return LiveAltOffset(
        module=_guidance_cfg,
        path=path,
        poll_s=float(getattr(_guidance_cfg, "ALT_OFFSET_POLL_S", 0.5)),
        max_abs_m=float(getattr(_guidance_cfg, "ALT_OFFSET_MAX_ABS_M", 30.0)),
        slew_mps=float(getattr(_guidance_cfg, "ALT_OFFSET_SLEW_MPS", 2.0)),
        default=0.0,
        on_change=lambda old, new: logger.warning(
            f"[AltOffset] ALT_OFFSET_M {old:+.2f} -> {new:+.2f} m "
            f"(canlı; saldırı nişan irtifası kaydırıldı)"
        ),
        on_error=lambda path, keep: logger.error(
            f"[AltOffset] {path} degisti ama kullanilabilir bir "
            f"'ALT_OFFSET_M = <sayi>' satiri bulunamadi -- {keep:+.2f} m ile "
            f"ucmaya devam ediliyor. Satiri duzeltip tekrar kaydedin."
        ),
    )


# Saldırı güdümünün komut irtifası ofseti. simple_guided_follow AYNI dosyadan
# kendi ofsetini okur; leader_slot_ned üzerinden gelen slot'a burada TEKRAR
# eklenmez (çift sayım olmasın diye) — bkz. _leader_slot_reader_thread.
ALT_OFFSET = _make_alt_offset()


def _make_formation_offset():
    """Formasyon ofseti için canlı okuyucu (guidance_config.FORMATION_OFFSET_M).

    Uçuş sırasında guidance_config.py'deki FORMATION_OFFSET_M satırını
    değiştirip kaydetmek yeter: ~0.5 s içinde formasyon yöneticisi yeni
    aralığı kullanır. Kod yeniden başlatılmaz, formasyon yeniden kurulmaz,
    slot atamaları sıfırlanmaz.

    Kırpma SİMETRİK DEĞİL: bu bir mesafedir, ofset değil. 0'a (veya negatife)
    inen bir değer tüm takipçileri liderin üstüne yığar -- bu yüzden taban
    FORMATION_OFFSET_MIN_M'dir ve bozuk/NaN bir değer 0.0'a değil TABANA düşer.
    Değişim FORMATION_OFFSET_SLEW_MPS ile rampalanır: 10 -> 25 m'lik bir
    düzenleme aksi hâlde her takipçinin hedefini tek karede 15 m kaydırır.
    """
    path = _guidance_cfg_path("FormOffset")
    lo = float(getattr(_guidance_cfg, "FORMATION_OFFSET_MIN_M", 3.0))
    hi = float(getattr(_guidance_cfg, "FORMATION_OFFSET_MAX_M", 60.0))
    return LiveAltOffset(
        module=_guidance_cfg,
        path=path,
        name="FORMATION_OFFSET_M",
        min_m=lo,
        max_m=hi,
        poll_s=float(getattr(_guidance_cfg, "FORMATION_OFFSET_POLL_S",
                             getattr(_guidance_cfg, "ALT_OFFSET_POLL_S", 0.5))),
        slew_mps=float(getattr(_guidance_cfg, "FORMATION_OFFSET_SLEW_MPS", 1.5)),
        default=float(getattr(_guidance_cfg, "FORMATION_OFFSET_M", 10.0)),
        on_change=lambda old, new: logger.warning(
            f"[FormOffset] FORMATION_OFFSET_M {old:.2f} -> {new:.2f} m "
            f"(canlı; formasyon aralığı {float(getattr(_guidance_cfg, 'FORMATION_OFFSET_SLEW_MPS', 1.5)):.1f} m/s ile kayacak)"
        ),
        on_error=lambda path, keep: logger.error(
            f"[FormOffset] {path} degisti ama kullanilabilir bir "
            f"'FORMATION_OFFSET_M = <sayi>' satiri bulunamadi -- {keep:.2f} m "
            f"ile ucmaya devam ediliyor. Satiri duzeltip tekrar kaydedin."
        ),
    )


# Formasyon aralığı (canlı). start_formation_following'e verilen 'offset'
# argümanı yalnızca BAŞLANGIÇ değeridir; _formation_manager_thread her tikte
# buradan okur.
FORMATION_OFFSET_LIVE = _make_formation_offset()


# =========================
# Formasyon Tipleri
# =========================
class Formation(Enum):
    LINE              = "LINE"
    HORIZONTAL_LINE   = "HORIZONTAL_LINE"
    V_SHAPE           = "V_SHAPE"
    DIAMOND           = "DIAMOND"


# =========================
# SwarmController
# =========================
class SwarmController:
    """
    Birleştirilmiş sürü kontrolcüsü.

    Mimari:
    - Tek bir paket-dispatcher thread'i   tüm MAVLink mesaj tiplerini işler.
    - Mod takibi   dispatcher'ın içine eklendi.
    - Yaw-rotasyonlu ve lookahead'li formasyon mantığı  .
    - follow_enable Event + pause_formation_following  .
    - parallel_launch.
    - Formasyon kurulduktan sonra simple_guided_follow . sh subprocess olarak başlatılır  .
    - Görev sonu RTL prosedürü  .
    - Her yerde loglama  .

    Güvenlik özellikleri:
    - Heartbeat watchdog: herhangi bir drone'dan 3s içinde heartbeat gelmezse RTL tetiklenir.
    - Klavye dinleyici: 'r' tuşu tüm drone'lara RTL, 'l' tuşu tüm drone'lara LAND gönderir.
    - Lider konum bekçisi: lider konumu 2s boyunca güncellenmezse takipçiler LOITER'a alınır.
    """

    def __init__(
        self,
        leader_id: int,
        connection_port: int,
        drone_ids: list,
        symmetric_horizontal: bool = False,
        ):
        self.drone_ids            = drone_ids
        self.leader_id            = leader_id
        self.symmetric_horizontal = symmetric_horizontal

        self.connection_string = f'udp:127.0.0.1:{connection_port}'
        logger.info(f"Bağlantı kuruluyor: {self.connection_string} | IDs={self.drone_ids}")
        self.master = mavutil.mavlink_connection(self.connection_string, source_system=255)

        # --- Paylaşılan durum ---
        self.drone_positions  = {}   # sysid -> (x, y, z)  NED
        self.drone_headings   = {}   # sysid -> yaw radyan
        self.drone_velocities = {}   # sysid -> (vx, vy, vz)
        self.drone_modes      = {}   # sysid -> mod string   
        self.takeoff_positions= {}   # sysid -> (n, e, d)  

        self.lock         = threading.Lock()
        self.stop_threads = threading.Event()
        self.threads      = []

        self.follow_enable = threading.Event()
        self.follow_enable.clear()

        self._slot_assignments = {} # Formasyon geçişlerinde en yakın slota yönlendirme için kullanılır
        self._slot_reassign_threshold_m = 5.0  # varsayılan, start_formation_following'de güncellenir
        # Son hesaplanan formasyon hedefleri {fid: (n,e,d)} — durum yayıncısı
        # viz_dashboard'un slot hedef çizgilerini çizebilmesi için yayınlar.
        self._last_formation_targets = {}

        self._current_formation = None
        self._current_offset    = None

        # Lider konum bekçisi için bayrak — False olduğunda takipçiler LOITER'a alınır
        self.leader_pos_ok = threading.Event()
        self.leader_pos_ok.set()   # başlangıçta sağlıklı kabul edilir

        self._hb_queues   = {did: queue.Queue()          for did in self.drone_ids}
        self._gpos_queues = {did: queue.Queue(maxsize=8) for did in self.drone_ids}
        
        self.drone_armed = {did: False for did in self.drone_ids}
        # COMMAND_ACK cevapları için drone başına kuyruk — arm/komut sonucunu doğrular
        self._command_ack_queues = {did: queue.Queue(maxsize=8) for did in self.drone_ids}
        
        self._nav_queues  = {did: queue.Queue(maxsize=8) for did in self.drone_ids}
        # MISSION_ITEM_REACHED için drone başına kuyruk — görev yeniden başlatma mantığında kullanılır
        self._mission_reached_queues = {did: queue.Queue(maxsize=16) for did in self.drone_ids}
        # MISSION_CURRENT için drone başına kuyruk — mevcut WP indeksini takip eder
        self._mission_current_queues = {did: queue.Queue(maxsize=4) for did in self.drone_ids}
        # MISSION_COUNT cevapları için kuyruk — dispatcher dışında recv_match kullanmaktan kaçınır
        self._mission_count_queue = queue.Queue(maxsize=4)
        # MISSION_ITEM/MISSION_ITEM_INT cevapları için kuyruk — son WP koordinatlarını almak için kullanılır
        self._mission_item_queue = queue.Queue(maxsize=4)

        # Kapatma sırasında sonlandırabilmek için simple_guided_follow subprocess handle'ı
        self._follow_proc = None
        # fetch_and_publish_ned_origin tarafından set edilen önbelleğe alınmış (lat, lon, alt) NED orijini
        self._ned_origin = None

        # Heartbeat watchdog için drone başına son heartbeat zamanı
        self._last_heartbeat_time = {did: time.monotonic() for did in self.drone_ids}
        # Lider konumunun son güncellenme zamanı (güvenlik özelliği 3)
        self._last_leader_pos_time = time.monotonic()
        self._alive_ids = set(self.drone_ids)
        self._grounded_ids = set()
        self._on_leader_lost = None
        self._election_lock = threading.Lock()
        self._election_in_progress = False

        # --- YENİ: yüksek hızlı telemetri kuyrukları (darbe/hasar analizi) --
        self._imu_queues    = {did: queue.Queue(maxsize=64) for did in self.drone_ids}
        self._vib_queues    = {did: queue.Queue(maxsize=16) for did in self.drone_ids}
        self._sys_queues    = {did: queue.Queue(maxsize=8)  for did in self.drone_ids}
        self._servo_queues  = {did: queue.Queue(maxsize=16) for did in self.drone_ids}

        # --- Parametre okuma yardımcı kuyruğu (get_param için genel amaçlı) ---
        # PARAM_VALUE cevapları için drone başına kuyruk — get_param bekler.
        # NOT: Hız tavanı ARTIK araçtan OKUNMUYOR; sabitlerden (NAV_SPEED /
        # MAX_SPEED) türetiliyor (bkz. apply_global_speed_limit). Bu kuyruk
        # yalnızca genel amaçlı get_param yardımcı metodu için tutuluyor.
        self._param_value_queues = {did: queue.Queue(maxsize=32) for did in self.drone_ids}
        # Drone başına son hesaplanan |ivme| (g) ve kalıcı darbe bayrağı.
        self._last_accel_g  = {did: 1.0   for did in self.drone_ids}
        self._impact_flag   = {did: False for did in self.drone_ids}
        # Drone başına son SYS_STATUS sağlık bit maskesi + batarya.
        self._sys_health    = {did: None  for did in self.drone_ids}
        # STATUSTEXT gürültü kırpma: sysid -> (son_metin, monotonic_zaman)
        self._last_statustext = {}
        # Saldırgan seçimi (F) için _kill_check'in gördüğü son hedef NED konumu.
        # Kesişme thread'i de her tikte günceller (yaw kilidi izleyicisi okur).
        self._last_target_ned = None
        # Saldırgan irtifa tavanı (config/controller.json'dan; main'de set edilir).
        self._attacker_max_alt = config.MAX_ALT_M

        # --- YENİ: saldırgan durum makinesi --------------------------------
        # Hangi takipçi şu an saldırıya atanmış durumda. None = yok.
        self._committed_attacker = None
        # (Eski) ikinci güdüm sürecine ait tanıtıcı — artık kullanılmıyor.
        self._attacker_proc = None
        # Lider seçiminde atlanması gereken drone'lar (saldırı ortasında).
        self._attack_skip = set()
        # drone_id -> bu zamana kadar tekrar saldırıya atanmamalı (monotonic).
        self._attack_cooldown = {}

        # --- Geçiş sonrası yaw kilidi durumu -------------------------------
        # drone_id -> {"yaw": donmuş baş açısı [rad], "t0": kilit anı [monotonic]}
        # Kilit, saldırgan formasyona bırakıldıktan SONRA da yaşamaya devam eder;
        # bu yüzden kesişme thread'inden ayrı bir izleyici thread'i tarafından
        # yönetilir (bkz. _yaw_lock_watchdog).
        self._yaw_locks = {}
        self._yaw_lock_mu = threading.Lock()
        # drone_id -> saldırıya girmeden ÖNCE okunan WP_YAW_BEHAVIOR (geri yükleme).
        self._wp_yaw_behavior_saved = {}

        self._shutting_down = threading.Event()

        # Tek dispatcher thread'ini başlat (MINE mimarisi)
        dispatch_thread = threading.Thread(
            target=self._packet_dispatcher_thread,
            name="PacketDispatcher",
            daemon=True,
        )
        dispatch_thread.start()
        self.threads.append(dispatch_thread)
        logger.info("Paket dispatcher thread başlatıldı.")

        self.wait_for_all_heartbeats()

    # ------------------------------------------------------------------
    # Paket dispatcher — tek thread, tüm mesaj tipleri  
    # Mod takibi de buraya eklendi  
    # ------------------------------------------------------------------
    def _packet_dispatcher_thread(self):
        while not self.stop_threads.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if not msg:
                continue

            src   = msg.get_srcSystem()
            mtype = msg.get_type()

            # --- HEARTBEAT: kuyruklar + mod takibi + watchdog zaman damgası ---
            if mtype == 'HEARTBEAT' and src in self._hb_queues:
                q = self._hb_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

                # Watchdog için son heartbeat zamanını güncelle
                self._last_heartbeat_time[src] = time.monotonic()

                # HEARTBEAT'ten arm durumunu güncelle (arm doğrulaması için)
                self.drone_armed[src] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                # Mod takibi   — ayrı bir thread'den kaçınmak için buraya entegre edildi
                try:
                    mode_name = mavutil.mode_string_v10(msg)
                    with self.lock:
                        if self.drone_modes.get(src) != mode_name:
                            logger.info(f"Mode update -> drone={src}, mode={mode_name}")
                            log_event("mode_change", drone_id=src, mode=mode_name)
                            self.drone_modes[src] = mode_name
                except Exception:
                    pass

            # --- GLOBAL_POSITION_INT: irtifa beklemesi + yön + PAYLAŞILAN NED konum/hız ---
            elif mtype == 'GLOBAL_POSITION_INT' and src in self._gpos_queues:
                q = self._gpos_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

                with self.lock:
                    if msg.hdg != 65535:
                        self.drone_headings[src] = math.radians(msg.hdg / 100.0)

                if self._ned_origin is not None:
                    n, e, d = latlon_to_ned(
                        msg.lat / 1e7,
                        msg.lon / 1e7,
                        msg.alt / 1000.0,   # AMSL metre
                        *self._ned_origin,
                    )
                    with self.lock:
                        self.drone_positions[src]  = (n, e, d)
                        # vx/vy/vz NED eksenlerinde cm/s -> m/s (eksenler ortak, yalnızca
                        # orijin farklı olduğundan hızların yeniden referanslanmasına gerek yok).
                        self.drone_velocities[src] = (
                            msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
                        )

                    # Lider konumu güncellendiğinde zaman damgasını güncelle (güvenlik özelliği 3)
                    if src == self.leader_id:
                        self._last_leader_pos_time = time.monotonic()

            # DÜZELTME: LOCAL_POSITION_NED alımı devre dışı. Her araç bu mesajı KENDİ
            # EKF orijinine göre bildirir; bu çerçeveleri drone'lar arasında karıştırmak
            # takipçileri yanlış yöne gönderiyordu. Konum/hız artık yukarıdaki
            # GLOBAL_POSITION_INT'ten, ortak orijin çerçevesinde geliyor.
            # elif mtype == 'LOCAL_POSITION_NED' and src in self.drone_ids:
            #     with self.lock:
            #         self.drone_positions[src]  = (msg.x, msg.y, msg.z)
            #         self.drone_velocities[src] = (msg.vx, msg.vy, msg.vz)
            #     if src == self.leader_id:
            #         self._last_leader_pos_time = time.monotonic()

            # --- NAV_CONTROLLER_OUTPUT: waypoint mesafesi ---
            elif mtype == 'NAV_CONTROLLER_OUTPUT' and src in self._nav_queues:
                q = self._nav_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_ITEM_REACHED: görev yeniden başlatma tespiti için ---
            elif mtype == 'MISSION_ITEM_REACHED' and src in self._mission_reached_queues:
                q = self._mission_reached_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_CURRENT: aktif waypoint indeksini takip eder ---
            elif mtype == 'MISSION_CURRENT' and src in self._mission_current_queues:
                q = self._mission_current_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- COMMAND_ACK: arm/komut sonucunu doğrulamak için ---
            elif mtype == 'COMMAND_ACK' and src in self._command_ack_queues:
                q = self._command_ack_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_COUNT: mission_request_list_send'e cevap ---
            elif mtype == 'MISSION_COUNT':
                q = self._mission_count_queue
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- MISSION_ITEM_INT: mission_request_int_send'e cevap ---
            elif mtype == 'MISSION_ITEM_INT':
                q = self._mission_item_queue
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- YENİ: RAW_IMU — birincil darbe dedektörü (ivme sıçraması) ----
            elif mtype == 'RAW_IMU' and src in self._imu_queues:
                # ArduPilot'ta RAW_IMU'nun xacc/yacc/zacc değerleri mg (mili-g) birimindedir.
                try:
                    ax = msg.xacc / 1000.0
                    ay = msg.yacc / 1000.0
                    az = msg.zacc / 1000.0
                    mag_g = math.sqrt(ax*ax + ay*ay + az*az)
                    self._last_accel_g[src] = mag_g
                except Exception:
                    pass
                q = self._imu_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- YENİ: VIBRATION — darbeyi doğrulayan titreşim sinyali --------
            elif mtype == 'VIBRATION' and src in self._vib_queues:
                q = self._vib_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- YENİ: SYS_STATUS — sensör sağlığı + batarya ------------------
            elif mtype == 'SYS_STATUS' and src in self._sys_queues:
                try:
                    self._sys_health[src] = {
                        "health": int(msg.onboard_control_sensors_health),
                        "present": int(msg.onboard_control_sensors_present),
                        "enabled": int(msg.onboard_control_sensors_enabled),
                        "batt_v": (msg.voltage_battery / 1000.0
                                   if msg.voltage_battery not in (0, 65535) else None),
                        "batt_a": (msg.current_battery / 100.0
                                   if msg.current_battery not in (0, -1) else None),
                    }
                except Exception:
                    pass
                q = self._sys_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- YENİ: SERVO_OUTPUT_RAW — motor doyumu (hasar kontrolü) -------
            elif mtype == 'SERVO_OUTPUT_RAW' and src in self._servo_queues:
                q = self._servo_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- YENİ: PARAM_VALUE — get_param cevabı (WPNAV_SPEED/ACCEL) -----
            elif mtype == 'PARAM_VALUE' and src in self._param_value_queues:
                q = self._param_value_queues[src]
                if q.full():
                    try: q.get_nowait()
                    except queue.Empty: pass
                try: q.put_nowait(msg)
                except queue.Full: pass

            # --- STATUSTEXT — otonom mod değişimi / failsafe NEDENİNİ yakala ---
            # ENABLE_STATUSTEXT_LOGGING ile açılıp kapatılır. Yalnızca loglar;
            # uçuş mantığına dokunmaz.
            elif (mtype == 'STATUSTEXT' and ENABLE_STATUSTEXT_LOGGING
                  and src in self._hb_queues):
                self._handle_statustext(src, msg)

    def _handle_statustext(self, src, msg):
        """
        Araçtan gelen STATUSTEXT'i swarm.log'a ve events.jsonl'a (panel olay
        akışına) yazar. Bir drone kendi kendine LAND/RTL'e düştüğünde nedenini
        (GCS/EKF/batarya failsafe vb.) görünür kılar.

        Tamamen EKLEMELİDİR — hiçbir durumu silmez/değiştirmez. Üst seviyedeki
        ENABLE_STATUSTEXT_LOGGING bayrağı ile güvenle kapatılabilir.
        """
        try:
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode("utf-8", "replace")
            text = (text or "").strip()
            if not text:
                return

            severity = int(getattr(msg, "severity", 6))

            # Gürültü kırpma: aynı drone'dan aynı metni kısa sürede tekrarlama.
            now = time.monotonic()
            prev = self._last_statustext.get(src)
            if prev and prev[0] == text and (now - prev[1]) < STATUSTEXT_DEDUP_S:
                return
            self._last_statustext[src] = (text, now)

            # MAVLink severity: 0=emergency … 3=error, 4=warning, 5..7=notice/info/debug
            if severity <= 3:
                logger.error(f"[STATUSTEXT] d{src} sev={severity}: {text}")
            elif severity == 4:
                logger.warning(f"[STATUSTEXT] d{src} sev={severity}: {text}")
            else:
                logger.info(f"[STATUSTEXT] d{src} sev={severity}: {text}")

            log_event("statustext", drone_id=src, severity=severity, text=text)
        except Exception:
            logger.exception("[STATUSTEXT] işlenirken hata.")

    # ------------------------------------------------------------------
    # Güvenlik özelliği 1 — Heartbeat watchdog thread'i
    # Her drone'un heartbeat'ini izler. Herhangi bir drone'dan 3 saniye
    # içinde heartbeat gelmezse, o drone'a RTL komutu gönderilir.
    # ------------------------------------------------------------------
    def _heartbeat_watchdog_thread(self, timeout_s: float = 3.0):
        rtl_sent     = set()
        _logged_lost = set()
        logger.info(f"Heartbeat watchdog başlatıldı (timeout={timeout_s}s)")

        while not self.stop_threads.is_set():
            now = time.monotonic()
            with self.lock:
                alive_now = set()
            for did in self.drone_ids:
                # Kalkışta görevden çıkarılan drone'lar: heartbeat gönderseler
                # bile 'hayatta' kümesine ASLA geri alınmaz.
                if did in self._grounded_ids:
                    self._alive_ids.discard(did)
                    continue
                last = self._last_heartbeat_time.get(did, now)
                age  = now - last
                if age > timeout_s:
                    if did not in _logged_lost:
                        logger.warning(
                            f"[Watchdog] Drone {did} heartbeat'i {age:.1f}s önce kesildi!"
                        )
                        log_event("heartbeat_lost", drone_id=did, age_s=round(age, 2))
                        _logged_lost.add(did)
                    if did not in rtl_sent:
                        self.set_mode(did, "RTL")      # elden geldiğince, tek sefer
                        rtl_sent.add(did)
                    # Bu drone'u mevcut 'hayatta' kümesinden çıkar.
                    self._alive_ids.discard(did)
                else:
                    alive_now.add(did)
                    self._alive_ids.add(did)
                    if did in _logged_lost:
                        logger.info(f"[Watchdog] Drone {did} heartbeat'i geri döndü.")
                        log_event("heartbeat_recovered", drone_id=did)
                        _logged_lost.discard(did)
                    rtl_sent.discard(did)

            
            # Seçim, ölü-işaretleme ile AYNI eşiği (timeout_s) kullanır. Böylece
            # bu tikte lider zaten _alive_ids'ten çıkarılmış olur -> erken failover yok.
            ldr = self.leader_id
            ldr_age = now - self._last_heartbeat_time.get(ldr, now)
            if ldr_age > timeout_s and self._on_leader_lost is not None:
                self._on_leader_lost(ldr, set(self._alive_ids))

            time.sleep(0.5)

    def start_heartbeat_watchdog(self, timeout_s: float = 3.0):
        """Heartbeat watchdog thread'ini başlatır. parallel_launch sonrasında çağrılmalıdır."""
        # Watchdog'u başlatmadan önce zaman damgalarını sıfırla, böylece
        # kalkış süresi yanlış uyarı tetiklemez.
        now = time.monotonic()
        for did in self.drone_ids:
            self._last_heartbeat_time[did] = now

        t = threading.Thread(
            target=self._heartbeat_watchdog_thread,
            args=(timeout_s,),
            name="HBWatchdog",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Heartbeat watchdog thread başlatıldı.")

    # ------------------------------------------------------------------
    # Güvenlik özelliği 2 — Klavye acil durum dinleyicisi
    # 'r' → tüm drone'lara RTL
    # 'l' → tüm drone'lara LAND
    # 'q' → programı durdur (mevcut finally bloğu RTL'yi ele alır)
    # Terminal raw modunda çalışır; engelleme olmadan tuş okur.
    # ------------------------------------------------------------------
    def _keyboard_listener_thread(self):
        logger.info(
            "[Klavye] Acil durum dinleyicisi başladı. "
            "r=RTL, l=LAND, q=Çıkış"
        )
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self.stop_threads.is_set():
                # Engelleme olmadan bir karakter olup olmadığını kontrol et
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1).lower()
                    if ch == 'r':
                        logger.warning("[Klavye] 'r' algılandı — tüm drone'lara RTL gönderiliyor!")
                        log_event("emergency_command", command="RTL", source="keyboard")
                        for did in self.drone_ids:
                            self.set_mode(did, "RTL")
                    elif ch == 'l':
                        logger.warning("[Klavye] 'l' algılandı — tüm drone'lara LAND gönderiliyor!")
                        log_event("emergency_command", command="LAND", source="keyboard")
                        for did in self.drone_ids:
                            self.set_mode(did, "LAND")
                    elif ch == 'q':
                        logger.warning("[Klavye] 'q' algılandı — program sonlandırılıyor!")
                        self.stop_threads.set()
                        break
        except Exception:
            logger.exception("Klavye dinleyici hatası.")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def start_keyboard_listener(self):
        """Klavye acil durum dinleyici thread'ini başlatır."""
        t = threading.Thread(
            target=self._keyboard_listener_thread,
            name="KeyboardListener",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Klavye dinleyicisi başlatıldı (r=RTL, l=LAND, q=Çıkış).")

    # ------------------------------------------------------------------
    # Güvenlik özelliği 3 — Lider konum bekçisi thread'i
    # Lider drone'un LOCAL_POSITION_NED konumu 2 saniyeden uzun süre
    # güncellenmezse, tüm takipçiler LOITER moduna alınır ve formasyon
    # komutları duraklatılır. Konum geri dönünce formasyon devam eder.
    # ------------------------------------------------------------------
    def _leader_position_watchdog_thread(self, stale_timeout_s: float = 2.0):
        loiter_active = False
        last_warn     = 0.0
        logger.info(f"Lider konum bekçisi başlatıldı (timeout={stale_timeout_s}s)")

        while not self.stop_threads.is_set():
            now = time.monotonic()
            age = now - self._last_leader_pos_time
            stale = age > stale_timeout_s

            if stale and not loiter_active:
                logger.warning(
                    f"[LiderBekçi] Lider konumu {age:.1f}s güncellenmedi! "
                    f"Takipçiler LOITER'a alınıyor."
                )
                log_event("leader_position_lost", age_s=round(age, 2))
                self.follow_enable.clear()   # formasyon komutlarını durdur
                self.leader_pos_ok.clear()
                for did in self.drone_ids:
                    if did != self.leader_id and did not in self._grounded_ids:
                        self.set_mode(did, "LOITER")
                loiter_active = True

            elif stale and loiter_active:
                if now - last_warn > 5.0:
                    logger.warning(
                        f"[LiderBekçi] Lider konumu hâlâ bayat ({age:.1f}s). "
                        f"Takipçiler LOITER'da bekliyor."
                    )
                    last_warn = now

            elif not stale and loiter_active:
                logger.info(
                    "[LiderBekçi] Lider konumu geri döndü — "
                    "takipçiler GUIDED'e alınıyor ve formasyon devam ediyor."
                )
                log_event("leader_position_recovered")
                for did in self.drone_ids:
                    if did != self.leader_id and did not in self._grounded_ids:
                        self.set_mode(did, "GUIDED")
                self.leader_pos_ok.set()
                self.follow_enable.set()   # formasyon komutlarını yeniden başlat
                loiter_active = False

            time.sleep(0.5)

    def start_leader_position_watchdog(self, stale_timeout_s: float = 2.0):
        """Lider konum bekçisi thread'ini başlatır. parallel_launch sonrasında çağrılmalıdır."""
        self._last_leader_pos_time = time.monotonic()
        t = threading.Thread(
            target=self._leader_position_watchdog_thread,
            args=(stale_timeout_s,),
            name="LeaderPosWatchdog",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Lider konum bekçisi thread'i başlatıldı.")

    # ------------------------------------------------------------------
    # Canlı formasyon görselleştirme paneli için durum yayıncısı  
    # Sürü durumunu (lider + takipçi NED pozisyonları, modlar, atanan
    # slotlar) periyodik olarak Redis'e yayınlar; viz_dashboard.py bunu
    # okuyup tarayıcıda canlı olarak çizer. Ayrıca daha düşük hızda
    # events.jsonl'a "state_snapshot" olarak da kaydeder (uçuş-sonrası
    # tekrar oynatma / analiz için).
    # ------------------------------------------------------------------
    def _state_publisher_thread(self, redis_client, rate_hz: float = 5.0, jsonl_rate_hz: float = 1.0):
        REDIS_KEY = "swarm_state"
        dt        = 1.0 / max(rate_hz, 0.1)
        jsonl_dt  = 1.0 / max(jsonl_rate_hz, 0.01)
        last_jsonl = 0.0
        logger.info(
            f"Durum yayıncısı başlatıldı (canlı={rate_hz:.1f}Hz, JSONL={jsonl_rate_hz:.1f}Hz)."
        )

        while not self.stop_threads.is_set():
            try:
                with self.lock:
                    positions  = dict(self.drone_positions)
                    headings   = dict(self.drone_headings)
                    modes      = dict(self.drone_modes)
                    velocities = dict(self.drone_velocities)
                slot_assignments  = dict(self._slot_assignments)
                formation_targets = dict(self._last_formation_targets)
                now_mono = time.monotonic()

                def _drone_snapshot(did):
                    sysh = self._sys_health.get(did) or {}
                    return {
                        "position": positions.get(did),
                        "heading": headings.get(did),
                        "mode": modes.get(did),
                        "slot": slot_assignments.get(did),
                        "is_leader": did == self.leader_id,
                        # --- Zenginleştirilmiş telemetri (viz_dashboard için) ---
                        "velocity": velocities.get(did),
                        "armed": self.drone_armed.get(did, False),
                        "hb_age_s": round(
                            now_mono - self._last_heartbeat_time.get(did, now_mono), 2
                        ),
                        "accel_g": round(self._last_accel_g.get(did, 1.0), 2),
                        "impact": bool(self._impact_flag.get(did, False)),
                        "grounded": did in self._grounded_ids,
                        "alive": did in self._alive_ids,
                        "attacker": did == self._committed_attacker,
                        "batt_v": sysh.get("batt_v"),
                        "batt_a": sysh.get("batt_a"),
                        # Formasyon yöneticisinin bu drone için son hesapladığı
                        # slot hedefi (n, e, d) — panel kesikli hedef çizgisi çizer.
                        "target": formation_targets.get(did),
                    }

                snapshot = {
                    "ts": time.time(),
                    "leader_id": self.leader_id,
                    "formation": self._current_formation,
                    "offset": self._current_offset,
                    "follow_enabled": self.follow_enable.is_set(),
                    "leader_pos_ok": self.leader_pos_ok.is_set(),
                    "attacker_id": self._committed_attacker,
                    "drones": {str(did): _drone_snapshot(did) for did in self.drone_ids},
                }

                try:
                    redis_client.set(REDIS_KEY, json.dumps(snapshot))
                except Exception:
                    logger.exception("Durum Redis'e yayınlanırken hata.")

                now = time.monotonic()
                if now - last_jsonl > jsonl_dt:
                    log_event("state_snapshot", **snapshot)
                    last_jsonl = now

            except Exception:
                logger.exception("Durum yayıncısı thread hatası.")

            time.sleep(dt)

    def start_state_publisher(self, redis_client, rate_hz: float = 5.0, jsonl_rate_hz: float = 1.0):
        """
        Canlı formasyon görselleştirme paneli (viz_dashboard.py) için durum
        yayıncısı thread'ini başlatır. Ne zaman çağrıldığı önemli değildir —
        kalkıştan önce de çağrılabilir, henüz konum yoksa alanlar None olur.
        """
        t = threading.Thread(
            target=self._state_publisher_thread,
            args=(redis_client,),
            kwargs={"rate_hz": rate_hz, "jsonl_rate_hz": jsonl_rate_hz},
            name="StatePublisher",
            daemon=True,
        )
        t.start()
        self.threads.append(t)
        logger.info("Durum yayıncısı thread'i başlatıldı.")

    # ------------------------------------------------------------------
    # Heartbeat / bağlantı yardımcıları
    # ------------------------------------------------------------------
    def wait_for_all_heartbeats(self, timeout: float = 30.0):
        logger.info(f"Heartbeat bekleniyor: {self.drone_ids}")
        unseen   = set(self.drone_ids)
        deadline = time.time() + timeout

        while unseen and time.time() < deadline:
            for did in list(unseen):
                try:
                    self._hb_queues[did].get(timeout=0.2)
                    unseen.discard(did)
                    logger.info(f"Heartbeat alındı -> {did}, kalan={unseen or 'none'}")
                except queue.Empty:
                    pass

        if unseen:
            raise ConnectionError(f"Tüm drone'lara bağlanılamadı. Eksik: {unseen}")
        logger.info("Tüm drone'lardan heartbeat alındı. Sistem hazır.")

    # ------------------------------------------------------------------
    # Mod yardımcıları  
    # ------------------------------------------------------------------
    def is_in_guided_mode(self, target_id: int) -> bool:
        with self.lock:
            current_mode = self.drone_modes.get(target_id, "")
        logger.debug(f"is_in_guided_mode drone={target_id} mode={current_mode}")
        return current_mode == "GUIDED"

    def _wait_for_guided(self, drone_id: int, timeout: float = 15.0) -> bool:
        """Heartbeat custom_mode == 4 (GUIDED) gösterene kadar bekler.  """
        deadline = time.time() + timeout
        q        = self._hb_queues[drone_id]
        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                if msg.custom_mode == 4:
                    return True
                self.set_mode(drone_id, "GUIDED")
            except queue.Empty:
                self.set_mode(drone_id, "GUIDED")
        return False

    # ------------------------------------------------------------------
    # AUTO döngüsü için görev yardımcıları  
    # ------------------------------------------------------------------
    def _restart_mission(self, target_id: int, seq: int = 2):
        """
        MAV_CMD_DO_SET_MISSION_CURRENT (seq=seq) göndererek AUTO görev
        döngüsünü yeniden başlatır.
        """
        logger.info(f"[MissionRestart] Misyon seq={seq}'den yeniden başlatılıyor (drone={target_id})")
        log_event("mission_restart", drone_id=target_id, seq=seq)
        self.master.mav.command_long_send(
            target_id, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            0,
            float(seq),   # param1: atlanacak sıra numarası
            0, 0, 0, 0, 0, 0,
        )

    def _get_mission_count(self, target_id: int, timeout: float = 5.0) -> int:
        """
        target_id'den MISSION_COUNT ister ve görev öğesi sayısını döndürür.
        Timeout durumunda -1 döndürür.
        """
        while True:
            try: self._mission_count_queue.get_nowait()
            except queue.Empty: break

        self.master.mav.mission_request_list_send(target_id, 0)
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = self._mission_count_queue.get(
                    timeout=min(1.0, deadline - time.time())
                )
                if msg.get_srcSystem() == target_id:
                    logger.info(f"[MissionRestart] Mission count={msg.count} (drone={target_id})")
                    return int(msg.count)
                try: self._mission_count_queue.put_nowait(msg)
                except queue.Full: pass
            except queue.Empty:
                self.master.mav.mission_request_list_send(target_id, 0)

        logger.warning(f"[MissionRestart] MISSION_COUNT timeout (drone={target_id})")
        return -1

    def _get_last_waypoint_global(self, target_id: int, last_wp_index: int, timeout: float = 5.0):
        """
        (last_wp_index - 1) indeksi için MISSION_ITEM_INT ister ve
        (lat, lon, alt) değerlerini float olarak döndürür.
        Timeout durumunda None döndürür.
        """
        item_seq = last_wp_index - 1
        while True:
            try: self._mission_item_queue.get_nowait()
            except queue.Empty: break

        self.master.mav.mission_request_int_send(target_id, 0, item_seq)
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = self._mission_item_queue.get(timeout=min(1.0, deadline - time.time()))
                if msg.get_srcSystem() == target_id and msg.seq == item_seq:
                    lat = msg.x / 1e7
                    lon = msg.y / 1e7
                    alt = msg.z
                    logger.info(
                        f"[MissionRestart] Son WP koordinatı (seq={item_seq}): "
                        f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}"
                    )
                    return lat, lon, alt
                try: self._mission_item_queue.put_nowait(msg)
                except queue.Full: pass
            except queue.Empty:
                self.master.mav.mission_request_int_send(target_id, 0, item_seq)

        logger.warning(f"[MissionRestart] MISSION_ITEM_INT timeout (drone={target_id}, seq={item_seq})")
        return None

    # ------------------------------------------------------------------
    # MAVLink komut yardımcıları
    # ------------------------------------------------------------------
    def set_mode(self, target_system_id: int, mode_name: str):
        COPTER_MODES = {
            "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
            "GUIDED": 4,    "LOITER": 5, "RTL": 6,    "CIRCLE": 7,
            "LAND": 9,      "POSHOLD": 16, "BRAKE": 17,
        }
        mode_id = COPTER_MODES.get(mode_name.upper())
        if mode_id is None:
            logger.warning(f"Bilinmeyen mode: '{mode_name}'")
            return
        logger.info(f"Mode set -> tgt={target_system_id}, mode={mode_name}, id={mode_id}")
        
        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )
        time.sleep(0.05)

    def arm(self, target_system_id: int, timeout: float = 15.0,
            retry_interval: float = 2.0) -> bool:
        """
        Drone'u arm eder ve gerçekten arm olduğunu doğrular.

        param2 = 0 → prearm güvenlik kontrolleri (GPS/EKF/pusula/batarya)
        UYGULANIR (force-arm YOK). Kontroller geçmezse dron arm olmaz.

        Doğrulama:
          * Birincil: HEARTBEAT'teki SAFETY_ARMED bayrağı (kayıplı linkte bile
            sürekli gelir, en güvenilir onay).
          * İkincil: COMMAND_ACK — reddedilme sebebini loglamak için kullanılır.

        Başarıda True, timeout/red durumunda False döner.
        """
        ack_q = self._command_ack_queues.get(target_system_id)
        # Eski ACK'leri temizle ki önceki komutların cevabını okumayalım
        if ack_q is not None:
            while True:
                try: ack_q.get_nowait()
                except queue.Empty: break

        logger.info(f"Arm -> {target_system_id} (prearm kontrolleri aktif)")
        deadline    = time.time() + timeout
        last_send   = 0.0
        denied_seen = False

        while time.time() < deadline:
            now = time.time()
            # Komutu periyodik olarak tekrar gönder (paket kaybına ve prearm'ın
            # sonradan düzelmesine — ör. GPS fix — karşı dayanıklılık).
            if now - last_send >= retry_interval:
                self.master.mav.command_long_send(
                    target_system_id, 0,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0,   # param1=1 (arm), param2=0 (force YOK)
                )
                last_send = now

            # 1) Birincil onay: heartbeat armed bayrağı
            if self.drone_armed.get(target_system_id):
                logger.info(f"Drone {target_system_id}: ARMED onaylandı (heartbeat).")
                log_event("arm_confirmed", drone_id=target_system_id)
                return True

            # 2) COMMAND_ACK — sonucu/sebebi logla
            if ack_q is not None:
                try:
                    ack = ack_q.get(timeout=0.5)
                    res = ack.result
                    if res == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        logger.info(f"Drone {target_system_id}: arm komutu kabul edildi (ACK) — "
                                    f"heartbeat onayı bekleniyor.")
                    elif res in (mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                                 mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                        logger.warning(f"Drone {target_system_id}: arm geçici reddedildi "
                                       f"(result={res}) — tekrar denenecek.")
                    else:
                        if not denied_seen:
                            logger.error(f"Drone {target_system_id}: ARM REDDEDİLDİ (result={res}) "
                                         f"— prearm kontrolü başarısız olabilir (GPS/EKF/batarya).")
                            log_event("arm_denied", drone_id=target_system_id, result=int(res))
                            denied_seen = True
                except queue.Empty:
                    pass
            else:
                time.sleep(0.2)

        logger.error(f"Drone {target_system_id}: {timeout:.0f}s içinde ARM doğrulanamadı.")
        log_event("arm_timeout", drone_id=target_system_id)
        return False

    def takeoff(self, target_system_id: int, altitude: float):
        logger.info(f"Takeoff -> {target_system_id} alt={altitude}")
        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,
        )
        time.sleep(0.05)

    def land(self, target_system_id: int):
        logger.info(f"LAND -> {target_system_id}")
        self.master.mav.command_long_send(
            target_system_id, 0,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.05)

    def goto_global_int(self, target_system_id: int, lat: float, lon: float, alt: float):
        logger.info(f"GOTO_GLOBAL_INT -> tgt={target_system_id} lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}")
        self.master.mav.set_position_target_global_int_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def goto_local_ned(self, target_system_id: int, north: float, east: float, down: float):
        logger.debug(f"GOTO_LOCAL_NED -> tgt={target_system_id} NED=({north:.2f},{east:.2f},{down:.2f})")
        self.master.mav.set_position_target_local_ned_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            north, east, down,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    # DÜZELTME: hedef nokta ORTAK NED çerçevesinde. MAV_FRAME_LOCAL_NED her araç
    # tarafından KENDİ EKF orijinine göre yorumlanır; bu yüzden aynı (n,e,d)
    # means a different physical point on every drone -> followers flew toward
    # slotları (kendi_orijini - lider_orijini) kadar kayıyordu, yani yanlış yöne.
    # Burada ortak çerçevedeki hedef tekrar lat/lon/alt(AMSL)'ye çevrilip
    # MAV_FRAME_GLOBAL_INT ile SET_POSITION_TARGET_GLOBAL_INT olarak gönderilir;
    # bu orijinden bağımsızdır: her araç aynı fiziksel noktaya uçar.
    def goto_shared_ned(self, target_system_id: int, north: float, east: float, down: float,
                        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                        ax: float = 0.0, ay: float = 0.0, az: float = 0.0,
                        yaw: float = 0.0, yaw_rate: float = 0.0,
                        type_mask: int = POSITION_ONLY_MASK):
        """
        Ortak NED çerçevesindeki bir hedefi araca gönderir. Konum, orijinden
        bağımsız olsun diye lat/lon/alt(AMSL)'ye çevrilip GLOBAL_INT ile yollanır.

        İLERİ-BESLEME (feedforward): hız/ivme/yaw alanları ve type_mask varsayılan
        olarak KONUM-YALNIZ'dır (mask == POSITION_ONLY_MASK), böylece takipçi ve
        saldırgan çağrıları (yalnız n,e,d geçen) davranışı DEĞİŞMEZ. Lider slotu
        okuyucusu, simple_guided_follow'un yayınladığı vx/vy/vz + ax/ay/az + yaw +
        type_mask'i buraya geçirir; --no-position-only modu böylece gerçekten
        hız/ivme ileri-beslemeli bir uçuş profili üretir (aksi halde bayrak
        işlevsizdir: her iki mod da konum-yalnız uçar).

        NOT (çerçeve): hız/ivme NED (Kuzey/Doğu/Aşağı) eksenlerinde ve orijinden
        bağımsız olduğundan, ortak çerçevede yayınlanan vx/vy/vz doğrudan
        GLOBAL_INT mesajına geçirilebilir — yalnızca KONUM'un dönüştürülmesi
        gerekir. type_mask bit anlamları LOCAL_NED ile _GLOBAL_INT'te aynıdır.
        Publisher'ın 'coordinate_frame' alanı (MAV_FRAME_LOCAL_NED) burada
        KULLANILMAZ; global gönderim için MAV_FRAME_GLOBAL_INT sabittir.
        """
        if self._ned_origin is None:
            logger.warning(
                f"goto_shared_ned: NED origin henüz yok, komut atlandı (tgt={target_system_id})."
            )
            return
        lat, lon, alt = ned_to_latlon(north, east, down, *self._ned_origin)
        logger.debug(
            f"GOTO_SHARED_NED -> tgt={target_system_id} NED=({north:.2f},{east:.2f},{down:.2f}) "
            f"=> lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m AMSL mask={type_mask}"
        )
        self.master.mav.set_position_target_global_int_send(
            0, target_system_id, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,   # alt = AMSL (origin_alt - down)
            int(type_mask),                         # publisher'ın seçtiği mask (varsayılan konum-yalnız)
            int(lat * 1e7), int(lon * 1e7), alt,
            float(vx), float(vy), float(vz),        # hız ileri-besleme (NED m/s)
            float(ax), float(ay), float(az),        # ivme ileri-besleme (NED m/s^2)
            float(yaw), float(yaw_rate),            # yaw [rad], yaw_rate [rad/s]
        )

    def _goto_with_yaw_lock(self, drone_id, north, east, down, lock_yaw=None):
        """
        goto_shared_ned sarmalayıcısı: yaw kilidi varsa donmuş baş açısını
        POSITION_YAW_MASK ile birlikte komutlar, yoksa hiçbir şey değişmez
        (konum-yalnız, eski davranış).

        lock_yaw None geçilirse kilit durumu buradan okunur; böylece çağıranın
        (formasyon döngüsü gibi) ayrıca sorgulaması gerekmez.
        """
        if lock_yaw is None:
            lock_yaw = self._locked_yaw(drone_id)
        if lock_yaw is None:
            self.goto_shared_ned(drone_id, north, east, down)
        else:
            self.goto_shared_ned(drone_id, north, east, down,
                                 yaw=lock_yaw, yaw_rate=0.0,
                                 type_mask=POSITION_YAW_MASK)

    # ------------------------------------------------------------------
    # Waypoint varış beklemesi (MINE — FRIEND'in guided-mode korumasıyla)
    # Dispatcher'dan paket çalmamak için _nav_queues'dan okur.
    # ------------------------------------------------------------------
    def wait_for_waypoint_arrival(self, target_id, radius=3.0, timeout=60):
        logger.info(f"WP bekleme -> drone={target_id}, radius={radius}, timeout={timeout}")
        start   = time.time()
        q       = self._nav_queues[target_id]
        while time.time() - start < timeout:
            if not self.is_in_guided_mode(target_id):
                logger.warning(f"Drone {target_id} WP beklerken GUIDED moddan çıktı. İptal.")
                return False
            try:
                msg = q.get(timeout=1.0)
                d   = msg.wp_dist
                logger.info(f"[WP] drone={target_id} mesafe={d:.2f} m")
                if d <= radius:
                    logger.info(f"Drone {target_id} WP'ye ulaştı.")
                    return True
            except queue.Empty:
                pass
        logger.warning(f"Drone {target_id} WP timeout.")
        return False

    # ------------------------------------------------------------------
    # İrtifa beklemesi  
    # ------------------------------------------------------------------
    def wait_for_takeoff(
        self,
        target_sysid: int,
        target_alt_m: float,
        threshold: float = 0.85,
        timeout: float   = 60.0,
    ) -> bool:
        needed   = target_alt_m * threshold
        deadline = time.time() + timeout
        q        = self._gpos_queues[target_sysid]
        logger.info(f"Drone {target_sysid}: climbing to {needed:.1f} m...")

        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                alt = msg.relative_alt / 1000.0
                logger.debug(f"Drone {target_sysid}: {alt:.1f} m")
                if alt >= needed:
                    logger.info(f"Drone {target_sysid}: airborne at {alt:.1f} m [OK]")
                    return True
            except queue.Empty:
                pass

        logger.warning(f"Drone {target_sysid}: did not reach altitude in time.")
        return False

    # ------------------------------------------------------------------
    # Paralel kalkış — drone başına irtifa için build_takeoff_altitudes kullanır (MINE + FRIEND)
    # ------------------------------------------------------------------
    def parallel_launch(self, drone_ids: list, takeoff_altitudes: dict):
        """
        Tüm drone'lar aynı anda kalkış yapar.
        takeoff_altitudes: dict[drone_id -> altitude_m]  (build_takeoff_altitudes'tan)
        """
        results = {}
        barrier = threading.Barrier(len(drone_ids))

        def launch_one(drone_id):
            alt = takeoff_altitudes[drone_id]

            # GUIDED iste
            self.set_mode(drone_id, "GUIDED")
            guided = self._wait_for_guided(drone_id, timeout=15)
            if not guided:
                logger.warning(f"Drone {drone_id}: GUIDED not confirmed, continuing.")

            # Arm — prearm kontrolleri aktif, sonuç doğrulanır
            logger.info(f"Drone {drone_id}: arming...")
            if not self.arm(drone_id):
                logger.error(f"Drone {drone_id}: ARM başarısız — kalkış İPTAL edildi.")
                results[drone_id] = False
                barrier.abort()   # bekleyen diğer thread'leri serbest bırak
                return

            # Kalkış
            logger.info(f"Drone {drone_id}: takeoff to {alt} m...")
            self.takeoff(drone_id, alt)

            # İrtifa beklemesi
            ok = self.wait_for_takeoff(drone_id, alt, threshold=0.85, timeout=60)
            results[drone_id] = ok

            # İsteğe bağlı RTB kullanımı için kalkış NED konumunu sakla
            with self.lock:
                pos = self.drone_positions.get(drone_id, (0, 0, 0))
                self.takeoff_positions[drone_id] = (pos[0], pos[1], -alt)

            logger.info(f"Drone {drone_id}: ready, waiting at barrier...")
            try:
                barrier.wait(timeout=90)
            except threading.BrokenBarrierError:
                logger.warning(f"Drone {drone_id}: barrier broken — continuing.")

        threads = []
        for drone_id in drone_ids:
            t = threading.Thread(target=launch_one, args=(drone_id,), daemon=True)
            t.start()
            threads.append(t)
            time.sleep(0.3)   # mod komutlarını kademelendir

        for t in threads:
            t.join(timeout=120)

        failed    = [did for did in drone_ids if not results.get(did)]
        succeeded = [did for did in drone_ids if results.get(did)]

        if not failed:
            logger.info("All drones airborne [OK]")
            return

        logger.warning(
            f"Kalkışta başarısız (arm/irtifa): {failed} — bu drone'lar görevden "
            f"çıkarılıyor (RTL), sürü onlarsız devam ediyor."
        )

        # Kullanıcı isteği: bir takipçi VEYA lider kalkışta başarısız olsa bile
        # görev o drone olmadan devam etmeli (heartbeat kaybındaki RTL+devam ile
        # aynı). Lider başarısızsa, hayatta kalanlardan SUCCESSION_ORDER'a göre
        # yeni bir lider seçilir; hiç uygun lider yoksa gerçekten iptal edilir.
        if self.leader_id in failed:
            new_ldr = next_leader(SUCCESSION_ORDER, set(succeeded), self.leader_id)
            if new_ldr is None:
                for did in failed:
                    self._ground_drone(did, reason="launch_failed")
                raise RuntimeError(
                    f"Lider {self.leader_id} ve tüm yedekler kalkışta başarısız — "
                    f"uçacak drone yok, görev iptal ediliyor."
                )
            logger.warning(
                f"[Launch] Lider {self.leader_id} kalkışta başarısız -> yeni lider "
                f"{new_ldr} (hayatta: {sorted(succeeded)})."
            )
            log_event("leader_reassigned_launch", old_leader=self.leader_id,
                      new_leader=new_ldr, alive=sorted(succeeded))
            self.leader_id = new_ldr

        # Başarısız drone'ların hepsini görevden çıkar (RTL + kalıcı hariç tutma).
        for did in failed:
            self._ground_drone(did, reason="launch_failed")

    def _ground_drone(self, drone_id: int, reason: str = ""):
        """
        Bir drone'u görevden ÇIKARIR: RTL'e alır ve sürünün geri kalanı onsuz
        devam eder. Heartbeat kaybındaki davranışın (RTL + devam) kalkış-hatası
        karşılığıdır. Grounded drone bir daha 'hayatta' sayılmaz; formasyon,
        lider seçimi, saldırgan seçimi ve konum bekçisi onu atlar.
        """
        logger.warning(f"[Ground] Drone {drone_id} görevden çıkarılıyor ({reason}) "
                       f"-> RTL, sürü onsuz devam ediyor.")
        log_event("drone_grounded", drone_id=drone_id, reason=reason)
        self._grounded_ids.add(drone_id)
        self._alive_ids.discard(drone_id)
        self.set_mode(drone_id, "RTL")

    # ------------------------------------------------------------------
    # Formasyon hesaplaması (MINE — yaw-rotasyonlu + lookahead)
    # ------------------------------------------------------------------
    def _slots_symmetric(self, n: int) -> list:
        """HORIZONTAL_LINE için simetrik slot üreteci  ."""
        slots = []
        k = 1
        while len(slots) < n:
            if len(slots) < n: slots.append(-k)
            if len(slots) < n: slots.append(k)
            k += 1
        return slots

    def _compute_all_slots(
        self,
        leader_pos,
        leader_yaw,
        leader_id,
        formation,
        offset,
        vertical_offsets,
        ):
        """
        Tüm takipçi slotlarının NED pozisyonlarını hesaplar.

        Ofsetler lider gövde çerçevesinde (Forward, Right, Down) tanımlanır
        ve ardından yalnızca yaw rotasyonu ile dünya NED çerçevesine döndürülür.
        Lookahead da gövde çerçevesinde uygulanır — bu, liderin dönüşlerinde
        formasyonun simetrisini korur (global NED lookahead'i dönüşlerde
        formasyonu asimetrik şekilde bozuyordu).
        """
        leader_vx, leader_vy, _ = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        lookahead_time = 2.5

        # Lookahead'i gövde çerçevesine dönüştür (yaw rotasyonunun tersi)
        # Bu, lider dönerken merkez noktasının her zaman lider burnunda kalmasını sağlar
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        # Global NED hızı → gövde çerçevesi hızı (yaw'ın tersi = yaw transpozesi)
        vx_body =  leader_vx * cos_yaw + leader_vy * sin_yaw   # ileri
        vy_body = -leader_vx * sin_yaw + leader_vy * cos_yaw   # sağ

        # Lookahead gövde çerçevesinde uygulanır
        fwd_lookahead = vx_body * lookahead_time
        rgt_lookahead = vy_body * lookahead_time

        followers = sorted([fid for fid in self.drone_ids if fid != leader_id])
        n         = len(followers)

        # Gövde çerçevesi slot tanımları: (forward, right, down_delta)
        # forward > 0 = lider önünde, right > 0 = lider sağında, down_delta > 0 = liderin altında
        body_slots = []

        if formation == Formation.LINE:
            for i in range(n):
                slot = i - (n - 1) / 2.0
                # LINE: dronlar lider etrafında sağa sola dizilir (right ekseninde)
                body_slots.append((-slot * offset, 0.0, 0.0))

        elif formation == Formation.HORIZONTAL_LINE:
            if self.symmetric_horizontal:
                sym_slots = self._slots_symmetric(n)
                for s in sym_slots:
                    body_slots.append((0.0, s * offset, 0.0))
            else:
                for i in range(n):
                    slot = i - (n - 1) / 2.0
                    body_slots.append((0.0, slot * offset, 0.0))

        elif formation == Formation.V_SHAPE:
            for i in range(n):
                v_slot = i + 1
                side   = -1 if v_slot % 2 == 0 else 1
                fwd    = -((v_slot // 2 + 1) * offset * 0.7)
                rgt    = ((v_slot // 2 + 1) * offset * 0.7) * side
                body_slots.append((fwd, rgt, 0.0))

        elif formation == Formation.DIAMOND:
            # Yatay slotlar (sağ/sol): gövde çerçevesi right ekseni
            # Dikey slotlar (üst/alt): NED Z ekseni — VERTICAL_OFFSETS ile kontrol edilir
            positive_offsets = [v for v in vertical_offsets.values() if v > 0]
            negative_offsets = [v for v in vertical_offsets.values() if v < 0]
            top_gap    = max(positive_offsets) if positive_offsets else offset
            bottom_gap = abs(min(negative_offsets)) if negative_offsets else offset
            body_slots = [
                (0.0,  offset, 0.0),          # sağ
                (0.0, -offset, 0.0),          # sol
                (0.0,  0.0,   -top_gap),      # üst  (NED down negatif = yukarı)
                (0.0,  0.0,    bottom_gap),   # alt
            ]

        # Gövde çerçevesi → dünya NED dönüşümü (yalnızca yaw)
        # Rotasyon matrisi (sadece yaw):
        #   NED_north = fwd * cos(yaw) - rgt * sin(yaw)
        #   NED_east  = fwd * sin(yaw) + rgt * cos(yaw)
        #   NED_down  = down_delta (dikey, yaw'dan etkilenmez)
        result = []
        for (fwd, rgt, down_delta) in body_slots:
            # Lookahead'i gövde ofsetiyle birleştir, sonra birlikte döndür
            total_fwd = fwd + fwd_lookahead
            total_rgt = rgt + rgt_lookahead

            ned_n = total_fwd * cos_yaw - total_rgt * sin_yaw
            ned_e = total_fwd * sin_yaw + total_rgt * cos_yaw
            ned_d = leader_pos[2] + down_delta   # dikey: global NED Z, yaw'dan bağımsız

            result.append((leader_pos[0] + ned_n, leader_pos[1] + ned_e, ned_d))

        return result, followers

    def _assign_slots(self, followers, slot_positions, vertical_offsets, leader_d):
        """
        Her takipçiyi en yakın boş slota atar — hysteresis ile.
        Mevcut atama, yeni en iyi slottan SLOT_REASSIGN_THRESHOLD_M'den
        daha yakın değilse korunur (slot değişimlerini önler).
        Döndürür: {follower_id: slot_index}
        """
        threshold  = self._slot_reassign_threshold_m
        n          = len(followers)
        assignment = {}   # follower_id -> slot_index (bu tick için yeni atama)
        used_slots = set()

        # 1. Geçici en iyi atamayı hesapla (her drone için en yakın serbest slot)
        best_for = {}   # follower_id -> (dist, slot_index)
        for fid in followers:
            with self.lock:
                pos = self.drone_positions.get(fid)
            if pos is None:
                best_for[fid] = (float('inf'), 0)
                continue
            dists = []
            for si, (sn, se, sd) in enumerate(slot_positions):
                dn = pos[0] - sn
                de = pos[1] - se
                dd = pos[2] - sd
                dists.append((math.sqrt(dn*dn + de*de + dd*dd), si))
            dists.sort()
            best_for[fid] = dists[0]   # (min_dist, best_slot_index)

        # 2. Hysteresis: mevcut atama geçerliyse ve yeni en iyi slot yeterince
        #    daha iyi değilse mevcut atamayı koru
        locked = {}   # follower_id -> slot_index (korunan atamalar)
        for fid in followers:
            current_slot = self._slot_assignments.get(fid)
            if current_slot is not None and current_slot < n:
                sn, se, sd = slot_positions[current_slot]
                with self.lock:
                    pos = self.drone_positions.get(fid)
                if pos is not None:
                    dn   = pos[0] - sn
                    de   = pos[1] - se
                    dd = pos[2] - sd
                    curr_dist = math.sqrt(dn*dn + de*de + dd*dd)
                    best_dist, best_slot = best_for[fid]
                    # Sadece yeni slot, mevcut slottan threshold kadar daha yakınsa değiştir
                    if best_slot != current_slot and (curr_dist - best_dist) < threshold:
                        locked[fid] = current_slot   # yeterince iyi değil, mevcut atamayı koru

        # 3. Önce kilitli atamaları uygula
        for fid, si in locked.items():
            if si not in used_slots:
                assignment[fid] = si
                used_slots.add(si)

        # 4. Kilitli olmayan drone'ları en yakın serbest slota ata (greedy)
        unassigned = [fid for fid in followers if fid not in assignment]
        # Mesafeye göre sırala: en yakın drone önce atar
        unassigned.sort(key=lambda fid: best_for[fid][0])
        for fid in unassigned:
            best_dist, best_slot = best_for[fid]
            if best_slot not in used_slots:
                assignment[fid] = best_slot
                used_slots.add(best_slot)
            else:
                # Tercih edilen slot alınmış — en yakın serbest slotu bul
                with self.lock:
                    pos = self.drone_positions.get(fid)
                fallback = None
                fallback_dist = float('inf')
                for si, (sn, se, sd) in enumerate(slot_positions):
                    if si in used_slots:
                        continue
                    if pos is not None:
                        dn = pos[0] - sn
                        de = pos[1] - se
                        dd = pos[2] - sd
                        d  = math.sqrt(dn*dn + de*de + dd*dd)
                    else:
                        d = float('inf')
                    if d < fallback_dist:
                        fallback_dist = d
                        fallback = si
                if fallback is not None:
                    assignment[fid] = fallback
                    used_slots.add(fallback)

        # Atamayı önbelleğe kaydet — değişen atamaları olay olarak kaydet
        for fid, si in assignment.items():
            prev = self._slot_assignments.get(fid)
            if prev != si:
                log_event(
                    "slot_reassignment",
                    follower_id=fid, previous_slot=prev, new_slot=si,
                )
        self._slot_assignments.update(assignment)
        return assignment

    def _assign_slots_diamond(self, followers, slot_positions, vertical_offsets):
        """
        DIAMOND formasyonu için özel slot atama.
        Slot 0: sağ,  Slot 1: sol  (yatay — düşük dikey ofset)
        Slot 2: üst,  Slot 3: alt  (dikey — yüksek dikey ofset)

        Atama mantığı:
        1. Drone'ları |vertical_offset| büyüklüğüne göre sırala.
        2. En büyük 2 |offset| → dikey slotlar (2=üst, 3=alt).
        Hangisinin üste hangisinin alta gideceğini drone'un mevcut
        Z konumuna göre belirle (daha yukarıda olan → üst slota).
        3. Kalan 2 drone → yatay slotlar (0=sağ, 1=sol).
        Hangisinin sağa hangisinin sola gideceğini drone'un mevcut
        yatay konumuna göre belirle (slot'a olan 2D mesafe ile).
        """
        assignment = {}

        # Drone'ları |vertical_offset|'e göre büyükten küçüğe sırala
        sorted_by_voffset = sorted(
            followers,
            key=lambda fid: abs(vertical_offsets.get(fid, 0.0)),
            reverse=True,
        )

        vertical_drones   = sorted_by_voffset[:2]   # en büyük 2 dikey ofset → üst/alt
        horizontal_drones = sorted_by_voffset[2:]   # kalan → sağ/sol

        # --- Dikey drone'ları üst (slot 2) ve alt (slot 3) olarak ata ---
        # Mevcut Z konumuna bak: daha negatif Z (daha yukarıda) → üst slota
        slot2_pos = slot_positions[2]   # üst slot NED pozisyonu
        slot3_pos = slot_positions[3]   # alt slot NED pozisyonu

        d0, d1 = vertical_drones[0], vertical_drones[1]
        with self.lock:
            pos0 = self.drone_positions.get(d0)
            pos1 = self.drone_positions.get(d1)

        # Pozisyon bilinmiyorsa dikey offsetin işaretini kullan (+ = yukarı = slot 2)
        if pos0 is None or pos1 is None:
            for fid in vertical_drones:
                v = vertical_offsets.get(fid, 0.0)
                assignment[fid] = 2 if v > 0 else 3
        else:
            # Her iki drone için slot 2 ve slot 3'e toplam mesafeyi karşılaştır
            # ve en iyi çifti seç (minimum toplam mesafe ataması)
            def dist3(pos, slot):
                return math.sqrt(sum((pos[i] - slot[i])**2 for i in range(3)))

            cost_0to2 = dist3(pos0, slot2_pos) + dist3(pos1, slot3_pos)
            cost_0to3 = dist3(pos0, slot3_pos) + dist3(pos1, slot2_pos)

            if cost_0to2 <= cost_0to3:
                assignment[d0] = 2
                assignment[d1] = 3
            else:
                assignment[d0] = 3
                assignment[d1] = 2

        # --- Yatay drone'ları sağ (slot 0) ve sol (slot 1) olarak ata ---
        if len(horizontal_drones) >= 2:
            h0, h1 = horizontal_drones[0], horizontal_drones[1]
            slot0_pos = slot_positions[0]
            slot1_pos = slot_positions[1]

            with self.lock:
                hpos0 = self.drone_positions.get(h0)
                hpos1 = self.drone_positions.get(h1)

            if hpos0 is None or hpos1 is None:
                assignment[h0] = 0
                assignment[h1] = 1
            else:
                def dist2(pos, slot):
                    return math.sqrt((pos[0]-slot[0])**2 + (pos[1]-slot[1])**2)

                cost_0to0 = dist2(hpos0, slot0_pos) + dist2(hpos1, slot1_pos)
                cost_0to1 = dist2(hpos0, slot1_pos) + dist2(hpos1, slot0_pos)

                if cost_0to0 <= cost_0to1:
                    assignment[h0] = 0
                    assignment[h1] = 1
                else:
                    assignment[h0] = 1
                    assignment[h1] = 0
        elif len(horizontal_drones) == 1:
            assignment[horizontal_drones[0]] = 0

        # Önbelleğe kaydet ve slot değişimlerini logla
        for fid, si in assignment.items():
            prev = self._slot_assignments.get(fid)
            if prev != si:
                log_event("slot_reassignment", follower_id=fid, previous_slot=prev, new_slot=si)
        self._slot_assignments.update(assignment)
        return assignment

    def calculate_formation_target(
        self,
        leader_pos,
        leader_yaw,
        follower_id,
        leader_id,
        formation,
        offset,
        vertical_offsets,
        min_alt_m: float = 10.0,
        ):
        """
        Bir takipçi için NED hedefini hesaplar.
        Slot ataması en yakın slota göre yapılır; hysteresis ile
        gereksiz slot değişimleri önlenir.
        """
        slot_positions, followers = self._compute_all_slots(
        leader_pos, leader_yaw, leader_id, formation, offset, vertical_offsets
        )
        leader_d = leader_pos[2]
        if formation == Formation.DIAMOND:
            assignment = self._assign_slots_diamond(followers, slot_positions, vertical_offsets)
        else:
            assignment = self._assign_slots(followers, slot_positions, vertical_offsets, leader_d)

        si = assignment.get(follower_id)
        if si is None:
            target_d = leader_pos[2]
            sn, se = leader_pos[0], leader_pos[1]
        else:
            sn, se, _ = slot_positions[si]
            v_offset = vertical_offsets.get(follower_id, 0.0)
            target_d = leader_d - v_offset

        # --- Güvenlik: takipçi irtifası min_alt_m'nin altına düşemez ---
        min_d = -abs(min_alt_m)          # NED z tavanı (en fazla bu kadar negatif olmayan taraf)
        safe_d = min(target_d, min_d)    # gerçek irtifa (=-z) her zaman >= min_alt_m
        if safe_d != target_d:
            logger.debug(
                f"[FollowSafety] f={follower_id}: hedef irtifa {-target_d:.1f}m, "
                f"{min_alt_m:.1f}m tabanına yükseltiliyor."
            )

        return (sn, se, safe_d)



    # ------------------------------------------------------------------
    # Takipçi thread'i (MINE mantığı + FRIEND'in follow_enable + guided koruması)
    # ------------------------------------------------------------------
    def _formation_manager_thread(self, formation, offset, vertical_offsets, min_alt_m=10.0):
        """
        TEK formasyon yöneticisi (takipçi başına thread'lerin yerine geçer).

        Neden tek thread: eski tasarımda her takipçi kendi thread'inde GLOBAL slot
        atamasını yeniden çalıştırıyor ve self._slot_assignments'ı kilit almadan
        değiştiriyordu. 2 Hz'de yarışan dört thread, iki yan drone'u anlık olarak
        aynı tarafa atayabiliyordu. Burada TEK thread, her tikte tutarlı tek bir
        anlık görüntüden bütün takipçilerin hedefini bir kez hesaplar.

        Lider değişebilir: her tikte self.leader_id okunur; seçim sonrası formasyon
        yeniden başlatmaya gerek kalmadan yeni liderin etrafına oturur.

        Serbest seyir (coast): lider konumu kısa süre bayatsa (< LEADER_LOITER_S),
        donmak yerine son hızıyla ekstrapole edilir — nihai LOITER kararı lider
        konum bekçisinindir.

        Saldırgan farkındalığı: göreve atanmış saldırgan (varsa) burada atlanır;
        onun güdümü süreç-içi kesişme thread'ine aittir.
        """
        logger.info(f"Formasyon yöneticisi başlatıldı -> form={formation.name}")
        last_log = 0.0
        send_dt  = 0.1    # 10 Hz komut hızı (daha sıkı takip, daha az gecikme)
        while not self.stop_threads.is_set():
            try:
                if self._shutting_down.is_set():
                    logger.info("[FORM] Kapanış algılandı — formasyon komutları durduruldu.")
                    return
                if not self.follow_enable.is_set():
                    time.sleep(0.2)
                    continue

                leader_id = self.leader_id   # değişebilir — her tikte yeniden oku

                with self.lock:
                    leader_pos = self.drone_positions.get(leader_id)
                    leader_yaw = self.drone_headings.get(leader_id, 0.0)
                    leader_vel = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
                    last_pos_t = self._last_leader_pos_time

                # --- Coast: 0.3..COAST arası son hızla ekstrapole et ---------
                # COAST..POS_LOITER arası ekstrapolasyon YOK (son konum tutulur);
                # POS_LOITER ötesinde konum bekçisi zaten LOITER'a almıştır.
                pos_age = time.monotonic() - last_pos_t
                if leader_pos and 0.3 < pos_age <= LEADER_COAST_S:
                    leader_pos = (
                        leader_pos[0] + leader_vel[0] * pos_age,
                        leader_pos[1] + leader_vel[1] * pos_age,
                        leader_pos[2] + leader_vel[2] * pos_age,
                    )

                if not leader_pos:
                    time.sleep(send_dt)
                    continue

                # Aktif takipçiler = lider, göreve atanmış saldırgan ve kalkışta
                # görevden çıkarılan (grounded) drone'lar hariç herkes
                # (saldırganın güdümü ayrı thread'e ait).
                followers = [fid for fid in self.drone_ids
                             if fid != leader_id
                             and fid != self._committed_attacker
                             and fid not in self._grounded_ids]

                # --- CANLI formasyon ofseti ----------------------------------
                # 'offset' argümanı yalnızca başlangıç değeridir; gerçek değer
                # her tikte guidance_config.FORMATION_OFFSET_M'den okunur, o
                # yüzden uçuş sırasında dosyayı kaydetmek yeter (yeniden
                # başlatma yok). Rampalıdır: 10 -> 25 m'lik bir düzenleme her
                # takipçinin hedefini tek karede 15 m kaydırmaz, saniyede
                # FORMATION_OFFSET_SLEW_MPS kadar kaydırır -- yoksa hepsi aynı
                # anda tam gaz yana atar ve slot ataması (_assign_slots) da
                # aynı karede yeniden karışabilir.
                offset_live = FORMATION_OFFSET_LIVE.value()
                self._current_offset = offset_live   # /state yayınında görünsün

                # Tüm hedefleri tek ve tutarlı bir geçişte hesapla.
                targets = self._compute_formation_targets(
                    leader_pos, leader_yaw, leader_id, followers,
                    formation, offset_live, vertical_offsets, min_alt_m,
                )

                for fid, tgt in targets.items():
                    if not self.is_in_guided_mode(fid):
                        continue
                    # Geçişten sonra formasyona BIRAKILMIŞ ama hâlâ hızlı olan
                    # saldırgan burada da kilitli yaw ile uçar: slotu genelde
                    # ARKADA olduğu için normalde tam gaz 180° dönerdi.
                    self._goto_with_yaw_lock(fid, *tgt)

                now = time.time()
                if now - last_log > 2.0:
                    logger.info(
                        f"[FORM] leader={leader_id} "
                        f"n_follow={len(followers)} "
                        f"offset={offset_live:.1f}m "
                        f"lead={tuple(round(x,1) for x in leader_pos)} "
                        f"coast_age={pos_age:.1f}s"
                    )
                    last_log = now

                time.sleep(send_dt)

            except Exception:
                logger.exception("Formasyon yöneticisi thread hatası.")
                time.sleep(0.5)

    def _compute_formation_targets(self, leader_pos, leader_yaw, leader_id,
                                   followers, formation, offset, vertical_offsets,
                                   min_alt_m):
        """
        Tüm aktif takipçilerin NED hedeflerini TEK tutarlı geçişte hesaplar.
        DIAMOND için sayıya toleranslı gövde-slot tablosunu ve tek bir
        en-yakın-slot atamasını kullanır. Döner: {takipci_id: (n, e, d)}.
        """
        n = len(followers)
        if n == 0:
            self._last_formation_targets = {}
            return {}

        if formation == Formation.DIAMOND:
            body_slots = diamond_body_slots(
                n, offset,
                DIAMOND_TOP_GAP_M, DIAMOND_BOTTOM_GAP_M,
                DIAMOND_LATERAL_STAGGER, DIAMOND_FORE_AFT_STAGGER,
            )
        else:
            # --- LINE / HORIZONTAL_LINE / V_SHAPE -----------------------------
            # Bu formasyonlarda dikey ayrımı VERTICAL_OFFSETS sözlüğü belirler.
            #
            # DÜZELTME (dikey ofset uygulanmıyordu): _compute_all_slots bu üç
            # formasyon için gövde slotlarını down_delta=0.0 ile üretir, yani
            # slotun z'si HER ZAMAN liderin irtifasına eşittir. Dikey ofset
            # slotun İÇİNDE DEĞİLDİR — orijinal calculate_formation_target()
            # bunu slot atamasından SONRA, drone başına uyguluyordu:
            #       sn, se, _ = slot_positions[si]      # slot z ATILIR
            #       v_offset  = vertical_offsets.get(follower_id, 0.0)
            #       target_d  = leader_d - v_offset     # drone'un kendi ofseti
            # Tek-thread'li yeniden yazımda bu adım düşmüş ve slotun z'si
            # (== lider irtifası) doğrudan kullanılmıştı; sonuçta TÜM takipçiler
            # liderle aynı irtifada uçuyordu. Aşağıda orijinal davranış geri
            # getirildi.
            #
            # İşaret kuralı (NED, aşağı pozitif): target_d = leader_d - v_offset
            #   v_offset > 0  ->  liderin ÜSTÜNDE
            #   v_offset < 0  ->  liderin ALTINDA
            # (DIAMOND kolu ile tutarlı: orada top_gap = max(pozitif ofsetler).)
            slot_positions, legacy_followers = self._compute_all_slots(
                leader_pos, leader_yaw, leader_id, formation, offset, vertical_offsets
            )
            assignment = self._assign_slots(
                legacy_followers, slot_positions, vertical_offsets, leader_pos[2]
            )
            leader_d = leader_pos[2]
            out = {}
            for fid in followers:
                si = assignment.get(fid)
                if si is None:
                    continue
                sn, se, _sd = slot_positions[si]   # slot z ATILIR (== leader_d)
                v_offset = vertical_offsets.get(fid, 0.0)
                target_d = leader_d - v_offset     # drone başına dikey ofset
                safe_d   = min(target_d, -abs(min_alt_m))   # min irtifa tabanı
                if safe_d != target_d:
                    logger.debug(
                        f"[FollowSafety] f={fid}: hedef irtifa {-target_d:.1f}m, "
                        f"{min_alt_m:.1f}m tabanına yükseltiliyor."
                    )
                out[fid] = (sn, se, safe_d)
            self._last_formation_targets = out
            return out

        # --- DIAMOND: gövde slotlarını dünya NED'ine döndür (yaw + ileri görüş) --
        leader_vx, leader_vy, _ = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        lookahead_time = 3.0
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        vx_body =  leader_vx * cos_yaw + leader_vy * sin_yaw
        vy_body = -leader_vx * sin_yaw + leader_vy * cos_yaw
        fwd_lookahead = vx_body * lookahead_time
        rgt_lookahead = vy_body * lookahead_time

        slot_positions = []
        for (fwd, rgt, down_delta) in body_slots:
            total_fwd = fwd + fwd_lookahead
            total_rgt = rgt + rgt_lookahead
            ned_n = total_fwd * cos_yaw - total_rgt * sin_yaw
            ned_e = total_fwd * sin_yaw + total_rgt * cos_yaw
            ned_d = leader_pos[2] + down_delta
            slot_positions.append((leader_pos[0] + ned_n,
                                   leader_pos[1] + ned_e, ned_d))

        # Histerezisli tek en-yakın-slot ataması (mevcut _assign_slots kullanılır;
        # açgözlü + histerezisli olup TEK thread'den çağrıldığında kilit-güvenlidir
        # — ki artık bu garanti).
        assignment = self._assign_slots(
            followers, slot_positions, vertical_offsets, leader_pos[2]
        )

        out = {}
        for fid in followers:
            si = assignment.get(fid)
            if si is None or si >= len(slot_positions):
                continue
            sn, se, sd = slot_positions[si]
            safe_d = min(sd, -abs(min_alt_m))   # asla minimum irtifanın altına inme
            out[fid] = (sn, se, safe_d)
        self._last_formation_targets = out
        return out

    def start_formation_following(self, leader_id, formation, offset, vertical_offsets,
                               slot_reassign_threshold_m: float = 5.0, follower_min_alt_m: float = 10.0):
        logger.info(
            f"Formasyon takibi başlıyor -> leader={leader_id}, "
            f"formation={formation.name}, offset={offset}, "
            f"slot_threshold={slot_reassign_threshold_m:.1f}m"
        )
        log_event(
            "formation_start", leader_id=leader_id, formation=formation.name,
            offset=offset, slot_reassign_threshold_m=slot_reassign_threshold_m,
        )
        self._slot_assignments = {}
        self._slot_reassign_threshold_m = slot_reassign_threshold_m
        self._current_formation = formation.name
        self._current_offset    = offset
        self.follow_enable.set()
        # TEK yönetici thread (eskiden takipçi başına bir thread vardı).
        t = threading.Thread(
            target=self._formation_manager_thread,
            args=(formation, offset, vertical_offsets),
            kwargs={"min_alt_m": follower_min_alt_m},
            name="FormationManager",
            daemon=True,
        )
        t.start()
        self.threads.append(t)

    def pause_formation_following(self):
        """Durdurmadan tüm takipçi thread'lerini duraklatır  ."""
        logger.info("Formasyon takibi DURDURULDU.")
        log_event("formation_pause")
        self.follow_enable.clear()

    # ------------------------------------------------------------------
    # Lider slot kontrolcüsü thread'i  
    # ------------------------------------------------------------------
    def _leader_slot_reader_thread(
        self,
        redis_client,
        leader_id:          int,
        stale_timeout:       float = 2.0,
        # simple_guided_follow 20 Hz yayinliyor; 10 Hz'de her ikinci setpoint
        # atiliyordu (2026-07-30 loglari). Es rate -> her slot araca gidiyor.
        send_rate_hz:        float = 20.0,
        min_alt_m:           float = 10.0,
        loop_start_wp_index: int   = 2,
        loop_arrival_dist_m: float = 12.0,
        wp_speed_mps:        float = 10.0,
        ):
        import json
        REDIS_KEY   = "leader_slot_ned"
        sleep_dt    = 1.0 / max(send_rate_hz, 1.0)
        last_warn   = 0.0
        guided_mode = True   # lideri en son hangi moda BİZİM aldığımızı takip eder

        # NED Z negatif-yukarı: -min_alt_m, Z için tavan değeridir (en negatif = en yüksek)
        min_z_ned = -abs(min_alt_m)

        logger.info(
            f"Leader slot controller başladı (leader={leader_id}, key={REDIS_KEY}, "
            f"min_alt={min_alt_m:.1f}m, loop_start_wp={loop_start_wp_index}, "
            f"arrival_dist={loop_arrival_dist_m:.1f}m)"
        )

        def drain_mission_current():
            """En son MISSION_CURRENT seq'ini döndürür, kuyrukta hiçbir şey yoksa None."""
            q = self._mission_current_queues[leader_id]
            last_seq = None
            while True:
                try:
                    msg = q.get_nowait()
                    last_seq = int(msg.seq)
                except queue.Empty:
                    break
            return last_seq

        # Görev döngüsü durumu, her AUTO'ya (yeniden) girdiğimizde sıfırlanır
        mission_state = {
            "mission_count":     -1,
            "last_wp_index":     -1,
            "last_wp_ned":       None,
            "current_seq":       -1,
            "awaiting_arrival":  False,
            "parked_since":      None,   # son WP'de sabit kalmaya başladığı an
        }

        def enter_auto():
            """wait_for_endurance() kurulumunu yansıtır: görev sayısı + son WP koordinatlarını al."""
            self.set_mode(leader_id, "AUTO")
            mission_state["mission_count"] = self._get_mission_count(leader_id)
            mission_state["parked_since"] = None
            if mission_state["mission_count"] <= 0:
                # AUTO modda görev yoksa araç HİÇBİR ŞEY yapmaz (ya da eski,
                # bilinmeyen bir görevi uçar). Bu, "bilmediğim bir yere gitti"
                # durumunun klasik sebebidir — yüksek sesle uyar.
                logger.error(
                    f"[LeaderSlot] KRİTİK: Lider {leader_id} üzerinde AUTO görevi YOK "
                    f"(mission_count={mission_state['mission_count']}). Tarama rotası "
                    f"yüklenmemiş! Lider AUTO'da bekleyecek. Mission Planner ile "
                    f"tarama görevini TÜM drone'lara yükleyin."
                )
            if mission_state["mission_count"] > 0:
                mission_state["last_wp_index"] = mission_state["mission_count"] - 1
                origin = self._ned_origin
                wp = self._get_last_waypoint_global(leader_id, mission_state["mission_count"])
                if wp is not None and origin is not None:
                    wp_lat, wp_lon, wp_alt = wp
                    mission_state["last_wp_ned"] = latlon_to_ned(
                        wp_lat, wp_lon, wp_alt, *origin
                    )
            mission_state["awaiting_arrival"] = False
            drain_mission_current()

        while not self.stop_threads.is_set():
            try:
                # KAPANIŞ: iniş/kapanış başladıysa lidere ARTIK komut gönderme.
                # Aksi halde aşağıdaki "AUTO'ya geri al" mantığı RTL'i ezer.
                if self._shutting_down.is_set():
                    logger.info(
                        f"[LeaderSlot] Kapanış algılandı — lider {leader_id} "
                        f"kontrolü bırakılıyor (RTL/iniş serbest)."
                    )
                    return

                # LİDER SALDIRIDAYSA slot kontrolünü BIRAK. Aksi hâlde bu thread
                # 20 Hz'de leader_slot_ned setpoint'ini, saldırı thread'i ise
                # 10 Hz'de kesişme noktasını AYNI araca yollar; araç iki komut
                # arasında gidip gelir ve hedefi asla vuramaz. Saldırı bitince
                # (_release_attacker -> _committed_attacker = None) bu döngü
                # kendiliğinden kaldığı yerden devam eder; guided_mode False'a
                # çekildiği için GUIDED + varsayılan hız yeniden kurulur.
                if self._committed_attacker == leader_id:
                    if guided_mode:
                        logger.info(
                            f"[LeaderSlot] Lider {leader_id} SALDIRIDA — slot "
                            f"kontrolü saldırı thread'ine bırakıldı."
                        )
                        guided_mode = False
                    time.sleep(sleep_dt)
                    continue

                raw = redis_client.get(REDIS_KEY)
                now = time.monotonic()

                if raw:
                    data = json.loads(raw)
                    # DÜZELTME: 'ts' yoksa/geçersizse veri BAYAT sayılır.
                    # Eskiden data.get("ts", 0.0) ile age=now hesaplanıyordu; bu
                    # sessizce yanlış davranışa yol açabiliyordu.
                    _ts = data.get("ts", None)
                    try:
                        age = float("inf") if _ts is None else (now - float(_ts))
                    except (TypeError, ValueError):
                        age = float("inf")

                    if age < stale_timeout:
                        # --- Taze tahmin: GUIDED'i garanti et ve slot gönder ---
                        if not guided_mode:
                            logger.info(
                                f"[LeaderSlot] simple_guided_follow verisi döndü "
                                f"— lider {leader_id} GUIDED moduna alınıyor."
                            )
                            self.set_mode(leader_id, "GUIDED")
                            # Problem 2: AUTO'da gönderilen WP_SPEED (DO_CHANGE_SPEED)
                            # GUIDED'e taşınır. GUIDED'e dönünce lider, aracın KENDİ
                            # ArduPilot varsayılan hızında (MAX_SPEED tavanıyla) uçmalı.
                            self._set_speed(leader_id, self._default_speed_mps(leader_id))
                            guided_mode = True
                            drain_mission_current()

                        raw_z     = float(data["z"])
                        clamped_z = min(raw_z, min_z_ned)
                        if clamped_z != raw_z:
                            logger.debug(
                                f"[LeaderSlot] Z floor applied: {raw_z:.2f} → {clamped_z:.2f} "
                                f"(min_alt={min_alt_m:.1f}m)"
                            )
                        # DÜZELTME: Redis slotu (leader_slot_ned) simple_guided_follow
                        # tarafından ORTAK orijin çerçevesinde üretiliyor, ancak
                        # MAV_FRAME_LOCAL_NED (leader's own EKF frame). goto_shared_ned
                        # sends the equivalent global point instead.
                        #
                        # İLERİ-BESLEME: publisher'ın yazdığı vx/vy/vz + ax/ay/az +
                        # yaw + type_mask alanlarını AYNEN geçir. Böylece
                        # --no-position-only modu gerçekten hız/ivme ileri-beslemeli
                        # profil üretir. Eski/az alanlı payload'lar için varsayılanlar
                        # POSITION_ONLY_MASK'e düşer -> tıpkı eski konum-yalnız akış.
                        # (Konum z zaten min-irtifa tabanına 'clamped_z' ile kırpıldı.)
                        self.goto_shared_ned(
                            leader_id,
                            data["x"], data["y"], clamped_z,
                            vx=float(data.get("vx", 0.0)),
                            vy=float(data.get("vy", 0.0)),
                            vz=float(data.get("vz", 0.0)),
                            ax=float(data.get("ax", 0.0)),
                            ay=float(data.get("ay", 0.0)),
                            az=float(data.get("az", 0.0)),
                            yaw=float(data.get("yaw", 0.0)),
                            yaw_rate=float(data.get("yaw_rate", 0.0)),
                            type_mask=int(data.get("type_mask", POSITION_ONLY_MASK)),
                        )

                    else:
                        # --- Bayat veri: AUTO'yu garanti et ve döngü mantığını çalıştır ---
                        if guided_mode:
                            logger.warning(
                                f"[LeaderSlot] Veri bayat ({age:.1f}s) "
                                f"— lider {leader_id} AUTO moduna alınıyor."
                            )
                            guided_mode = False
                            enter_auto()
                        else:
                            # Lider AUTO'dan düştüyse (örn. görev bitti, LOITER'a
                            # geçti ya da başka bir şey modu değiştirdi) TEKRAR AUTO
                            # yap. Kullanıcı isteği: hedef yokken lider sonsuza dek
                            # kendi rotasını uçmalı.
                            _m = self.drone_modes.get(leader_id, "")
                            if _m and _m != "AUTO":
                                logger.warning(
                                    f"[LeaderSlot] Lider {leader_id} AUTO'da değil "
                                    f"(mod={_m}) — AUTO'ya geri alınıyor."
                                )
                                enter_auto()

                        if now - last_warn > 5.0:
                            logger.warning(f"[LeaderSlot] Hâlâ bekleniyor... (bayatlık={age:.1f}s)")
                            last_warn = now

                        self._run_mission_loop_step(
                            leader_id, drain_mission_current, mission_state,
                            loop_arrival_dist_m, loop_start_wp_index, wp_speed_mps
                        )

                else:
                    # --- Anahtar Redis'te henüz yok: AUTO'yu garanti et ve döngü mantığını çalıştır ---
                    if guided_mode:
                        logger.warning(
                            f"[LeaderSlot] '{REDIS_KEY}' henüz yok "
                            f"— lider {leader_id} AUTO moduna alınıyor."
                        )
                        guided_mode = False
                        enter_auto()
                    else:
                        _m = self.drone_modes.get(leader_id, "")
                        if _m and _m != "AUTO":
                            logger.warning(
                                f"[LeaderSlot] Lider {leader_id} AUTO'da değil "
                                f"(mod={_m}) — AUTO'ya geri alınıyor."
                            )
                            enter_auto()

                    if now - last_warn > 5.0:
                        logger.warning(f"[LeaderSlot] '{REDIS_KEY}' Redis anahtarı bekleniyor...")
                        last_warn = now

                    self._run_mission_loop_step(
                        leader_id, drain_mission_current, mission_state,
                        loop_arrival_dist_m, loop_start_wp_index, wp_speed_mps
                    )

            except Exception:
                logger.exception("Leader slot controller error.")

            time.sleep(sleep_dt)

    def _set_speed(self, target_id: int, speed_mps: float, allow_over_max: bool = False):
        """
        MAV_CMD_DO_CHANGE_SPEED göndererek drone'un yatay uçuş hızını (hız
        TAVANINI) ayarlar. Cevap beklenmez — fire-and-forget.

        Global MAX_SPEED tavanı: pozitif bir hız isteniyorsa ve allow_over_max
        False ise değer MAX_SPEED ile sınırlandırılır. Yalnızca saldırı/kill
        modu allow_over_max=True geçerek bu tavanı aşabilir.
        """
        if speed_mps > 0 and not allow_over_max and speed_mps > MAX_SPEED:
            logger.debug(f"[SpeedLimit] Drone {target_id}: istenen {speed_mps:.1f} m/s "
                         f"-> {MAX_SPEED:.1f} m/s tavanına kırpıldı.")
            speed_mps = MAX_SPEED
        self.master.mav.command_long_send(
            target_id, 0,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,                    # param1: hız tipi (1 = zemin hızı)
            float(speed_mps),     # param2: istenen hız [m/s]
            -1,                   # param3: gaz kelebeği (-1 = değiştirme)
            0, 0, 0, 0,
        )

    def set_param(self, target_id: int, name: str, value: float):
        """
        Bir drone'un parametresini ayarlar (PARAM_SET). Cevap beklenmez.

        Bu, saldırgan için KRİTİK: ArduPilot'ta GUIDED konum hedefine giderken
        ulaşılabilen tepe hız v_tepe = sqrt(WPNAV_ACCEL * mesafe) ile sınırlıdır.
        DO_CHANGE_SPEED sadece tavanı yükseltir; asıl darboğaz WPNAV_ACCEL'dir.
        """
        try:
            self.master.mav.param_set_send(
                target_id, 1,
                name.encode("utf-8") if isinstance(name, str) else name,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            time.sleep(0.02)
        except Exception:
            logger.exception(f"set_param hatası (tgt={target_id}, {name}={value})")

    def get_param(self, target_id: int, name: str, timeout: float = 3.0):
        """
        Bir drone'un parametresini okur (PARAM_REQUEST_READ -> PARAM_VALUE).
        Değeri float olarak döndürür; timeout durumunda None döner.

        Dispatcher tüm paket okumalarının sahibi olduğundan, cevap PARAM_VALUE
        kuyruğundan alınır (doğrudan recv_match KULLANILMAZ).
        """
        q = self._param_value_queues.get(target_id)
        # Eski PARAM_VALUE'ları temizle ki önceki bir okumanın cevabını almayalım.
        if q is not None:
            while True:
                try: q.get_nowait()
                except queue.Empty: break

        pname = name.encode("utf-8") if isinstance(name, str) else name
        deadline  = time.time() + timeout
        last_send = 0.0
        while time.time() < deadline:
            now = time.time()
            # Periyodik tekrar iste (paket kaybına karşı).
            if now - last_send >= 0.5:
                self.master.mav.param_request_read_send(target_id, 0, pname, -1)
                last_send = now
            if q is None:
                time.sleep(0.1); continue
            try:
                msg = q.get(timeout=0.2)
            except queue.Empty:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode("utf-8", "ignore")
            got = got.rstrip("\x00")
            if got == name:
                return float(msg.param_value)
            # Başka bir parametreye ait cevap — yoksay, yeniden istenecek.

        logger.warning(f"[get_param] Drone {target_id}: '{name}' okunamadı (timeout={timeout:.0f}s).")
        return None

    def _effective_cruise_mps(self) -> float:
        """Etkin seyir hızı [m/s] = min(NAV_SPEED, MAX_SPEED). SABİT tabanlı."""
        return min(NAV_SPEED, MAX_SPEED)

    def _default_speed_mps(self, drone_id: int) -> float:
        """
        Formasyon/seyir için hedef WPNav hızı [m/s] = min(NAV_SPEED, MAX_SPEED).

        SABİTLERDEN türetilir (araçtan OKUNMAZ); böylece önceki bir düşük-MAX
        koşusundan araca yazılmış bozuk bir WPNAV_SPEED değerinden etkilenmez.
        """
        return self._effective_cruise_mps()

    def apply_global_speed_limit(self, drone_ids, param_timeout: float = 3.0):
        """
        seyir hızını ayarlar ve global MAX_SPEED tavanını uygular.
        Saldırı/kill modu bu limitten muaftır (ayrı yüksek profil).
        """
        cruise_mps = self._effective_cruise_mps()
        cruise_cms = cruise_mps * 100.0
        logger.info(
            f"[SpeedLimit] Seyir hızı uygulanıyor: WPNAV_SPEED={cruise_mps:.1f} m/s "
            f"(NAV_SPEED={NAV_SPEED:.1f}, MAX_SPEED={MAX_SPEED:.1f}), "
            f"WPNAV_ACCEL={NAV_WPNAV_ACCEL_CMS/100.0:.1f} m/s^2"
        )
        for did in drone_ids:
            self.set_param(did, "WPNAV_SPEED", cruise_cms)
            self.set_param(did, "WPNAV_ACCEL", NAV_WPNAV_ACCEL_CMS)
            log_event("speed_limit_applied", drone_id=did,
                      wpnav_speed_cms=cruise_cms,
                      wpnav_accel_cms=NAV_WPNAV_ACCEL_CMS,
                      nav_speed_mps=NAV_SPEED, max_speed_mps=MAX_SPEED)

    def _apply_attack_dynamics(self, drone_id: int):
        """Saldırgan drone'u yüksek hız/ivme profiline geçirir (MAX_SPEED muaf)."""
        self.set_param(drone_id, "WPNAV_SPEED", ATTACK_WPNAV_SPEED_CMS)
        self.set_param(drone_id, "WPNAV_ACCEL", ATTACK_WPNAV_ACCEL_CMS)
        # Saldırı/kill modu global tavandan MUAF — allow_over_max=True.
        self._set_speed(drone_id, ATTACK_SPEED_MPS, allow_over_max=True)

    def _restore_default_dynamics(self, drone_id: int):
        """
        Problem 3 — saldırgan formasyona dönerken normal seyir hız/ivme profiline
        geri döner; böylece sürünün geri kalanıyla AYNI hızda uçar.

        Profil apply_global_speed_limit ile AYNI sabit-tabanlı değerlerdir:
        WPNAV_SPEED = min(NAV_SPEED, MAX_SPEED), WPNAV_ACCEL = NAV_WPNAV_ACCEL_CMS.
        """
        cruise_mps = self._effective_cruise_mps()
        self.set_param(drone_id, "WPNAV_SPEED", cruise_mps * 100.0)
        self.set_param(drone_id, "WPNAV_ACCEL", NAV_WPNAV_ACCEL_CMS)
        # Çalışma-anı DO_CHANGE_SPEED tavanını da seyir hızına al.
        self._set_speed(drone_id, cruise_mps)

    def _run_mission_loop_step(
        self, leader_id, drain_current_fn, state, arrival_dist_m, start_wp_index, wp_speed_mps
        ):
        """
        Kanıtlanmış dayanıklılık-turu deseninin bir iterasyonu; lider AUTO'da
        ve taze slot verisi yokken her tikte çağrılır.
        """
        if wp_speed_mps > 0:
            if self.drone_modes.get(leader_id, "") == "RTL":
                self._set_speed(leader_id, RTL_SPEED)
            else:
                self._set_speed(leader_id, wp_speed_mps)

        last_wp_index = state.get("last_wp_index", -1)
        if last_wp_index < 0:
            return

        seq = drain_current_fn()
        if seq is not None:
            state["current_seq"] = seq
            if seq >= last_wp_index:
                state["awaiting_arrival"] = True

        if not state.get("awaiting_arrival"):
            return

        last_wp_ned = state.get("last_wp_ned")
        if last_wp_ned is None:
            logger.warning(
                f"[MissionRestart] Son WP koordinatı bilinmiyor — "
                f"mesafe kontrolü atlanıyor, doğrudan döngü başlatılıyor."
            )
            self._restart_mission(leader_id, seq=start_wp_index)
            state["awaiting_arrival"] = False
            return

        with self.lock:
            leader_pos = self.drone_positions.get(leader_id)

        if leader_pos is None:
            return

        dn = leader_pos[0] - last_wp_ned[0]
        de = leader_pos[1] - last_wp_ned[1]
        dist = math.sqrt(dn * dn + de * de)

        logger.debug(f"[MissionRestart] Son WP'ye kalan mesafe: {dist:.1f}m")

        if dist < arrival_dist_m:
            logger.info(
                f"[MissionRestart] Son WP'ye ulaşıldı (mesafe={dist:.1f}m < {arrival_dist_m:.1f}m) "
                f"— misyon WP {start_wp_index}'den yeniden başlatılıyor."
            )
            self._restart_mission(leader_id, seq=start_wp_index)
            state["awaiting_arrival"] = False
            state["parked_since"] = None
            return

        # --- PARK-EMNİYETİ (yeni) -------------------------------------------
        # Mesafe eşiği çok dar tutulursa (örn. 1 m) araç son WP'nin 1-2 m
        # yakınında durur ve `dist < arrival_dist_m` ASLA sağlanmaz; görev hiç
        # yeniden başlamaz ve lider orada sonsuza dek asılı kalır (log'da tam
        # olarak bu görüldü). Bu emniyet: lider fiilen DURMUŞSA (hız ~0) ve bu
        # birkaç saniye sürdüyse, mesafeye bakmaksızın görevi yeniden başlat.
        with self.lock:
            vel = self.drone_velocities.get(leader_id, (0.0, 0.0, 0.0))
        speed = math.sqrt(vel[0]**2 + vel[1]**2)
        now_m = time.monotonic()
        if speed < 1.0:
            if state.get("parked_since") is None:
                state["parked_since"] = now_m
            elif now_m - state["parked_since"] > 5.0:
                logger.warning(
                    f"[MissionRestart] Lider son WP yakınında DURMUŞ "
                    f"(mesafe={dist:.1f}m, hız={speed:.1f}m/s, 5s+) — "
                    f"görev WP {start_wp_index}'den yeniden başlatılıyor."
                )
                self._restart_mission(leader_id, seq=start_wp_index)
                state["awaiting_arrival"] = False
                state["parked_since"] = None
        else:
            state["parked_since"] = None

    # ------------------------------------------------------------------
    # simple_guided_follow.sh'i yeni bir gnome-terminal penceresinde başlat  
    # ------------------------------------------------------------------
    def launch_guided_follow(
        self,
        script_path: str = "simple_guided_follow.sh",
        redis_host: str  = "localhost",
        redis_port: int  = 6379,
        min_alt_m:  float = 10.0,
        loop_start_wp_index: int   = 2,
        loop_arrival_dist_m: float = 12.0,
        wp_speed_mps: float = 10.0,
        ):
        # abs_path = os.path.abspath(script_path)
        # logger.info(f"Launching simple_guided_follow in new terminal: {abs_path}")
        # try:
        #     self._follow_proc = subprocess.Popen(
        #         [
        #             "gnome-terminal",
        #             "--title=simple_guided_follow",
        #             "--",
        #             "bash", "-c",
        #             f"bash {abs_path}; echo '[simple_guided_follow] Bitti. Kapatmak için Enter.'; read",
        #         ]
        #     )
        #     logger.info(f"gnome-terminal launched (pid={self._follow_proc.pid})")
        # except Exception:
        #     logger.exception("simple_guided_follow gnome-terminal launch error.")

        try:
            import redis as redis_lib
            rc = redis_lib.Redis(host=redis_host, port=redis_port, db=0)
            rc.ping()
            t = threading.Thread(
                target=self._leader_slot_reader_thread,
                args=(rc, self.leader_id),
                kwargs={
                    "min_alt_m": min_alt_m,
                    "loop_start_wp_index": loop_start_wp_index,
                    "loop_arrival_dist_m": loop_arrival_dist_m,
                    "wp_speed_mps": wp_speed_mps,
                },
                name="LeaderSlotCtrl",
                daemon=True,
            )
            t.start()
            self.threads.append(t)
            logger.info(f"Leader slot controller thread başlatıldı ({redis_host}:{redis_port})")
        except Exception:
            logger.exception("Redis bağlantısı kurulamadı — leader slot controller başlatılamadı.")

    # ------------------------------------------------------------------
    # Lider drone'dan NED orijinini al ve Redis'e yayınla  
    # ------------------------------------------------------------------
    def fetch_and_publish_ned_origin(self, leader_id: int, redis_client, timeout: float = 10.0):
        """
        Dispatcher'ın mevcut _gpos_queues'u aracılığıyla lider drone'un
        GLOBAL_POSITION_INT'inden NED orijinini (yer seviyesi GPS) okur.
        Kalkıştan ÖNCE çağrılmalıdır.
        """
        import json
        logger.info(f"NED origin bekleniyor (leader={leader_id}, gpos queue)...")

        q        = self._gpos_queues[leader_id]
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                msg = q.get(timeout=min(1.0, deadline - time.time()))
                rel_alt_m = msg.relative_alt / 1000.0
                if rel_alt_m > 2.0:
                    logger.warning(
                        f"[NedOrigin] Drone {leader_id} zaten havada "
                        f"(relative_alt={rel_alt_m:.1f}m) — mevcut GPS ile devam ediliyor."
                    )
                home_lat = msg.lat / 1e7
                home_lon = msg.lon / 1e7
                home_alt = msg.alt / 1000.0

                payload = {"lat": home_lat, "lon": home_lon, "alt": home_alt}
                redis_client.set("ned_origin", json.dumps(payload))
                self._ned_origin = (home_lat, home_lon, home_alt)
                logger.info(
                    f"NED origin Redis'e yazıldı: lat={home_lat:.7f} "
                    f"lon={home_lon:.7f} alt={home_alt:.1f}m "
                    f"(relative_alt={rel_alt_m:.1f}m)"
                )
                return home_lat, home_lon, home_alt

            except queue.Empty:
                pass

        raise RuntimeError(
            f"NED origin alınamadı — drone {leader_id}'den {timeout}s içinde "
            "GLOBAL_POSITION_INT gelmedi. Dispatcher çalışıyor mu?"
        )


    def wait_for_gps_health(self, drone_id, min_fix=3, min_sats=10, timeout=60.0):
        """
        Drone sağlıklı bir GPS fix bildirene kadar bekler; zaman aşımında hata verir.
        GPS_RAW_INT gerektirir; akmıyorsa talep eder.
        min_fix=3 -> 3B kilit. min_sats -> görünür uydu eşiği.
        """
        logger.info(f"[GPSGate] Drone {drone_id}: GPS sağlığı bekleniyor "
                    f"(fix>={min_fix}, sats>={min_sats})...")
        self.request_message_interval(drone_id, 'GPS_RAW_INT', 2.0)
        deadline = time.time() + timeout
        last_log = 0.0
        while time.time() < deadline:
            # GPS_RAW_INT ayrı bir kuyrukta tutulmuyor; kısa bir recv ile
            # doğrudan master'dan fırsatçı olarak okunur.
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1.0)
            if msg and msg.get_srcSystem() == drone_id:
                fix  = int(msg.fix_type)
                sats = int(msg.satellites_visible)
                if time.time() - last_log > 2.0:
                    logger.info(f"[GPSGate] Drone {drone_id}: fix={fix} sats={sats}")
                    last_log = time.time()
                if fix >= min_fix and sats >= min_sats:
                    logger.info(f"[GPSGate] Drone {drone_id}: GPS OK (fix={fix}, sats={sats}).")
                    log_event("gps_ok", drone_id=drone_id, fix=fix, sats=sats)
                    return True
        log_event("gps_gate_timeout", drone_id=drone_id)
        raise RuntimeError(
            f"[GPSGate] Drone {drone_id}: {timeout:.0f}s içinde sağlıklı GPS "
            f"(fix>={min_fix}, sats>={min_sats}) alınamadı — kalkış İPTAL."
        )

    def gps_gate_all(self, drone_ids, min_fix=3, min_sats=10, timeout=60.0):
        """Her drone'un GPS sağlığını kontrol eder. Biri geçemezse görev iptal edilir."""
        for did in drone_ids:
            self.wait_for_gps_health(did, min_fix=min_fix, min_sats=min_sats, timeout=timeout)
        logger.info("[GPSGate] Tüm drone'lar sağlıklı GPS fix ile hazır.")


    _MSG_IDS = {
        'GPS_RAW_INT': 24, 'RAW_IMU': 27, 'SERVO_OUTPUT_RAW': 36,
        'SYS_STATUS': 1, 'VIBRATION': 241, 'GLOBAL_POSITION_INT': 33,
        'ATTITUDE': 30, 'VFR_HUD': 74,
    }

    def request_message_interval(self, drone_id, msg_name, rate_hz):
        """Drone'dan msg_name mesajını rate_hz hızında akıtmasını ister (0 = kapat)."""
        msg_id = self._MSG_IDS.get(msg_name)
        if msg_id is None:
            logger.warning(f"request_message_interval: bilinmeyen mesaj '{msg_name}'")
            return
        interval_us = -1 if rate_hz <= 0 else int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            drone_id, 0,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            float(msg_id), float(interval_us), 0, 0, 0, 0, 0,
        )
        time.sleep(0.02)

    def setup_high_rate_telemetry(self, drone_ids):
        """Tüm drone'larda darbe/sağlık mesaj akışlarını talep eder."""
        logger.info("Yüksek hızlı telemetri akışları isteniyor...")
        for did in drone_ids:
            for msg_name, hz in MSG_RATE_HZ.items():
                self.request_message_interval(did, msg_name, hz)
            # Şartnamedeki "Vuruş Tespiti Uçuş Kaydı" için GLOBAL_POSITION_INT /
            # ATTITUDE akışlarının >= 20 Hz olmasını da garanti et.
            self.request_message_interval(did, 'GLOBAL_POSITION_INT', 25.0)
            self.request_message_interval(did, 'ATTITUDE', 25.0)
            self.request_message_interval(did, 'VFR_HUD', 20.0)
        logger.info("Telemetri akış istekleri gönderildi (bench'te gerçek hızı doğrula).")


    def start_flight_csv_loggers(self, drone_ids, log_dir="logs/flight_csv", rate_hz=25.0):
        import csv as _csv
        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")

        def logger_thread(did):
            path = os.path.join(log_dir, f"drone{did}_{stamp}.csv")
            f = open(path, "w", newline="")
            w = _csv.writer(f)
            w.writerow(["wall_time", "lat", "lon", "alt_amsl", "rel_alt",
                        "vn", "ve", "vd", "accel_g", "roll", "pitch", "yaw",
                        "mode", "batt_v", "batt_a", "armed"])
            dt = 1.0 / max(rate_hz, 1.0)
            rows = 0
            logger.info(f"[FlightCSV] Drone {did} -> {path} @ {rate_hz:.0f}Hz")
            while not self.stop_threads.is_set():
                try:
                    with self.lock:
                        pos  = self.drone_positions.get(did)
                        vel  = self.drone_velocities.get(did, (0, 0, 0))
                        yaw  = self.drone_headings.get(did, 0.0)
                        mode = self.drone_modes.get(did, "")
                        armed = self.drone_armed.get(did, False)
                    # Son attitude/global verisini kuyruklardan fırsatçı olarak al.
                    lat = lon = alt = rel = None
                    try:
                        gq = self._gpos_queues[did]
                        gmsg = gq.queue[-1] if len(gq.queue) else None
                        if gmsg is not None:
                            lat = gmsg.lat / 1e7
                            lon = gmsg.lon / 1e7
                            alt = gmsg.alt / 1000.0
                            rel = gmsg.relative_alt / 1000.0
                    except Exception:
                        pass
                    sysh = self._sys_health.get(did) or {}
                    w.writerow([
                        f"{time.time():.3f}",
                        lat, lon, alt, rel,
                        vel[0], vel[1], vel[2],
                        f"{self._last_accel_g.get(did, 0.0):.3f}",
                        "", "", f"{yaw:.4f}",
                        mode, sysh.get("batt_v"), sysh.get("batt_a"),
                        int(bool(armed)),
                    ])
                    rows += 1
                    if rows % 50 == 0:
                        f.flush()
                except Exception:
                    logger.exception(f"[FlightCSV] Drone {did} yazma hatası.")
                time.sleep(dt)
            f.flush(); f.close()

        for did in drone_ids:
            t = threading.Thread(target=logger_thread, args=(did,),
                                 name=f"FlightCSV-{did}", daemon=True)
            t.start()
            self.threads.append(t)

    # ==================================================================
    # YENİ: lider seçim işleyicisi (lider kaybında heartbeat bekçisi çağırır)
    # ==================================================================
    def enable_leader_failover(self):
        """Lider heartbeat kaybının seçimi tetiklemesi için geri çağırmayı kaydeder."""
        self._on_leader_lost = self._elect_new_leader
        logger.info("Lider failover etkin (heartbeat kaybında seçim yapılır).")

    def _elect_new_leader(self, dead_leader, alive_ids):
        """
        SUCCESSION_ORDER sırasında hayatta olan ve saldırı ortasında olmayan bir
        sonraki drone'u lider yapar. eski lider (linki dönerse) takipçi olur.
        """
        with self._election_lock:
            if self._election_in_progress:
                return
            if self.leader_id != dead_leader:
                return   # zaten başka biri seçilmiş
            new_ldr = next_leader(SUCCESSION_ORDER, alive_ids, dead_leader,
                                  skip_ids=self._attack_skip)
            if new_ldr is None:
                logger.error("[Election] Uygun yeni lider yok! Sürü LOITER'a alınıyor.")
                self.follow_enable.clear()
                for did in alive_ids:
                    self.set_mode(did, "LOITER")
                return
            self._election_in_progress = True

        logger.warning(f"[Election] Lider {dead_leader} kayboldu -> yeni lider {new_ldr}")
        log_event("leader_elected", old_leader=dead_leader, new_leader=new_ldr,
                  alive=sorted(alive_ids))

        # Yeni lider tarama görevini uçmalı (TÜM drone'lara önceden yüklenmiş).
        # AUTO'ya al; formasyon yöneticisi her tikte self.leader_id okuduğu için
        # kendiliğinden yeni liderin etrafına oturur.
        self.leader_id = new_ldr
        self.set_mode(new_ldr, "AUTO")

        # Ölü lidere elden geldiğince RTL gönder (yalnızca linki dönerse ulaşır).
        self.set_mode(dead_leader, "RTL")

        # Yeni geometrinin temiz hesaplanması için slot atamalarını sıfırla.
        self._slot_assignments = {}
        self.follow_enable.set()
        self._last_leader_pos_time = time.monotonic()

        with self._election_lock:
            self._election_in_progress = False

    # ==================================================================
    # YENİ: darbe + hasar öz-kontrol izleyicisi (gerçek telemetri).
    # Süreli ivme sıçramasından darbeyi tespit eder, titreşimle doğrular,
    # ardından otonom hasar öz-kontrolü çalıştırır. Geçerse göreve devam eder;
    # kalırsa OLDUĞU YERE İNER (hasarlı aracı RTL ile sahada sürükleme).
    # ==================================================================
    def start_impact_monitor(self, drone_ids):
        def monitor(did):
            above_since = None
            logger.info(f"[Impact] Drone {did} izleniyor.")
            while not self.stop_threads.is_set():
                g = self._last_accel_g.get(did, 1.0)
                now = time.monotonic()
                if g >= IMPACT_ACCEL_G:
                    if above_since is None:
                        above_since = now
                    elif (now - above_since) * 1000.0 >= IMPACT_MIN_MS:
                        if not self._impact_flag[did]:
                            logger.warning(f"[Impact] Drone {did}: DARBE tespit edildi "
                                           f"(|a|={g:.1f}g).")
                            log_event("impact_detected", drone_id=did, accel_g=round(g, 2))
                            self._impact_flag[did] = True
                            self._on_impact(did)
                else:
                    above_since = None
                time.sleep(0.02)   # 50 Hz check
        for did in drone_ids:
            t = threading.Thread(target=monitor, args=(did,),
                                 name=f"Impact-{did}", daemon=True)
            t.start()
            self.threads.append(t)

    def _on_impact(self, did):
        """Otonom hasar öz-kontrolünü ayrı bir thread'de çalıştırır."""
        t = threading.Thread(target=self._damage_self_check, args=(did,),
                             name=f"Damage-{did}", daemon=True)
        t.start()
        self.threads.append(t)

    def _damage_self_check(self, did, hold_s=3.0):
        """
        Tamamen otonom, insan müdahalesi yok. Kısa süre bekleyip gerçek telemetri
        ile aracın hâlâ kontrol edilebilir olup olmadığını değerlendirir:
          * SYS_STATUS sensör sağlık bit maskesi (jiroskop/ivmeölçer/motor)
          * batarya akımı makullüğü
          * SERVO_OUTPUT_RAW doyumu (hasarlı pervane motoru tepede sabitler)
        Geçerse görevine döner. Kalırsa olduğu yere iner.
        """
        logger.info(f"[Damage] Drone {did}: hasar öz-kontrolü ({hold_s:.0f}s)...")
        t0 = time.monotonic()
        sat_hits = 0
        samples = 0
        while time.monotonic() - t0 < hold_s and not self.stop_threads.is_set():
            # Motor doyum kontrolü
            try:
                sq = self._servo_queues[did]
                smsg = sq.queue[-1] if len(sq.queue) else None
                if smsg is not None:
                    outs = [getattr(smsg, f"servo{i}_raw", 0) for i in range(1, 5)]
                    if any(o >= 1980 for o in outs if o):   # pinned near max
                        sat_hits += 1
                    samples += 1
            except Exception:
                pass
            time.sleep(0.1)

        sysh = self._sys_health.get(did) or {}
        health = sysh.get("health")
        present = sysh.get("present")
        health_bad = False
        if health is not None and present:
            # Mevcut+etkin bir sensör sağlıksız bildiriyorsa bu kırmızı bayraktır.
            unhealthy = present & ~health
            if unhealthy:
                health_bad = True

        motor_bad = samples > 0 and (sat_hits / samples) > 0.4

        if health_bad or motor_bad:
            logger.error(f"[Damage] Drone {did}: HASARLI kabul edildi "
                         f"(health_bad={health_bad}, motor_bad={motor_bad}) -> LAND.")
            log_event("damage_confirmed", drone_id=did,
                      health_bad=health_bad, motor_bad=motor_bad)
            self.set_mode(did, "LAND")
            self._alive_ids.discard(did)
            # Hasarlı drone saldırgansa görevi bırakır ve sıradaki saldırgana devreder.
            if self._committed_attacker == did:
                self._release_attacker(did, reason="damaged")
                if ENABLE_KILL_STRATEGY:
                    self._commit_next_attacker()
        else:
            logger.info(f"[Damage] Drone {did}: SAĞLAM — göreve devam.")
            log_event("damage_cleared", drone_id=did)
            self._impact_flag[did] = False
            # Darbeden sağ çıktı: saldırgan idiyse formasyona döner; hedef hâlâ
            # hayattaysa saldırı mantığı başka bir saldırgan atayabilir.
            if self._committed_attacker == did:
                self._release_attacker(did, reason="survived_hit")

    def publish_pursuer_state(self, redis_client, drone_id, key):
        """Bir drone'un canlı NED durumunu, güdüm sürecinin okuması için yayınlar."""
        with self.lock:
            pos = self.drone_positions.get(drone_id)
            vel = self.drone_velocities.get(drone_id, (0, 0, 0))
        if pos is None:
            return
        payload = {"x": pos[0], "y": pos[1], "z": pos[2],
                   "vx": vel[0], "vy": vel[1], "vz": vel[2],
                   "ts": time.monotonic()}
        try:
            redis_client.set(key, json.dumps(payload))
        except Exception:
            pass

    def start_pursuer_state_publisher(self, redis_client, rate_hz=20.0):
        """
        Liderin canlı NED durumunu yayınlar; böylece liderin kendi güdüm süreci
        (simple_guided_follow --leader-state-key leader_state_ned) CPA kesişme
        geometrisini hesaplayabilir.
        """
        def pub():
            # LİDERİN canlı NED durumunu yayınlar. Saldırganın artık burada
            # yayınlanan bir duruma ihtiyacı yok — süreç-içi kesişme hesabı
            # doğrudan self.drone_positions'dan okuyor.
            dt = 1.0 / max(rate_hz, 1.0)
            while not self.stop_threads.is_set():
                self.publish_pursuer_state(redis_client, self.leader_id, LEADER_STATE_KEY)
                time.sleep(dt)
        t = threading.Thread(target=pub, name="PursuerStatePub", daemon=True)
        t.start()
        self.threads.append(t)

    def _pick_attacker(self, target_ned=None):
        """
        Uygun bir saldırgan seçer.

        F (çarpışma önleme): target_ned verildiyse ve ENABLE_TARGET_SIDE_SELECTION
        açıksa, uygun adaylar arasından çıkış vektörü hedef yönüyle EN İYİ
        hizalanan (yani zaten hedef TARAFINDA olan) drone'u seçer — böylece
        sürünün içinden geçmeden dışa doğru ayrılır. target_ned yoksa veya özellik
        kapalıysa ATTACKER_PRIORITY sırasını kullanır (eski davranış birebir korunur).
        """
        now = time.monotonic()

        def _eligible(did):
            # LİDER DE SALDIRABİLİR (2026-07-31). Eskiden burada
            #     if did == self.leader_id:
            #         return False
            # vardı; ATTACKER_PRIORITY = [LEADER_ID] senaryosunda bu, aday
            # listesini HER ZAMAN boşaltıyor ve _pick_attacker None döndürüyordu
            # -> hiçbir saldırı hiç başlamıyordu. Lider saldırırken slot
            # kontrolü _leader_slot_reader_thread içinde bırakılır (orada
            # _committed_attacker kontrolüne bak), böylece iki thread aynı
            # araca aynı anda setpoint yollamaz.
            if did in self._grounded_ids:
                return False
            if did not in self._alive_ids:
                return False
            if did == self._committed_attacker:
                return False
            if self._impact_flag.get(did):
                return False
            # Başarısız denemeden sonra bekleme süresindeki drone'ları atla.
            if self._attack_cooldown.get(did, 0.0) > now:
                return False
            return True

        candidates = [d for d in ATTACKER_PRIORITY if _eligible(d)]
        if not candidates:
            return None

        # Öncelik sırası (eski davranış) — hedef bilinmiyorsa ya da F kapalıysa.
        if target_ned is None or not ENABLE_TARGET_SIDE_SELECTION:
            return candidates[0]

        # Hedef-tarafı seçimi: sürü merkezinden hedefe birim vektör ile
        # merkezden aday drone'a birim vektörün iç çarpımı en büyük olan aday.
        with self.lock:
            positions = {d: p for d, p in self.drone_positions.items() if p is not None}
        others = [p for d, p in positions.items() if d not in self._grounded_ids]
        if not others:
            return candidates[0]
        cn = sum(p[0] for p in others) / len(others)
        ce = sum(p[1] for p in others) / len(others)
        tdx, tdy = target_ned[0] - cn, target_ned[1] - ce
        tnorm = math.hypot(tdx, tdy)
        if tnorm < 1e-3:
            return candidates[0]
        tux, tuy = tdx / tnorm, tdy / tnorm

        best, best_score = None, -2.0
        for did in candidates:
            p = positions.get(did)
            if p is None:
                continue
            dx, dy = p[0] - cn, p[1] - ce
            dnorm = math.hypot(dx, dy)
            score = (dx * tux + dy * tuy) / dnorm if dnorm > 1e-3 else 0.0
            if score > best_score:
                best, best_score = did, score
        return best if best is not None else candidates[0]

    def commit_attacker(self, drone_id, script_path=None, redis_host="localhost",
                        redis_port=6379, min_alt_m=10.0):
        """
        Bir takipçiyi saldırıya atar — İKİNCİ bir güdüm penceresi AÇMADAN.

        Önceki tasarıma göre düzeltme: artık 2. bir simple_guided_follow süreci
        başlatılmıyor. Hedef izi zaten Redis'te ('dogru_rakip_telemetri', tek
        çalışan controller/güdüm zinciri tarafından güncelleniyor). Bu metot,
        hafif bir süreç-içi kesişme thread'i başlatır:
          * AYNI hedef izini Redis'ten okur,
          * BU saldırganın canlı NED durumunu okur (self.drone_positions),
          * çarpışma üçgeni ile gerçek kesişme noktasını hesaplar,
          * saldırganı doğrudan goto_shared_ned ile yönlendirir.
        İkinci terminal yok, ikinci IMM yok, attacker_slot_ned gidiş-dönüşü yok.
        """
        if self._committed_attacker is not None:
            logger.warning(f"[Attack] Zaten saldıran var ({self._committed_attacker}).")
            return
        logger.warning(f"[Attack] Drone {drone_id} SALDIRIYA atandı (in-process intercept, "
                       f"max hız {ATTACK_SPEED_MPS:.0f} m/s).")
        log_event("attacker_committed", drone_id=drone_id)
        self._committed_attacker = drone_id
        self._attack_skip.add(drone_id)   # lider seçimi bir saldırganı terfi ettirmemeli
        self.set_mode(drone_id, "GUIDED")
        # Tam saldırı hız/ivme profilini HEMEN uygula. Sürüden ayrılma artık
        # TAM HIZDA yapılıyor (aim noktası dışa/yukarı saptırılarak sürünün
        # etrafından dönülür — bkz. _egress_deflect); bu yüzden hızı geciktirmeye
        # gerek yok. (SADECE DO_CHANGE_SPEED yeterli DEĞİL: S-eğrisi hedefte
        # frenlediği için tepe hız sqrt(WPNAV_ACCEL * mesafe) ile sınırlı; bu
        # yüzden _apply_attack_dynamics WPNAV_ACCEL/WPNAV_SPEED'i de yükseltir.)
        self._apply_attack_dynamics(drone_id)

        # Geçiş sonrası yaw kilidi WP_YAW_BEHAVIOR=0 yazacak; ÖNCEKİ değerini
        # şimdi, yaklaşma safhasında öğren (kilit tam geçiş anında kurulduğu için
        # orada 3 sn'lik bir get_param beklemesi göze alınamaz). get_param BLOKE
        # ettiğinden ayrı bir thread'e alınır: saldırının başlaması gecikmesin.
        if ENABLE_ATTACK_YAW_LOCK:
            self._cache_wp_yaw_behavior(drone_id)

        rc = redis.Redis(host=redis_host, port=redis_port, db=0)
        t = threading.Thread(target=self._attacker_intercept_thread,
                             args=(rc, drone_id, min_alt_m),
                             name=f"AttackerIntercept-{drone_id}", daemon=True)
        t.start()
        self.threads.append(t)

    def _attacker_intercept_thread(self, redis_client, drone_id, min_alt_m,
                                   send_rate_hz=10.0, target_key="dogru_rakip_telemetri"):
        """
        Göreve atanmış tek bir saldırgan için süreç-içi kesişme güdümü.
        """
        dt = 1.0 / max(send_rate_hz, 1.0)
        min_z = -abs(min_alt_m)
        origin = self._ned_origin
        if origin is None:
            logger.error("[Attack] NED origin yok - intercept baslatilamiyor.")
            self._release_attacker(drone_id, reason="no_origin")
            return

        prev_tp       = None
        prev_meas_t   = None  
        prev_ctrl_ts  = None 
        tgt_vel       = (0.0, 0.0, 0.0)
        best_rng      = float("inf")   # bu saldırıda ulaşılan en yakın menzil
        miss_armed    = False          # yeterince yaklaşınca True olur
        pass_armed    = False          # terminal menzile girildi -> geçiş beklenir
        t_commit      = time.monotonic()
        last_log      = 0.0

        # Saldırgan irtifa tavanı (aim clamp + egress sapması için).
        max_alt_m = getattr(self, "_attacker_max_alt", config.MAX_ALT_M)

        # Canlı irtifa ofsetinin rampasını bu saldırının başına sabitle: tekil
        # (singleton) okuyucu saldırılar ARASINDA yoklanmıyor, o yüzden eski
        # _applied/_last_t değerleriyle başlarsak yeni saldırının ilk saniyeleri
        # bayat bir irtifaya nişan alırdı.
        ALT_OFFSET.reset()
        alt_offset_m = ALT_OFFSET.value()
        alt_clipped_since_log = False
        logger.info(f"[Attack] d{drone_id} irtifa ofseti {alt_offset_m:+.2f} m "
                    f"(canlı: guidance_config.ALT_OFFSET_M)")

        while not self.stop_threads.is_set() and self._committed_attacker == drone_id:
            try:
                if self._shutting_down.is_set():
                    logger.info(f"[Attack] Kapanış algılandı — d{drone_id} saldırısı durduruldu.")
                    self._release_attacker(drone_id, reason="shutdown")
                    return

                # ---- kesin zaman aşımı --------------------------------------
                elapsed = time.monotonic() - t_commit
                if elapsed > ATTACK_TIMEOUT_S:
                    logger.warning(f"[Attack] d{drone_id} {ATTACK_TIMEOUT_S:.0f}s icinde "
                                   f"vuramadi -> takipci moduna donuyor.")
                    self._release_attacker(drone_id, reason="timeout")
                    return

                raw = redis_client.get(target_key)
                if not raw:
                    time.sleep(dt); continue
                tj = json.loads(raw)
                if isinstance(tj, dict) and tj.get("valid") is False:
                    time.sleep(dt); continue
                if "lat" not in tj or "lon" not in tj:
                    time.sleep(dt); continue

                # ---- hedefi ortak NED'e çevir --------------------------------
                t_alt_rel = float(tj.get("alt", 0.0))
                tn, te, td = latlon_to_ned(
                    float(tj["lat"]), float(tj["lon"]),
                    origin[2] + t_alt_rel, *origin)
                tp = (tn, te, td)
                now = time.monotonic()

                ctrl_ts = tj.get("ctrl_ts")
                is_new = False
                meas_dt = None
                if ctrl_ts is not None:
                    ctrl_ts = float(ctrl_ts)
                    if prev_ctrl_ts is None or ctrl_ts > prev_ctrl_ts:
                        is_new = True
                        if prev_ctrl_ts is not None:
                            meas_dt = ctrl_ts - prev_ctrl_ts
                else:
                    if prev_tp is None or (abs(tp[0]-prev_tp[0]) +
                                           abs(tp[1]-prev_tp[1]) +
                                           abs(tp[2]-prev_tp[2])) > 1e-6:
                        is_new = True
                        if prev_meas_t is not None:
                            meas_dt = now - prev_meas_t

                if is_new:
                    if prev_tp is not None and meas_dt and meas_dt > 1e-3:
                        raw_v = ((tp[0]-prev_tp[0])/meas_dt,
                                 (tp[1]-prev_tp[1])/meas_dt,
                                 (tp[2]-prev_tp[2])/meas_dt)
                        spd = math.sqrt(raw_v[0]**2 + raw_v[1]**2 + raw_v[2]**2)
                        if spd <= ATTACK_MAX_TGT_SPD:      # sıçramaları reddet
                            a_lpf = ATTACK_VEL_LPF
                            tgt_vel = (a_lpf*raw_v[0] + (1-a_lpf)*tgt_vel[0],
                                       a_lpf*raw_v[1] + (1-a_lpf)*tgt_vel[1],
                                       a_lpf*raw_v[2] + (1-a_lpf)*tgt_vel[2])
                    prev_tp      = tp
                    prev_meas_t  = now
                    prev_ctrl_ts = ctrl_ts if ctrl_ts is not None else prev_ctrl_ts

                # ---- saldırganın kendi durumu -------------------------------
                with self.lock:
                    ap = self.drone_positions.get(drone_id)
                    av = self.drone_velocities.get(drone_id, (0.0, 0.0, 0.0))
                if ap is None:
                    time.sleep(dt); continue

                rel = (tp[0]-ap[0], tp[1]-ap[1], tp[2]-ap[2])
                rng = math.sqrt(rel[0]**2 + rel[1]**2 + rel[2]**2)

                # Yaw kilidi izleyicisi ile _pick_attacker için taze hedef NED'i.
                self._last_target_ned = tp

                # ---- GEÇİŞ TESPİTİ + yaw kilidi ------------------------------
                # Menzil değişim hızı (range-rate) ANALİTİK hesaplanır:
                #   d(rng)/dt = birim_kerteriz · (hedef_hızı - kendi_hızımız)
                # İşaret negatifken kapanıyoruz, pozitife dönmesi = en yakın
                # geçiş anı (drone hedef düzlemini kesti). Menzil FARKI almaya
                # göre gecikmesizdir ve GPS gürültüsüne karşı daha temizdir —
                # kilidi tam o anda kurabilmek için bu önemli.
                if ENABLE_ATTACK_YAW_LOCK and rng > 1e-3:
                    if rng <= ATTACK_TERMINAL_M:
                        pass_armed = True
                    if pass_armed:
                        rr = ((rel[0]*(tgt_vel[0]-av[0]) +
                               rel[1]*(tgt_vel[1]-av[1]) +
                               rel[2]*(tgt_vel[2]-av[2])) / rng)
                        if rr > 0.0:
                            self._lock_yaw(drone_id,
                                           reason=f"target_pass_rng_{rng:.0f}m")
                            # Yalnız kilit GERÇEKTEN kurulduysa kur-bir-kez
                            # bayrağını düşür; baş açısı o an bilinmiyorsa
                            # sonraki tiklerde tekrar denenir.
                            if self._locked_yaw(drone_id) is not None:
                                pass_armed = False

                # ---- iptal kontrolleri --------------------------------------
                if rng > ATTACK_ABORT_MAX_M:
                    logger.warning(f"[Attack] d{drone_id} hedef cok uzaklasti "
                                   f"({rng:.0f}m > {ATTACK_ABORT_MAX_M:.0f}m) -> takipci.")
                    self._release_attacker(drone_id, reason="target_ran_away")
                    return

                if rng < best_rng:
                    best_rng = rng
                if best_rng <= ATTACK_ABORT_ARM_M:
                    miss_armed = True
                if miss_armed and rng > best_rng + ATTACK_ABORT_OPEN_M:
                    logger.warning(f"[Attack] d{drone_id} ISKA: menzil aciliyor "
                                   f"({rng:.0f}m > en iyi {best_rng:.0f}m + "
                                   f"{ATTACK_ABORT_OPEN_M:.0f}m) -> takipci.")
                    self._release_attacker(drone_id, reason="missed_range_opening")
                    return

                # ---- güdüm ---------------------------------------------------
                atk_spd = ATTACK_SPEED_MPS   # saldırı boyunca maksimum hız komutlanır

                if rng <= ATTACK_TERMINAL_M:
                    # Terminal safha: çok kısa menzilde kesişme çözümü kötü
                    # koşullanır; hedefin hemen önüne kısa sabit öngörüyle nişan al.
                    ip = (tp[0] + tgt_vel[0]*0.3,
                          tp[1] + tgt_vel[1]*0.3,
                          tp[2] + tgt_vel[2]*0.3)
                    phase = "TERM"
                    t_int = rng / max(atk_spd, 1.0)
                else:
                    t_int = solve_intercept_time(rel, tgt_vel, atk_spd)
                    if t_int is None:
                        # Kesişme çözümü yok (hedef bu geometride bizden kaçıyor).
                        # Doğrudan hedefe nişan al ve kapatmaya çalış; umutsuzsa
                        # ıska dedektörü / zaman aşımı serbest bırakacak.
                        ip = tp
                        phase = "PURSUE"
                        t_int = rng / max(atk_spd, 1.0)
                    else:
                        ip = (tp[0] + tgt_vel[0]*t_int,
                              tp[1] + tgt_vel[1]*t_int,
                              tp[2] + tgt_vel[2]*t_int)
                        phase = "INT"

                # --- Nişan noktasını kesişmenin ÖTESİNE uzat ---
                ex = ip[0] - ap[0]; ey = ip[1] - ap[1]; ez = ip[2] - ap[2]
                enorm = math.sqrt(ex*ex + ey*ey + ez*ez)
                if enorm > 1e-3:
                    ux, uy, uz = ex/enorm, ey/enorm, ez/enorm
                    aim = (ip[0] + ux*ATTACK_OVERSHOOT_M,
                           ip[1] + uy*ATTACK_OVERSHOOT_M,
                           ip[2] + uz*ATTACK_OVERSHOOT_M)
                else:
                    aim = ip

                # --- Sürüden TAM HIZDA ayrılma (egress) — çarpışma önleme (#3) ---
                # Sürünün içindeyken UZAK nişan noktasını dışa+yukarı saptır;
                # nişan uzak kaldığı için S-eğrisi frenlemez -> tam hız korunur,
                # yörünge sürünün ETRAFINDAN döner. Temizken sapma sıfırlanır.
                deflected = False
                if ENABLE_ATTACK_EGRESS:
                    aim, deflected = self._egress_deflect(drone_id, ap, aim)

                # İrtifa zarfı — İKİ tarafı da sınırla (NED z = -irtifa):
                #   taban: min irtifanın altına inme (eski davranış),
                #   tavan: max irtifanın (config/controller.json) üstüne çıkma.
                # CANLI irtifa ofseti (guidance_config.ALT_OFFSET_M): araç
                # irtifasını olduğundan ~5 m YÜKSEK okuduğu için komutlanan her
                # irtifada gerçekte o kadar ALÇAK oturuyor -> ofset olmadan
                # saldırgan hedefin 5 m ALTINDAN geçer. Zarf kırpmalarından
                # ÖNCE uygulanır ki taban/tavan sınırları GERÇEK komuta baksın.
                # Uçuş sırasında guidance_config.py düzenlenip kaydedildiğinde
                # ~0.5 s içinde etkir; değişim ALT_OFFSET_SLEW_MPS ile rampalanır.
                alt_offset_m = ALT_OFFSET.value(now)
                ceil_z = -(max_alt_m - EGRESS_CEIL_PAD_M)   # daha negatif olamaz
                aim_z_wanted = aim[2] - alt_offset_m  # NED z aşağı-pozitif: eksi = yukarı
                aim_z = min(aim_z_wanted, min_z)   # taban
                aim_z = max(aim_z, ceil_z)   # tavan
                # SADECE TAVAN kırpması bildirilir: aim_z istenenden BÜYÜKSE
                # (NED'de büyük = alçak) tavan ofseti yutmuş demektir ve
                # saldırgan yine hedefin ALTINDAN geçer — sessiz kalmamalı.
                # Taban kırpması aracı istenenden YUKARI iter; bu güvenli
                # taraftır ve bayrak yakmaz (yoksa hedef min_alt altında
                # uçarken bayrak sürekli yanar ve anlamını yitirirdi).
                if alt_offset_m > 0.0 and (aim_z - aim_z_wanted) > 0.05:
                    alt_clipped_since_log = True

                # --- Komut gönderimi ------------------------------------------
                # Yaw kilitliyse donmuş baş açısı her komutta AÇIKÇA gönderilir
                # (maske yaw alanını kullanır); kilit yoksa yaw yok sayılır ve
                # davranış eskisiyle birebir aynıdır.
                lock_yaw = self._locked_yaw(drone_id)
                if self.is_in_guided_mode(drone_id):
                    if (ENABLE_VELOCITY_TERMINAL and phase == "TERM"
                            and not deflected):
                        # KILL / terminal safha: saf HIZ vektörü (ram-through).
                        # Konum hedefi YOK -> S-eğrisi frenlemesi YOK; drone
                        # hedefin İÇİNDEN tam hızla geçer (WPNAV_ACCEL'e dokunmadan).
                        dxr = aim[0] - ap[0]; dyr = aim[1] - ap[1]; dzr = aim_z - ap[2]
                        vnorm = math.sqrt(dxr*dxr + dyr*dyr + dzr*dzr)
                        if vnorm > 1e-3:
                            vs = ATTACK_SPEED_MPS / vnorm
                            self._send_velocity_ned(drone_id, dxr*vs, dyr*vs, dzr*vs,
                                                    yaw=lock_yaw)
                        else:
                            self._goto_with_yaw_lock(drone_id, aim[0], aim[1], aim_z,
                                                     lock_yaw)
                    else:
                        self._goto_with_yaw_lock(drone_id, aim[0], aim[1], aim_z,
                                                 lock_yaw)

                if now - last_log > 1.0:
                    tspd = math.sqrt(tgt_vel[0]**2 + tgt_vel[1]**2 + tgt_vel[2]**2)
                    aspd_act = math.sqrt(av[0]**2 + av[1]**2 + av[2]**2)
                    tag = ("VEL" if (ENABLE_VELOCITY_TERMINAL and phase == "TERM"
                                     and not deflected)
                           else ("DEF" if deflected else phase))
                    logger.info(f"[Attack] d{drone_id} {phase}/{tag} rng={rng:.0f}m "
                                f"alt_off={alt_offset_m:+.1f}m"
                                f"{'(ZARF KIRPTI)' if alt_clipped_since_log else ''} "
                                f"best={best_rng:.0f}m t_int={t_int:.1f}s "
                                f"tgt_spd={tspd:.1f} atk_spd={aspd_act:.1f}m/s "
                                f"t={elapsed:.0f}s kesisme=({ip[0]:.0f},{ip[1]:.0f})")
                    last_log = now
                    alt_clipped_since_log = False

            except Exception:
                logger.exception("[Attack] intercept thread error.")
            time.sleep(dt)

    def _egress_deflect(self, drone_id, ap, aim):
        """
        Sürüden TAM HIZDA ayrılma (#3 — çarpışma önleme, hız kaybı YOK).

        Girdi:  aim  -> normal kesişme nişan noktası (UZAK; overshoot dahil).
        Döner:  (yeni_aim, deflected_bool)
                deflected=True ise aim, sürünün etrafından dönmek için dışa+yukarı
                saptırılmıştır (yön döndürülür ama nişan UZAK kalır -> S-eğrisi
                frenlemez, tam hız korunur). Temizse aim aynen döner, False.

        Mantık: en yakın komşuya yatay mesafe EGRESS_CLEAR_RADIUS_M'nin altındaysa
        (yani hâlâ sürünün içindeyiz), nişan yönünü sürü merkezinden DIŞA ve
        YUKARI doğru, ne kadar 'içeride' olduğumuzla orantılı bir ağırlıkla harmanla.
        Uzaklaştıkça ağırlık 0'a iner ve saldırgan doğal olarak gerçek kesişmeye
        oturur. İrtifa tavanı çağıran taraftaki aim_z clamp'inde korunur.
        """
        with self.lock:
            others = [p for d, p in self.drone_positions.items()
                      if d != drone_id and d not in self._grounded_ids and p is not None]
        if not others:
            return aim, False

        min_horiz = min(math.hypot(ap[0] - p[0], ap[1] - p[1]) for p in others)
        if min_horiz >= EGRESS_CLEAR_RADIUS_M:
            return aim, False   # yatayda temiz -> sapma yok, düz kesişme.

        # Ne kadar içerideyiz? (0 = sınırda, 1 = merkezde) -> sapma ağırlığı.
        strength = (EGRESS_CLEAR_RADIUS_M - min_horiz) / EGRESS_CLEAR_RADIUS_M
        weight = max(0.0, min(1.0, strength)) * EGRESS_DEFLECT_GAIN

        # Nişana birim yön.
        ax_, ay_, az_ = aim[0]-ap[0], aim[1]-ap[1], aim[2]-ap[2]
        adist = math.sqrt(ax_*ax_ + ay_*ay_ + az_*az_)
        if adist < 1e-3:
            return aim, False
        tux, tuy, tuz = ax_/adist, ay_/adist, az_/adist

        # Dışa (sürü merkezinden saldırgana) yatay birim yön.
        cn = sum(p[0] for p in others) / len(others)
        ce = sum(p[1] for p in others) / len(others)
        ox, oy = ap[0] - cn, ap[1] - ce
        onorm = math.hypot(ox, oy)
        if onorm < 1e-3:
            # Saldırgan ~merkezde (ör. üst/alt slot): nişana dik bir yön seç.
            ox, oy = -tuy, tux
            onorm = math.hypot(ox, oy) or 1.0
        oux, ouy = ox / onorm, oy / onorm

        # Yön harmanı: nişan + ağırlık*(dışa + yukarı). (NED yukarı = -z.)
        dx = tux + weight * oux
        dy = tuy + weight * ouy
        dz = tuz + weight * (-1.0)   # yukarı bileşen
        dn = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        dx, dy, dz = dx/dn, dy/dn, dz/dn

        # UZAK nişanı koru: aynı mesafede, saptırılmış yönde.
        new_aim = (ap[0] + dx*adist, ap[1] + dy*adist, ap[2] + dz*adist)
        return new_aim, True

    # ------------------------------------------------------------------
    # Geçiş sonrası YAW KİLİDİ
    # ------------------------------------------------------------------
    def _locked_yaw(self, drone_id):
        """Bu drone'un donmuş baş açısı [rad], kilit yoksa None."""
        with self._yaw_lock_mu:
            st = self._yaw_locks.get(drone_id)
        return st["yaw"] if st else None

    def _cache_wp_yaw_behavior(self, drone_id):
        """
        Saldırı ÖNCESİ WP_YAW_BEHAVIOR'ı arka planda okuyup saklar; kilit
        çözülürken bu değer geri yazılır. get_param bloke ettiği için ayrı
        thread'de çalışır — commit_attacker'ı geciktirmez.
        """
        if drone_id in self._wp_yaw_behavior_saved:
            return
        def _read():
            val = self.get_param(drone_id, "WP_YAW_BEHAVIOR")
            if val is None:
                logger.warning(f"[YawLock] d{drone_id}: WP_YAW_BEHAVIOR okunamadı; "
                               f"kilit çözülürken varsayılan "
                               f"{DEFAULT_WP_YAW_BEHAVIOR:.0f} yazılacak.")
                return
            self._wp_yaw_behavior_saved[drone_id] = float(val)
            logger.info(f"[YawLock] d{drone_id}: saldırı öncesi "
                        f"WP_YAW_BEHAVIOR={val:.0f} saklandı.")
        t = threading.Thread(target=_read, name=f"YawBehaviorRead-{drone_id}",
                             daemon=True)
        t.start()
        self.threads.append(t)

    def _lock_yaw(self, drone_id, reason=""):
        """
        Baş açısını ANLIK değerinde dondurur ve kilidi izleyen thread'i başlatır.

        İki katmanlı: (1) WP_YAW_BEHAVIOR=0 -> ArduCopter'ın hedefe/rotaya doğru
        OTOMATİK yaw'ını kapatır, (2) komutlarda sabit yaw alanı -> donmuş açı
        açıkça komutlanır (yaw_rate yok sayılır). Tek başına hiçbiri yeterli
        değil: (1) olmadan otomatik yaw komutumuzla çekişir, (2) olmadan araç
        son yaw hedefine sürüklenebilir.
        """
        if not ENABLE_ATTACK_YAW_LOCK:
            return
        # Baş açısı, kilit mutex'i ALINMADAN önce okunur: iki kilidi iç içe
        # almamak için (dispatcher self.lock altında drone_headings yazıyor).
        with self.lock:
            hdg = self.drone_headings.get(drone_id)
        if hdg is None:
            logger.warning(f"[YawLock] d{drone_id}: baş açısı bilinmiyor, "
                           f"kilit KURULAMADI ({reason}).")
            return
        with self._yaw_lock_mu:
            if drone_id in self._yaw_locks:
                return                      # zaten kilitli
            self._yaw_locks[drone_id] = {"yaw": float(hdg),
                                         "t0": time.monotonic()}
        # Otomatik yaw'ı kapat (kilit boyunca yalnız bizim sabit açımız geçerli).
        self.set_param(drone_id, "WP_YAW_BEHAVIOR", 0.0)
        logger.warning(f"[YawLock] d{drone_id} KİLİTLENDİ ({reason}) — baş açısı "
                       f"{math.degrees(hdg):.0f}° donduruldu; çözülme: yatay hız "
                       f"<= {ATTACK_YAW_UNLOCK_SPEED_MPS:.0f} m/s, yeniden hücum "
                       f"veya {ATTACK_YAW_LOCK_TIMEOUT_S:.0f}s.")
        log_event("yaw_locked", drone_id=drone_id, reason=reason,
                  heading_deg=math.degrees(hdg))
        t = threading.Thread(target=self._yaw_lock_watchdog, args=(drone_id,),
                             name=f"YawLock-{drone_id}", daemon=True)
        t.start()
        self.threads.append(t)

    def _unlock_yaw(self, drone_id, reason=""):
        """Kilidi kaldırır ve WP_YAW_BEHAVIOR'ı saldırı ÖNCESİ değerine döndürür."""
        with self._yaw_lock_mu:
            st = self._yaw_locks.pop(drone_id, None)
        if st is None:
            return
        prev = self._wp_yaw_behavior_saved.get(drone_id)
        if prev is None:
            prev = DEFAULT_WP_YAW_BEHAVIOR
            logger.warning(f"[YawLock] d{drone_id}: saldırı öncesi WP_YAW_BEHAVIOR "
                           f"okunamamıştı — varsayılan {prev:.0f} yazılıyor.")
        self.set_param(drone_id, "WP_YAW_BEHAVIOR", float(prev))
        held = time.monotonic() - st["t0"]
        logger.warning(f"[YawLock] d{drone_id} SERBEST ({reason}) — {held:.1f}s kilitli kaldı.")
        log_event("yaw_unlocked", drone_id=drone_id, reason=reason, held_s=held)

    def _yaw_lock_watchdog(self, drone_id, rate_hz=10.0):
        """
        Kilidi çözecek koşulu bekler. Kesişme thread'inden AYRIDIR: saldırgan
        formasyona bırakıldıktan sonra da (asıl tehlikeli yavaşlama safhası)
        çalışmaya devam eder.

        Çözülme koşulları (ilk gerçekleşen kazanır):
          1) yatay yer hızı <= ATTACK_YAW_UNLOCK_SPEED_MPS,
          2) saldırı HÂLÂ canlıyken dönüş tamamlandı: kendi hız vektörü ile hedef
             kerterizi arasındaki açı, ATTACK_YAW_RECHASE_CONFIRM_S boyunca
             KESİNTİSİZ olarak ATTACK_YAW_RECHASE_ANGLE_DEG altında kaldı,
          3) ATTACK_YAW_LOCK_TIMEOUT_S doldu (güvenlik ağı).
        """
        dt = 1.0 / max(rate_hz, 1.0)
        t0 = time.monotonic()
        rechase_since = None       # açı eşiğinin altına ilk girilen an
        while not self.stop_threads.is_set():
            with self._yaw_lock_mu:
                if drone_id not in self._yaw_locks:
                    return          # başkası çözdü (kapanış vb.)
            if self._shutting_down.is_set():
                self._unlock_yaw(drone_id, reason="shutdown")
                return

            elapsed = time.monotonic() - t0
            if elapsed > ATTACK_YAW_LOCK_TIMEOUT_S:
                logger.warning(f"[YawLock] d{drone_id}: {ATTACK_YAW_LOCK_TIMEOUT_S:.0f}s "
                               f"içinde çözülme koşulu oluşmadı -> zorla serbest.")
                self._unlock_yaw(drone_id, reason="timeout")
                return

            with self.lock:
                pos = self.drone_positions.get(drone_id)
                vel = self.drone_velocities.get(drone_id, (0.0, 0.0, 0.0))
            tgt = self._last_target_ned

            # (1) Hız koşulu — YATAY yer hızı (tırmanma/alçalma sayılmaz).
            gnd_spd = math.hypot(vel[0], vel[1])
            if gnd_spd <= ATTACK_YAW_UNLOCK_SPEED_MPS:
                self._unlock_yaw(drone_id, reason=f"slowed_to_{gnd_spd:.1f}mps")
                return

            # (2) Yeniden hücum — SADECE saldırı hâlâ bu drone'daysa geçerli.
            # Formasyona bırakılmış bir drone için bu koşul devre dışıdır; o
            # durumda yalnız hız (1) veya zaman aşımı (3) kilidi çözer.
            if (self._committed_attacker == drone_id
                    and pos is not None and tgt is not None):
                lx, ly, lz = tgt[0]-pos[0], tgt[1]-pos[1], tgt[2]-pos[2]
                lnorm = math.sqrt(lx*lx + ly*ly + lz*lz)
                vnorm = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
                if lnorm > 1e-3 and vnorm > 1e-3:
                    # Hız vektörü ile hedefe kerteriz arasındaki açı.
                    cos_a = (lx*vel[0] + ly*vel[1] + lz*vel[2]) / (lnorm * vnorm)
                    cos_a = max(-1.0, min(1.0, cos_a))
                    angle_deg = math.degrees(math.acos(cos_a))
                    if angle_deg <= ATTACK_YAW_RECHASE_ANGLE_DEG:
                        now = time.monotonic()
                        if rechase_since is None:
                            rechase_since = now
                        elif now - rechase_since >= ATTACK_YAW_RECHASE_CONFIRM_S:
                            self._unlock_yaw(
                                drone_id,
                                reason=f"rechase_angle_{angle_deg:.0f}deg")
                            return
                    else:
                        rechase_since = None    # eşiğin dışına çıktı -> sayaç sıfır
            else:
                rechase_since = None

            time.sleep(dt)

    def _send_velocity_ned(self, target_id, vn, ve, vd, yaw=None):
        """
        Saf HIZ komutu (SET_POSITION_TARGET_LOCAL_NED, yalnız-hız maskesi).

        Konum hedefi YOK -> ArduCopter S-eğrisi konum planlayıcısı devrede değil,
        yani hedefte sıfıra FRENLEME yok. Terminal/kill safhasında hedefin İÇİNDEN
        tam hızla geçmek için kullanılır (WPNAV_ACCEL'e DOKUNMADAN hızlanma).

        NED hız bileşenleri orijinden bağımsız olduğundan LOCAL_NED çerçevesinde
        doğrudan gönderilir (konum alanları yok sayılır).

        yaw None değilse (geçiş sonrası yaw kilidi) maske yaw alanını KULLANIR ve
        verilen mutlak baş açısı [rad] komutlanır; yaw_rate yine yok sayılır.
        """
        mask = VELOCITY_ONLY_MASK if yaw is None else VELOCITY_YAW_MASK
        self.master.mav.set_position_target_local_ned_send(
            0, target_id, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            0.0, 0.0, 0.0,              # konum (yok sayılır)
            float(vn), float(ve), float(vd),
            0.0, 0.0, 0.0,              # ivme (yok sayılır)
            float(yaw or 0.0), 0.0,     # yaw [rad] (kilitliyse geçerli), yaw_rate yok sayılır
        )

    def _release_attacker(self, drone_id, reason=""):
        """
        Saldırganı formasyona geri döndürür.

        DİKKAT: yaw kilidi burada BİLEREK çözülmez. Release çoğu zaman geçişten
        hemen sonra (drone hâlâ ~22 m/s) tetiklenir ve formasyon slotu ARKADADIR;
        kilidi burada açmak tam da önlemek istediğimiz tam-gaz 180° dönüşünü
        yaratır. Kilidi _yaw_lock_watchdog çözer: yatay hız 13 m/s'ye inince
        (release sonrası bu koşul zaten hızla oluşur) veya 10 sn zaman aşımında.
        """
        logger.info(f"[Attack] Drone {drone_id} saldırıdan serbest ({reason}) "
                    f"-> takipci moduna donuyor.")
        log_event("attacker_released", drone_id=drone_id, reason=reason)
        if self._committed_attacker == drone_id:
            self._committed_attacker = None
        self._attack_skip.discard(drone_id)
        # Formasyon uçuşu için aracın KENDİ varsayılan hız/ivme profilini
        # (MAX_SPEED tavanıyla) geri yükle — released attacker sürüyle aynı hızda uçar.
        self._restore_default_dynamics(drone_id)
        # Bekleme: başarısız denemeden hemen sonra bu drone'u tekrar seçme.
        if reason not in ("hit", "survived_hit"):
            self._attack_cooldown[drone_id] = time.monotonic() + ATTACK_COOLDOWN_S

    def _commit_next_attacker(self):
        """Sıradaki uygun saldırganı seçip atar (yeniden saldırı devri)."""
        # F: mümkünse hedef-tarafı seçimi için son bilinen hedef konumunu kullan.
        nxt = self._pick_attacker(target_ned=self._last_target_ned)
        if nxt is None:
            logger.warning("[Attack] Uygun başka saldırgan yok.")
            return
        self.commit_attacker(nxt, min_alt_m=self._attacker_min_alt)

    def sequential_landing(self, per_drone_gap_s=2.0, verify_s=6.0):
        """
        Sürüyü kademeli olarak RTL'e alır ve HEPSİNİN gerçekten RTL'e geçtiğini
        DOĞRULAR.
        """
        logger.info("--- Sıralı iniş prosedürü ---")

        self._shutting_down.set()
        self.pause_formation_following()
        time.sleep(0.3)   # thread'lerin son tikini bitirmesine izin ver

        order = [self.leader_id]
        if self._committed_attacker is not None and self._committed_attacker != self.leader_id:
            order.append(self._committed_attacker)
        order += [d for d in self.drone_ids
                  if d != self.leader_id and d != self._committed_attacker]

        for did in order:
            if did not in self._alive_ids:
                logger.info(f"[SeqLand] Drone {did} atlandı (hayatta değil).")
                continue
            mode_now = self.drone_modes.get(did, "")
            if mode_now == "LAND":
                logger.info(f"[SeqLand] Drone {did} zaten LAND — atlandı (hasarlı).")
                continue
            logger.info(f"[SeqLand] Drone {did} -> RTL")
            log_event("sequential_rtl", drone_id=did)
            self.set_mode(did, "RTL")
            time.sleep(per_drone_gap_s)

        # 3) DOĞRULAMA: RTL'e geçmeyen kaldıysa tekrar dene.
        deadline = time.time() + verify_s
        pending = set()
        while time.time() < deadline:
            pending = set()
            for did in self.drone_ids:
                if did not in self._alive_ids:
                    continue
                m = self.drone_modes.get(did, "")
                if m not in ("RTL", "LAND"):
                    pending.add(did)
            if not pending:
                logger.info("[SeqLand] DOĞRULANDI: tüm araçlar RTL/LAND modunda.")
                return
            for did in pending:
                logger.warning(
                    f"[SeqLand] Drone {did} hâlâ RTL'de değil "
                    f"(mod={self.drone_modes.get(did,'?')}) — RTL tekrar gönderiliyor."
                )
                self.set_mode(did, "RTL")
            time.sleep(1.0)

        if pending:
            logger.error(
                f"[SeqLand] UYARI: {sorted(pending)} araçları {verify_s:.0f}s içinde "
                f"RTL'e geçmedi! Kumandadan manuel müdahale gerekebilir."
            )

    def stop_all(self):
        """Tüm thread'leri durdurur ve bağlantıyı kapatır."""
        logger.info("Tüm thread'ler durduruluyor...")
        self._shutting_down.set()   # komut gönderen thread'ler derhal dursun

        # Açık kalmış yaw kilitleri: WP_YAW_BEHAVIOR araçta 0 kalmasın.
        # stop_threads'ten ÖNCE yapılır, aksi halde izleyici thread'ler geri
        # yükleme yapmadan çıkar.
        for did in list(self._yaw_locks.keys()):
            self._unlock_yaw(did, reason="stop_all")

        self.stop_threads.set()

        # simple_guided_follow subprocess'i hâlâ çalışıyorsa sonlandır
        if self._follow_proc is not None and self._follow_proc.poll() is None:
            logger.info("simple_guided_follow süreci sonlandırılıyor...")
            self._follow_proc.terminate()
            try:
                self._follow_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._follow_proc.kill()
            logger.info("simple_guided_follow süreci durduruldu.")

        for t in self.threads:
            try:
                if t.is_alive():
                    t.join(timeout=3)
            except Exception:
                logger.exception("Thread join hatası.")
        try:
            self.master.close()
        except Exception:
            logger.exception("MAVLink bağlantısı kapatılırken hata.")
        logger.info("Bağlantı kapatıldı.")


# =========================
# Yardımcı fonksiyonlar
# =========================

def load_waypoints_from_file(filename="waypoints.json") -> list:
    """Waypoint'leri bir JSON dosyasından yükler.  """
    logger.info(f"Waypoints okunuyor: {filename}")
    try:
        with open(filename, 'r', encoding="utf-8") as f:
            waypoints = json.load(f)
        if not waypoints:
            logger.warning("Waypoint dosyası boş.")
            return []
        logger.info(f"{len(waypoints)} adet waypoint yüklendi.")
        return waypoints
    except FileNotFoundError:
        logger.error(f"'{filename}' bulunamadı, görev atlanacak.")
        return []
    except json.JSONDecodeError:
        logger.error(f"'{filename}' geçerli JSON değil.")
        return []
    except Exception:
        logger.exception("Waypoints okunurken beklenmeyen hata.")
        return []


def latlon_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    """
    Düz-dünya equirectangular yaklaşımı.
    Orijine göre metre cinsinden (north, east, down) döndürür.
    """
    R_EARTH = 111320.0
    north = (lat - origin_lat) * R_EARTH
    east  = (lon - origin_lon) * R_EARTH * math.cos(math.radians(origin_lat))
    down  = -(alt - origin_alt)
    return north, east, down

def ned_to_latlon(north, east, down, origin_lat, origin_lon, origin_alt):
    """
    latlon_to_ned'in tersi: paylaşılan NED çerçevesindeki (north, east, down)
    noktasını (lat, lon, alt AMSL) olarak döndürür.
    """
    R_EARTH = 111320.0
    lat = origin_lat + north / R_EARTH
    lon = origin_lon + east / (R_EARTH * math.cos(math.radians(origin_lat)))
    alt = origin_alt - down
    return lat, lon, alt


def build_takeoff_altitudes(
    base_altitude: float,
    vertical_offsets: dict,
    drone_ids: list,
    ) -> dict:
    """
    Drone başına kalkış irtifasını döndürür.
    vertical_offset > 0 → drone liderden daha yükseğe kalkar.  
    """
    return {
        did: base_altitude + float(vertical_offsets.get(did, 0.0))
        for did in drone_ids
    }

# =========================
# Main
# =========================
def main():
    # --- AYARLAR ---
    CONNECTION_PORT      = 14554
    TAKEOFF_ALTITUDE     = 40.0
    # CANLI: bu yalnızca BAŞLANGIÇ değeridir ve guidance_config.py'deki
    # FORMATION_OFFSET_M'den okunur. Uçuş sırasında değiştirmek için BU satırı
    # DEĞİL, guidance_config.py'deki FORMATION_OFFSET_M satırını düzenleyip
    # kaydedin -- formasyon yöneticisi ~0.5 s içinde yeni aralığa rampalanır.
    # (Buradaki 10.0 yalnızca guidance_config bulunamazsa devreye girer.)
    FORMATION_OFFSET     = FORMATION_OFFSET_LIVE.target()
    SELECTED_FORMATION   = Formation.DIAMOND
    SYMMETRIC_HORIZONTAL = True

    LEADER_MIN_ALT_M     = 30.0
    FOLLOWER_MIN_ALT_M   = 25.0
    SLOT_REASSIGN_THRESHOLD_M = 14.0 # Slot yeniden atama eşiği — bir drone, mevcut slotundan bu kadar (metre) daha yakın bir slot varsa yeni slota geçer
    # Çok küçük → sürekli takas, çok büyük → yavaş tepki

    LEADER_LOOP_START_WP = 2
    LEADER_LOOP_ARRIVAL_DIST_M = 2.0
    WP_SPEED = 8.0 # AUTO modunda lider drone'un uçuş hızı [m/s]. 0 olarak ayarlanırsa hız komutu gönderilmez.

    GUIDED_FOLLOW_SCRIPT = "simple_guided_follow.sh"

    # --- Güvenlik ayarları ---
    GPS_MIN_SATS             = 10
    GPS_GATE_TIMEOUT_S       = 40.0

    VERTICAL_OFFSETS = {
        3:  4.0,   # üst gözlemci   -> liderin 4 m ÜSTÜ
        5:  2.0,   # kanat saldırgan -> liderin 2 m ÜSTÜ
        2: -2.0,   # kanat saldırgan -> liderin 2 m ALTI
        4: -4.0,   # alt gözlemci   -> liderin 4 m ALTI   444444444444444
    }

    TAKEOFF_ALTITUDES = build_takeoff_altitudes(
        TAKEOFF_ALTITUDE, VERTICAL_OFFSETS, DRONE_IDS
    )
    logger.info(f"Takeoff alts per drone: {TAKEOFF_ALTITUDES}")

    REDIS_HOST       = config.REDIS_HOST   # tek kaynak: controller.json (config.py)
    REDIS_PORT       = config.REDIS_PORT
    CAMERA_REDIS_KEY = 'leader_cam_frame'

    redis_client_img = redis.Redis(
        host=REDIS_HOST, port=REDIS_PORT, db=0,
        socket_timeout=0.1,
        socket_connect_timeout=2.0,
    )

    _latest_frame = [None]
    _camera_running = [True]

    def _camera_thread():
        error_count = 0
        last_error_log = 0.0
        while _camera_running[0]:
            try:
                frame_data = redis_client_img.get(CAMERA_REDIS_KEY)
                if frame_data:
                    nparr = np.frombuffer(frame_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        _latest_frame[0] = frame
                        error_count = 0
                    else:
                        logger.warning("OpenCV görüntüyü çözemedi (bozuk byte).")
            except Exception as e:
                # Redis timeout veya bağlantı hatası — her hatayı sessizce
                # yutmak yerine ara sıra logla.
                error_count += 1
                now = time.time()
                if now - last_error_log > 5.0:
                    logger.warning(f"Kamera Redis hatası ({error_count}x): {e}")
                    last_error_log = now
            time.sleep(0.033)   # ~30 Hz alma hızı

    # Waypoint'leri JSON'dan yükle  
    WAYPOINT_LIST = load_waypoints_from_file("waypoints.json")

    cam_thread = threading.Thread(target=_camera_thread, name="CameraFetch", daemon=True)
    cam_thread.start()

    swarm = None
    try:
        swarm = SwarmController(
            LEADER_ID,
            CONNECTION_PORT,
            DRONE_IDS,
            symmetric_horizontal=SYMMETRIC_HORIZONTAL,
        )
        logger.info(f"SEÇİLEN FORMASYON: {SELECTED_FORMATION.value}")

        # Canlı formasyon görselleştirme paneli (viz_dashboard.py) için durum yayıncısını başlat
        redis_state_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)

        _stale_keys = [
            "leader_slot_ned", "attacker_slot_ned", "attacker_state_ned",
            "leader_state_ned", "dogru_rakip_telemetri", "yanlis_rakip_telemetri",
            "hedef_status", "ned_origin",
        ]
        for _k in _stale_keys:
            try:
                if redis_state_client.delete(_k):
                    logger.warning(f"[RedisFlush] Bayat anahtar silindi: '{_k}'")
            except Exception:
                logger.exception(f"[RedisFlush] '{_k}' silinemedi")
        logger.info("[RedisFlush] Önceki koşudan kalan anahtarlar temizlendi.")
        # Testten önce şunu çalıştır:  pkill -f simple_guided_follow

        swarm.start_state_publisher(redis_state_client, rate_hz=5.0, jsonl_rate_hz=1.0)
        swarm.setup_high_rate_telemetry(DRONE_IDS)
        swarm.apply_global_speed_limit(DRONE_IDS)
        swarm.gps_gate_all(DRONE_IDS, min_fix=3, min_sats=GPS_MIN_SATS, timeout=GPS_GATE_TIMEOUT_S)

        redis_origin = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)
        swarm.fetch_and_publish_ned_origin(LEADER_ID, redis_origin)

        # YENİ: drone başına >=20 Hz uçuş CSV kaydını başlat (Vuruş Tespiti kaydı).
        swarm.start_flight_csv_loggers(DRONE_IDS, rate_hz=25.0)

        # Drone başına irtifalarla paralel kalkış
        logger.info("Paralel kalkış başlıyor...")
        swarm.parallel_launch(DRONE_IDS, TAKEOFF_ALTITUDES)

        swarm.start_heartbeat_watchdog(timeout_s=LEADER_HB_TIMEOUT_S)
        swarm.start_leader_position_watchdog(stale_timeout_s=LEADER_POS_LOITER_S)

        swarm.enable_leader_failover()
        swarm.start_impact_monitor(DRONE_IDS)

        redis_pursuer = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0)
        swarm.start_pursuer_state_publisher(redis_pursuer, rate_hz=20.0)
        swarm._attacker_min_alt = FOLLOWER_MIN_ALT_M
        swarm._attacker_max_alt = config.MAX_ALT_M   # saldırgan irtifa tavanı (controller.json)

        # swarm.start_keyboard_listener()
        # logger.info(
        #     "Güvenlik sistemleri aktif: "
        #     "Heartbeat watchdog, Lider konum bekçisi, Klavye dinleyicisi (r=RTL, l=LAND, q=Çıkış)"
        # )

        # Formasyon takibini başlat
        logger.info("--- Formasyon Takibi Başlatılıyor ---")
        swarm.start_formation_following(
            swarm.leader_id, SELECTED_FORMATION, FORMATION_OFFSET, VERTICAL_OFFSETS, slot_reassign_threshold_m=SLOT_REASSIGN_THRESHOLD_M, follower_min_alt_m=FOLLOWER_MIN_ALT_M,
        )
        logger.info("Takipçi drone'lar lideri 3D takip ediyor.")

        time.sleep(5)

        logger.info("--- simple_guided_follow başlatılıyor (lider drone uçak takibine geçiyor) ---")
        swarm.launch_guided_follow(
            GUIDED_FOLLOW_SCRIPT,
            min_alt_m=LEADER_MIN_ALT_M,
            loop_start_wp_index=LEADER_LOOP_START_WP,
            loop_arrival_dist_m=LEADER_LOOP_ARRIVAL_DIST_M,
            wp_speed_mps=WP_SPEED,
        )
        logger.info("simple_guided_follow arka planda çalışıyor.")

        
        SHOW_CAMERA = False   # ekransız (cv2 penceresi olmadan) çalışmak için False yap
        
        rc_target = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0,
                                decode_responses=True)

        if ENABLE_KILL_STRATEGY:
            logger.info("ENABLE_KILL_STRATEGY=True -> saldırı mantığı AKTİF.")
        else:
            logger.info("ENABLE_KILL_STRATEGY=False -> yalnızca kalkış/formasyon/iniş.")

        if SHOW_CAMERA:
            cv2.namedWindow("Leader Drone Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Leader Drone Camera", 640, 480)
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Goruntu bekleniyor...", (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        last_no_frame_warn = 0.0

        def _kill_check():
            """Otonom saldırgan atama kontrolünün tek bir iterasyonu."""
            if not ENABLE_KILL_STRATEGY:
                return
            if swarm._committed_attacker is not None:
                return
            try:
                raw = rc_target.get("dogru_rakip_telemetri")
                if not raw:
                    return
                tj = json.loads(raw)
                if not (tj.get("valid", True) and "lat" in tj):
                    return
                origin = swarm._ned_origin
                with swarm.lock:
                    lp = swarm.drone_positions.get(swarm.leader_id)
                if origin and lp:
                    tn, te, td = latlon_to_ned(
                        tj["lat"], tj["lon"],
                        origin[2] + tj.get("alt", 0.0), *origin)
                    # F: son bilinen hedef NED'i sakla (yeniden saldırı devri de kullanır).
                    swarm._last_target_ned = (tn, te, td)
                    rng = math.sqrt((lp[0]-tn)**2 + (lp[1]-te)**2)
                    if rng <= COMMIT_RANGE_M:
                        atk = swarm._pick_attacker(target_ned=(tn, te, td))
                        if atk is not None:
                            logger.warning(f"[Kill] Hedef menzilde ({rng:.0f}m) "
                                           f"-> saldırgan {atk} atanıyor.")
                            swarm.commit_attacker(
                                atk, redis_host=REDIS_HOST,
                                redis_port=REDIS_PORT, min_alt_m=FOLLOWER_MIN_ALT_M)
            except Exception:
                logger.exception("[Kill] kontrol döngüsü hatası.")

        while not swarm.stop_threads.is_set():
            _kill_check()

            if SHOW_CAMERA:
                frame = _latest_frame[0]
                if frame is not None:
                    if frame.max() == 0:
                        logger.warning("Görüntü siyah!")
                    cv2.imshow("Leader Drone Camera", frame)
                else:
                    cv2.imshow("Leader Drone Camera", placeholder)
                    now = time.time()
                    if now - last_no_frame_warn > 5.0:
                        logger.warning(
                            f"Henüz kamera görüntüsü yok (Redis key='{CAMERA_REDIS_KEY}'). "
                            f"leader_to_redis.py çalışıyor mu ve doğru drone topic'ine "
                            f"abone mi? (Lider = drone {LEADER_ID})")
                        last_no_frame_warn = now
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    logger.info("Kamera penceresi kapatıldı.")
                    break
            else:
                time.sleep(0.1)

    except (ConnectionError, KeyboardInterrupt):
        logger.exception("PROGRAM KESİLDİ / BAĞLANTI HATASI.")
    except Exception:
        logger.exception("BEKLENMEYEN HATA.")

    finally:
        _camera_running[0] = False

        try:
            import signal as _signal
            _signal.signal(_signal.SIGINT, _signal.SIG_IGN)
            logger.info("Kapanış başladı — ek Ctrl+C sinyalleri yok sayılıyor.")
        except Exception:
            pass

        if swarm:
            logger.info("--- Görev Sonu: Sıralı İniş Prosedürü ---")
            
            try:
                swarm.sequential_landing(per_drone_gap_s=2.0, verify_s=6.0)
            except Exception:
                logger.exception("Sıralı iniş sırasında hata — yine de RTL denenecek.")
                for _d in swarm.drone_ids:
                    try: swarm.set_mode(_d, "RTL")
                    except Exception: pass

            logger.info("Sıralı iniş komutları gönderildi. İniş için bekleniyor (25s)...")
            try:
                time.sleep(25)
            except Exception:
                pass
            swarm.stop_all()
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

if __name__ == "__main__":
    main()