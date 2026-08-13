#!/usr/bin/env python3
"""Gimbal uyumlu, erisilebilir LOS/PN terminal gudumu.

Konumlu gudum hedefin arkasindaki slotu kurar. Bu kontrolcu yalnizca goruntulu
devirden sonra calisir ve o geometrinin kalan isini yapar:

* YERLES: LOS oranini PN yanal ivmesiyle kucultur.
* VUR: LOS yeterince kararlilastiginda kapanma hizini arttirir.
* DON: kalan sure eyleyici gecikmesinden kisa oldugunda son erisilebilir
  carpisma komutunu dondurur; hedeften gecmis bir ufku optimize edip ters
  komut uretmez.

Hedeften kullanilan tek telemetri buyuklugu ``Olcum.menzil_m``'dir. Hedef
hizi, rotasi ve ivmesi kullanilmaz. Diger girdiler kamera LOS'u ve aracin
kendi durumudur.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from goruntulu_temel import (GoruntuluDongu, GoruntuluKontrolcu, Komut,
                             govde_ileri_ned)
from los_gudum import TurevSuzgec, eps_coz, kelepce, wrap180


def _aci_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    c = float(np.dot(a, b)) / (na * nb)
    return math.degrees(math.acos(kelepce(c, -1.0, 1.0)))


def _yatay_koni(v_simdi: np.ndarray, v_istek: np.ndarray,
                azami_aci_deg: float) -> np.ndarray:
    """Istenen yatay yonu mevcut hiz etrafindaki guvenli koniye izdusur."""
    v = np.asarray(v_simdi, float).copy()
    u = np.asarray(v_istek, float).copy()
    hv, hu = v[:2], u[:2]
    nv, nu = float(np.linalg.norm(hv)), float(np.linalg.norm(hu))
    if nv < 1.0 or nu < 1e-9:
        return u
    a0, a1 = math.atan2(hv[1], hv[0]), math.atan2(hu[1], hu[0])
    fark = math.radians(wrap180(math.degrees(a1 - a0)))
    sinir = math.radians(float(azami_aci_deg))
    if abs(fark) <= sinir:
        return u
    a = a0 + kelepce(fark, -sinir, sinir)
    u[:2] = nu * np.array([math.cos(a), math.sin(a)])
    return u


class TerminalLosKontrolcu(GoruntuluKontrolcu):
    """PN + kapanma itkisi + erisilebilir komut konisi."""

    ad = "terminal_los"

    def __init__(
            self,
            n_pn=4.0,
            pn_kapanma_taban_mps=10.0,
            a_yatay_max_mps2=5.0,
            a_dikey_max_mps2=1.5,
            a_jerk_z_mps3=4.0,
            komut_ufku_s=0.70,
            komut_aci_max_deg=45.0,
            v_max_mps=35.0,
            yerles_ivme_mps2=0.5,
            vur_ivme_mps2=4.0,
            kapanma_hedef_mps=7.0,
            kapanma_kp=0.35,
            yerles_los_dps=4.0,
            yerles_ex_deg=12.0,
            yerles_sure_s=0.45,
            vur_zorunlu_menzil_m=18.0,
            vur_zorunlu_los_dps=7.0,
            vur_kapanma_min_mps=1.0,
            vur_cikis_los_dps=11.0,
            terminal_menzil_m=3.0,
            terminal_tgo_s=0.25,
            terminal_yatay_duzeltme_mps2=1.0,
            dikey_tau_s=1.4,
            dikey_hiz_tau_s=0.8,
            dikey_ac_m=1.2,
            dikey_kapa_m=0.6,
            dikey_terminal_menzil_m=8.0,
            tirmanma_hiz_max_mps=2.5,
            alcalma_hiz_max_mps=2.0,
            tau_turev_s=0.22,
            tau_menzil_s=0.40,
            yaw_kp=0.70,
            yaw_kd=0.85,
            yaw_rate_max_dps=75.0,
            yaw_olu_bant_deg=0.7,
            yaw_komutu_ver=True,
            aim_deg=0.0,
            min_irtifa_m=15.0,
            iska_arm_m=20.0,
            iska_acilma_m=8.0,
            iska_acilma_hizi_mps=2.0,
            iska_onay_dongu=3):
        self.n_pn = float(n_pn)
        self.pn_kapanma_taban = float(pn_kapanma_taban_mps)
        self.a_yatay_max = float(a_yatay_max_mps2)
        self.a_dikey_max = float(a_dikey_max_mps2)
        self.a_jerk_z = float(a_jerk_z_mps3)
        self.komut_ufku = float(komut_ufku_s)
        self.komut_aci_max = float(komut_aci_max_deg)
        self.v_max = float(v_max_mps)
        self.yerles_ivme = float(yerles_ivme_mps2)
        self.vur_ivme = float(vur_ivme_mps2)
        self.kapanma_hedef = float(kapanma_hedef_mps)
        self.kapanma_kp = float(kapanma_kp)
        self.yerles_los = float(yerles_los_dps)
        self.yerles_ex = float(yerles_ex_deg)
        self.yerles_sure = float(yerles_sure_s)
        self.vur_zorunlu_menzil = float(vur_zorunlu_menzil_m)
        self.vur_zorunlu_los = float(vur_zorunlu_los_dps)
        self.vur_kapanma_min = float(vur_kapanma_min_mps)
        self.vur_cikis_los = float(vur_cikis_los_dps)
        self.terminal_menzil = float(terminal_menzil_m)
        self.terminal_tgo = float(terminal_tgo_s)
        self.terminal_yatay_duzeltme = float(terminal_yatay_duzeltme_mps2)
        self.dikey_tau = float(dikey_tau_s)
        self.dikey_hiz_tau = float(dikey_hiz_tau_s)
        self.dikey_ac = float(dikey_ac_m)
        self.dikey_kapa = float(dikey_kapa_m)
        self.dikey_terminal_menzil = float(dikey_terminal_menzil_m)
        self.tirmanma_hiz_max = float(tirmanma_hiz_max_mps)
        self.alcalma_hiz_max = float(alcalma_hiz_max_mps)
        self.tau_turev = float(tau_turev_s)
        self.tau_menzil = float(tau_menzil_s)
        self.yaw_kp = float(yaw_kp)
        self.yaw_kd = float(yaw_kd)
        self.yaw_rate_max = float(yaw_rate_max_dps)
        self.yaw_olu_bant = float(yaw_olu_bant_deg)
        self.yaw_komutu_ver = bool(yaw_komutu_ver)
        self.aim_deg = float(aim_deg)
        self.min_irtifa = float(min_irtifa_m)
        self.iska_arm = float(iska_arm_m)
        self.iska_acilma = float(iska_acilma_m)
        self.iska_acilma_hizi = float(iska_acilma_hizi_mps)
        self.iska_onay_dongu = int(iska_onay_dongu)
        self.tani = {}
        self.tohumla(None)

    def tohumla(self, devir):
        self.d_ex = TurevSuzgec(self.tau_turev)
        self.d_eps = TurevSuzgec(self.tau_turev)
        self.d_yaw = TurevSuzgec(self.tau_turev)
        self.d_menzil = TurevSuzgec(self.tau_menzil)
        self.d_rz = TurevSuzgec(0.35)
        self.faz = "YERLES"
        self._faz_onceki = self.faz
        self._yerles_sayaci_s = 0.0
        self._imza = None
        self._t_son_yeni = None
        self._t_onceki = None
        self._menzil_son = None
        self._dikey_aktif = False
        self._a_z = 0.0
        self._donmus_v = None
        self._vurus_olayi = False
        self._en_iyi_r = float("inf")
        self._iska_sayac = 0
        self._devir_v = None
        if devir and devir.get("cmd_vel_ned") is not None:
            try:
                self._devir_v = np.asarray(devir["cmd_vel_ned"], float)
            except (TypeError, ValueError):
                self._devir_v = None
        if devir and devir.get("range_m") is not None:
            try:
                self._menzil_son = float(devir["range_m"])
            except (TypeError, ValueError):
                pass

    def _erisilebilir(self, v_simdi, v_ham):
        """Ivme zarfi, yon konisi ve hiz tavanini sirayla uygula."""
        v = np.asarray(v_simdi, float).reshape(3)
        u = _yatay_koni(v, np.asarray(v_ham, float).reshape(3),
                        self.komut_aci_max)
        dv = u - v
        dv_tavan = self.a_yatay_max * self.komut_ufku
        nd = float(np.linalg.norm(dv))
        if nd > dv_tavan:
            u = v + dv * (dv_tavan / nd)
        n = float(np.linalg.norm(u))
        if n > self.v_max:
            u *= self.v_max / n
        # Dikey kanal yatay carpisma yasasini ve sonraki konumlu toparlanmayi
        # bozamaz. NED'de negatif=tirmanma, pozitif=alcalma.
        u[2] = kelepce(float(u[2]), -self.tirmanma_hiz_max,
                       self.alcalma_hiz_max)
        # Sayisal son savunma: hareketli bir araca asla ters yarikure komutu.
        if float(np.linalg.norm(v)) > 2.0 and float(np.dot(u, v)) < 0.0:
            u = v.copy()
        return u

    def _dikey_ivme(self, r, eps_deg, vz, dt, terminal):
        """Bagil dikey konum/turevden sönümlü takip ivmesi [NED]."""
        r_z = -float(r) * math.sin(math.radians(float(eps_deg)))
        r_z_nokta = self.d_rz.guncelle(r_z, dt)
        if abs(r_z) >= self.dikey_ac:
            self._dikey_aktif = True
        elif abs(r_z) <= self.dikey_kapa:
            self._dikey_aktif = False

        if not self._dikey_aktif or terminal:
            ham = 0.0
        else:
            # r_z = z_hedef-z_biz.  r_z_dot = vz_hedef-vz_biz.
            # vz_istek = vz_hedef + r_z/tau -> hata = r_z_dot+r_z/tau.
            hiz_hatasi = r_z_nokta + r_z / max(self.dikey_tau, 1e-3)
            ham = kelepce(hiz_hatasi / max(self.dikey_hiz_tau, 1e-3),
                          -self.a_dikey_max, self.a_dikey_max)
        adim = self.a_jerk_z * dt
        self._a_z += kelepce(ham - self._a_z, -adim, adim)
        return self._a_z, r_z, r_z_nokta

    def komut(self, o) -> Komut:
        dt = kelepce(float(o.dt), 1e-3, 0.30)
        ex = 0.0 if o.ex_deg is None else float(o.ex_deg)
        ey = 0.0 if o.ey_deg is None else float(o.ey_deg)
        eps = kelepce(eps_coz(ex, ey, self.aim_deg), -80.0, 80.0)

        yaw_deg = None if o.yaw_rad is None else math.degrees(o.yaw_rad)
        yaw_nokta = 0.0
        if yaw_deg is not None:
            if self.d_yaw.x_onceki is not None:
                yaw_deg = self.d_yaw.x_onceki + wrap180(
                    yaw_deg - self.d_yaw.x_onceki)
            yaw_nokta = self.d_yaw.guncelle(yaw_deg, dt)

        imza = o.t_capture if o.t_capture is not None else (
            o.ex_deg, o.ey_deg, o.bbox_w, o.bbox_h)
        yeni = imza != self._imza
        self._imza = imza
        t = float(o.t)
        dt_olcum = dt if self._t_son_yeni is None else kelepce(
            t - self._t_son_yeni, 1e-3, 0.50)
        if self._t_onceki is not None and t - self._t_onceki > 0.7:
            self.d_ex.sifirla()
            self.d_eps.sifirla()
            self.d_menzil.sifirla()
            self.d_rz.sifirla()
            dt_olcum = dt
        self._t_onceki = t

        if o.menzil_m is not None and math.isfinite(float(o.menzil_m)):
            r = kelepce(float(o.menzil_m), 0.5, 500.0)
            self._menzil_son = r
        else:
            r = self._menzil_son if self._menzil_son is not None else 40.0

        if yeni:
            d_ex = self.d_ex.guncelle(ex, dt_olcum)
            d_eps = self.d_eps.guncelle(eps, dt_olcum)
            d_r = self.d_menzil.guncelle(r, dt_olcum)
            self._t_son_yeni = t
        else:
            d_ex, d_eps, d_r = self.d_ex.d, self.d_eps.d, self.d_menzil.d
        q_az = d_ex + yaw_nokta
        kapanma = max(-d_r, 0.0)
        tgo = r / kapanma if kapanma > 0.5 else None

        v_ned = (np.asarray(o.vel_ned, float).reshape(3) if o.vel_ned is not None
                 else (self._devir_v.copy() if self._devir_v is not None
                       else np.zeros(3)))
        yaw = 0.0 if o.yaw_rad is None else float(o.yaw_rad)
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_h = np.array([cy * v_ned[0] + sy * v_ned[1],
                        -sy * v_ned[0] + cy * v_ned[1], v_ned[2]])

        # Faz makinesi. VUR sert virajda tekrar YERLES'e donebilir; DON latch.
        terminal = (r <= self.terminal_menzil
                    or (tgo is not None and tgo <= self.terminal_tgo))
        if terminal:
            self.faz = "DON"
        elif self.faz == "VUR" and (abs(q_az) > self.vur_cikis_los
                                     or (d_r > 0.0
                                         and r > self.vur_zorunlu_menzil)):
            self.faz = "YERLES"
            self._yerles_sayaci_s = 0.0
        elif self.faz == "YERLES":
            uygun = (abs(q_az) <= self.yerles_los
                     and abs(ex) <= self.yerles_ex
                     and kapanma >= self.vur_kapanma_min)
            self._yerles_sayaci_s = (self._yerles_sayaci_s + dt
                                      if uygun else max(0.0,
                                                        self._yerles_sayaci_s-dt))
            if (self._yerles_sayaci_s >= self.yerles_sure
                    or (r <= self.vur_zorunlu_menzil
                        and abs(q_az) <= self.vur_zorunlu_los
                        and kapanma >= self.vur_kapanma_min)):
                self.faz = "VUR"

        # Ilk gecisten sonra kamerayla donup kafa-kafaya ikinci saldiri yapma.
        # Arkayi yeniden kurmak konumlu katmanin gorevidir.
        self._en_iyi_r = min(self._en_iyi_r, r)
        iska_kosulu = (self._en_iyi_r <= self.iska_arm
                       and r >= self._en_iyi_r + self.iska_acilma
                       and d_r >= self.iska_acilma_hizi)
        self._iska_sayac = self._iska_sayac + 1 if iska_kosulu else 0
        if self._iska_sayac >= self.iska_onay_dongu:
            sebep = (f"gecis sonrasi acilma: en_iyi={self._en_iyi_r:.1f}m "
                     f"simdi={r:.1f}m dr={d_r:+.1f}mps")
            return Komut(vel_ned=v_ned.copy(), yaw_rate_dps=None,
                         birak=True, birak_sebep=sebep,
                         olay="iska_birak", olay_detay=sebep)

        los_c = math.radians(ex)
        u_los = np.array([math.cos(los_c), math.sin(los_c)])
        u_sag = np.array([-math.sin(los_c), math.cos(los_c)])
        vc = max(kapanma, self.pn_kapanma_taban)
        a_yan = self.n_pn * vc * math.radians(q_az)
        lat_tavan = (self.terminal_yatay_duzeltme if self.faz == "DON"
                     else self.a_yatay_max)
        a_yan = kelepce(a_yan, -lat_tavan, lat_tavan)
        if self.faz == "YERLES":
            a_ileri = self.yerles_ivme
        elif self.faz == "VUR":
            a_ileri = self.vur_ivme + self.kapanma_kp * (
                self.kapanma_hedef - kapanma)
            a_ileri = kelepce(a_ileri, 0.0, self.a_yatay_max)
        else:
            a_ileri = 0.0
        # Buyuk yon hatasinda gaz vermek donus yaricapini buyutur.
        if abs(q_az) > self.vur_cikis_los:
            a_ileri = 0.0
        a_h = a_ileri * u_los + a_yan * u_sag
        na = float(np.linalg.norm(a_h))
        if na > self.a_yatay_max:
            a_h *= self.a_yatay_max / na

        a_z, r_z, r_z_nokta = self._dikey_ivme(
            r, eps, float(v_h[2]), dt_olcum if yeni else dt,
            self.faz == "DON" or r <= self.dikey_terminal_menzil)
        v_ham_h = v_h.copy()
        v_ham_h[:2] += self.komut_ufku * a_h
        v_ham_h[2] += self.komut_ufku * a_z
        v_ham = govde_ileri_ned(yaw, v_ham_h[0], v_ham_h[1], v_ham_h[2])

        if self.faz == "DON":
            if self._donmus_v is None:
                self._donmus_v = self._erisilebilir(v_ned, v_ham)
            v_istek = self._donmus_v.copy()
        else:
            self._donmus_v = None
            v_istek = v_ham
        v_cmd = self._erisilebilir(v_ned, v_istek)
        if o.pos_ned is not None and -float(o.pos_ned[2]) < self.min_irtifa:
            v_cmd[2] = min(v_cmd[2], 0.0)

        e_yaw = 0.0 if abs(ex) <= self.yaw_olu_bant else (
            ex - math.copysign(self.yaw_olu_bant, ex))
        yaw_rate = kelepce(self.yaw_kp * e_yaw + self.yaw_kd * d_ex,
                           -self.yaw_rate_max, self.yaw_rate_max)

        olay = ""
        detay = ""
        if self.faz != self._faz_onceki:
            olay = f"faz_{self.faz.lower()}"
            detay = (f"{self._faz_onceki}->{self.faz} r={r:.1f}m "
                     f"qaz={q_az:.1f}dps kapanma={kapanma:.1f}mps")
            self._faz_onceki = self.faz
        if (not self._vurus_olayi and o.vibe_max is not None
                and float(o.vibe_max) > 10.0 and r < 3.0):
            olay, detay = "vurus_basarili", f"vibe={o.vibe_max:.1f} r={r:.2f}m"
            self._vurus_olayi = True

        self.tani = {
            "faz": self.faz, "r": r, "d_r": d_r, "kapanma": kapanma,
            "tgo": tgo, "ex": ex, "eps": eps, "d_ex": d_ex,
            "d_eps": d_eps, "yaw_nokta": yaw_nokta, "q_az": q_az,
            "a_ileri": a_ileri, "a_yan": a_yan, "a_z": a_z,
            "r_z": r_z, "r_z_nokta": r_z_nokta,
            "cmd_real_aci": _aci_deg(v_cmd, v_ned),
            "cmd_real_dv": float(np.linalg.norm(v_cmd-v_ned)),
        }
        return Komut(vel_ned=v_cmd,
                     yaw_rate_dps=(yaw_rate if self.yaw_komutu_ver else None),
                     olay=olay, olay_detay=detay)


def arg_ayristirici():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sure", type=float, default=None)
    p.add_argument("--loop-hz", type=float, default=20.0)
    p.add_argument("--tau", type=float, default=0.20,
                   help="ortak komut LPF zaman sabiti [s]")
    p.add_argument("--log", default=None)
    p.add_argument("--n-pn", type=float, default=4.0)
    p.add_argument("--vur-ivme", type=float, default=4.0)
    p.add_argument("--komut-ufku", type=float, default=0.70)
    p.add_argument("--terminal-menzil", type=float, default=3.0)
    p.add_argument("--terminal-tgo", type=float, default=0.25)
    p.add_argument("--v-max", type=float, default=35.0)
    p.add_argument("--no-yaw", action="store_true",
                   help="yaw-rate gonderme; yanal LOS komutu yine hiz "
                        "setpointiyle roll/pitch uretir")
    return p


def main():
    a = arg_ayristirici().parse_args()
    k = TerminalLosKontrolcu(n_pn=a.n_pn, vur_ivme_mps2=a.vur_ivme,
                             komut_ufku_s=a.komut_ufku,
                             terminal_menzil_m=a.terminal_menzil,
                             terminal_tgo_s=a.terminal_tgo,
                             yaw_komutu_ver=not a.no_yaw,
                             v_max_mps=a.v_max)
    print("[terminal_los] YERLES/VUR/DON "
          f"N={k.n_pn:.1f} a_vur={k.vur_ivme:.1f}m/s2 "
          f"erisilebilirlik={k.a_yatay_max:.1f}m/s2 x {k.komut_ufku:.2f}s "
          f"yon_konisi=+/-{k.komut_aci_max:.0f}deg "
          f"terminal={k.terminal_menzil:.1f}m/{k.terminal_tgo:.2f}s "
          f"yaw={'acik' if k.yaw_komutu_ver else 'otopilotta'}",
          flush=True)
    GoruntuluDongu(k, loop_hz=a.loop_hz, tau_s=a.tau,
                   log_yolu=a.log).calistir(a.sure)


if __name__ == "__main__":
    main()
