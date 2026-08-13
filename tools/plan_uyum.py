#!/usr/bin/env python3
"""ARACA YUKLU GOREV ile ISTENEN PLAN uyusuyor mu? (senaryo.sh on kapisi)

NEDEN VAR (2026-08-09, IKI DENEME SESSIZCE YANDI):
tools/senaryo.sh'de PLAN degiskeni YALNIZCA "YENIDEN_BASLAT == 1" dalinda
YILDIZ_TARGET_PLAN olarak yigina geciyor. YENIDEN_BASLAT=0 ile kosunca
(GUI'yi korumak icin standart kuralimiz) PLAN HICBIR SEY YAPMIYOR: hedef,
yigin acilirken yuklenen gorevi ucmaya devam ediyor (varsayilan
hedef_elips.plan). Ustelik ekrana ">>> hedef ucak: AUTO, duz rota"
yaziliyordu -- o etiket PLAN_AD'den geliyor, ARACA YUKLENENDEN DEGIL.
Sonuc: "duz regresyon" diye iki deneme (tyawaccduz, kpnduz) aslinda ELIPS
uctu ve biri "duz regresyon GECTI" diye raporlandi.

NASIL KARSILASTIRIR: waypoint SAYISI degil GEOMETRI. Yukleme sirasinda
home item eklenip cikarilabildigi icin sayi guvenilmez; onun yerine
konum kapsami (kuzey-guney ve dogu-bati acikligi) ve DO_JUMP varligi
karsilastirilir. Bu ucu birden tutuyorsa ayni rotadir.

CIKIS KODU: 0 = uyumlu, 1 = UYUMSUZ, 2 = karar verilemedi (arac/plan
okunamadi). senaryo.sh 1'de DURUR, 2'de uyarip devam eder.
"""
import argparse, json, math, sys, time

R_DUNYA = 6371000.0


def kapsam(noktalar):
    """(kuzey-guney [m], dogu-bati [m]) acikligi."""
    la = [p[0] for p in noktalar]
    lo = [p[1] for p in noktalar]
    kg = (max(la) - min(la)) * math.pi / 180.0 * R_DUNYA
    db = ((max(lo) - min(lo)) * math.pi / 180.0 * R_DUNYA
          * math.cos(math.radians(la[0])))
    return kg, db


def plandan(yol):
    it = json.load(open(yol))['mission']['items']
    n = [(i['params'][4], i['params'][5]) for i in it
         if i.get('params') and i['params'][4] not in (None, 0)
         and i['params'][5] not in (None, 0)]
    if not n:
        return None
    kg, db = kapsam(n)
    return dict(kg=kg, db=db, jump=any(i.get('command') == 177 for i in it))


def aractan(baglanti, zaman_asimi=25.0):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(baglanti, source_system=248)
    if m.wait_heartbeat(timeout=zaman_asimi) is None:
        return None
    m.mav.mission_request_list_send(m.target_system, m.target_component)
    t0 = time.time(); adet = None
    while time.time() - t0 < 10:
        x = m.recv_match(type='MISSION_COUNT', blocking=True, timeout=2)
        if x:
            adet = x.count; break
    if not adet:
        return None
    n, jump = [], False
    for seq in range(adet):
        m.mav.mission_request_int_send(m.target_system, m.target_component, seq)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            x = m.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=1)
            if x and x.seq == seq:
                if x.command == 177:
                    jump = True
                if x.x and x.y:
                    n.append((x.x / 1e7, x.y / 1e7))
                break
    if not n:
        return None
    kg, db = kapsam(n)
    return dict(kg=kg, db=db, jump=jump)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', required=True)
    p.add_argument('--baglanti', default='udpin:127.0.0.1:14601')
    p.add_argument('--tolerans', type=float, default=0.15,
                   help='kapsam icin bagil tolerans (varsayilan %%15)')
    a = p.parse_args()

    d = plandan(a.plan)
    if d is None:
        print(f"plan_uyum: PLAN okunamadi ({a.plan})", file=sys.stderr)
        return 2
    v = aractan(a.baglanti)
    if v is None:
        print("plan_uyum: aractan gorev okunamadi (heartbeat/gorev yok)",
              file=sys.stderr)
        return 2

    def yakin(x, y):
        return abs(x - y) <= a.tolerans * max(abs(x), abs(y), 1.0)

    uyum = (yakin(d['kg'], v['kg']) and yakin(d['db'], v['db'])
            and d['jump'] == v['jump'])
    print(f"plan_uyum: DOSYA kapsam K-G {d['kg']:.0f} m / D-B {d['db']:.0f} m "
          f"DO_JUMP {d['jump']}")
    print(f"plan_uyum: ARAC  kapsam K-G {v['kg']:.0f} m / D-B {v['db']:.0f} m "
          f"DO_JUMP {v['jump']}")
    if uyum:
        print("plan_uyum: UYUMLU")
        return 0
    print("plan_uyum: *** UYUMSUZ -- araca YUKLU GOREV istenen PLAN DEGIL ***",
          file=sys.stderr)
    print("plan_uyum: sebep: YENIDEN_BASLAT=0 iken PLAN araca yuklenmez; "
          "yigini YILDIZ_TARGET_PLAN=<plan> ile yeniden baslatin.",
          file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
