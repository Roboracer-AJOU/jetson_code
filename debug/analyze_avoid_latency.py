#!/usr/bin/env python3
"""계측 CSV 에서 "한 박자" 가 어디서 새는지 뽑는다.

장애물 접근 한 번을 하나의 **조우** 로 묶고, 체인의 각 단계가 장애물까지
몇 m 남았을 때 반응했는지 나란히 놓는다. 시간이 아니라 거리로 봐야
"늦었다" 가 판단된다 — 같은 0.2 초라도 6 m/s 면 1.2 m 다.

    python3 debug/analyze_avoid_latency.py debug/lap.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

SEEN_M = 12.0        # 이 안에 들어오면 "보였다"
MIN_FRAMES = 8       # 스쳐 지나가는 오검 제외 (~0.16 s)
GAP_S = 1.0          # 이만큼 끊기면 다른 조우
STEER_DELTA = 5.0    # 조향이 기준선에서 이만큼 벌어지면 "움직였다" [deg]


def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def load(p: Path):
    with p.open() as fh:
        return [r for r in csv.DictReader(fh)]


def encounters(rows):
    """장애물이 연속으로 보인 구간들."""
    out, cur, last_t = [], [], None
    for r in rows:
        d, t = _f(r["obs_d"]), _f(r["t"])
        if d is not None and d <= SEEN_M:
            if last_t is not None and t - last_t > GAP_S and cur:
                out.append(cur)
                cur = []
            cur.append(r)
            last_t = t
        elif cur and last_t is not None and t - last_t > GAP_S:
            out.append(cur)
            cur, last_t = [], None
    if cur:
        out.append(cur)
    return [e for e in out if len(e) >= MIN_FRAMES]


def first(rows, pred):
    for r in rows:
        if pred(r):
            return r
    return None


def d_of(r):
    return _f(r["obs_d"]) if r else None


def describe(e, idx, base_steer):
    d0 = _f(e[0]["obs_d"])
    dmin = min(x for x in (_f(r["obs_d"]) for r in e) if x is not None)
    vmax = max(_f(r["v"]) or 0.0 for r in e)

    seen = e[0]
    slow = first(e, lambda r: (_f(r["scale"]) or 1.0) < 0.95)
    raw = _f(e[0].get("raw_d") or "")
    mode = first(e, lambda r: r["mode"] in ("AVOID", "REJOIN"))
    ovr = first(e, lambda r: r["override"] == "1")
    fgm = first(e, lambda r: r["fgm_en"] == "1")
    path = first(e, lambda r: (int(r["path_n"]) if r["path_n"] else 0) > 0)
    # 조향 기준선은 **모드가 바뀐 그 순간의 조향** 이어야 한다. 주행 전체의
    # 중앙값을 쓰면 코너 조향이 섞여서, 회피와 무관한 값과 비교하게 된다.
    mark = mode or e[0]
    base = _f(mark["steer"]) or 0.0
    after = [r for r in e if _f(r["t"]) >= _f(mark["t"])]
    steer = first(after, lambda r: abs((_f(r["steer"]) or 0.0) - base) > STEER_DELTA)
    aeb = first(e, lambda r: r["aeb"] == "1")

    far = max(
        (x for x in (_f(r.get("raw_far") or "") for r in e) if x is not None),
        default=None,
    )
    print(f"\n  ── 조우 {idx}  (t={_f(e[0]['t']):.1f}s, 최고 {vmax:.1f} m/s, "
          f"최근접 {dmin:.2f} m"
          + (f", 이때 최원거리 검출 {far:.1f} m" if far else "") + ")")
    if raw is not None:
        print(f"     원시 검출은 이미 {raw:.2f} m 에 있었다 "
              f"(게이트 통과는 {d0:.2f} m)")
    print(f"     {'단계':<22} {'남은거리':>8}  {'시각':>7}  {'속도':>6}")
    stages = [
        ("검출 (/static_obstacles)", seen),
        ("감속 시작 (scale<0.95)", slow),
        ("모드 AVOID/REJOIN", mode),
        ("경로 override 켜짐", ovr),
        ("FGM enable", fgm),
        ("/local_path 발행", path),
        ("조향 실제로 움직임", steer),
        ("AEB", aeb),
    ]
    for name, r in stages:
        if r is None:
            print(f"     {name:<22} {'—':>8}")
            continue
        dd = d_of(r)
        ds = f"{dd:.2f} m" if dd is not None else "-"
        print(f"     {name:<22} {ds:>8}  {_f(r['t']):6.2f}s  "
              f"{_f(r['v']):5.2f}")

    if aeb is not None and (mode is None or _f(aeb["t"]) < _f(mode["t"])):
        print("     ** AEB 가 회피보다 먼저다 — 회피가 아예 안 걸렸다 **")
    if mode is not None and path is not None:
        lag = _f(path["t"]) - _f(mode["t"])
        gap = (d_of(mode) or 0) - (d_of(path) or 0)
        print(f"     모드 → 경로 발행까지 {lag*1000:5.0f} ms ({gap:.2f} m 진행)")
    if mode is not None and steer is not None:
        lag = _f(steer["t"]) - _f(mode["t"])
        gap = (d_of(mode) or 0) - (d_of(steer) or 0)
        print(f"     모드 → 조향 {STEER_DELTA:.0f}° 변화까지 {lag*1000:5.0f} ms "
              f"({gap:.2f} m 진행, {base:+.1f}° → {_f(steer['steer']):+.1f}°)")
    if seen is not None and mode is not None:
        lag = _f(mode["t"]) - _f(seen["t"])
        gap = (d_of(seen) or 0) - (d_of(mode) or 0)
        print(f"     검출 → 모드 전환까지 {lag*1000:5.0f} ms ({gap:.2f} m 진행)")
    return {
        "seen": d_of(seen), "slow": d_of(slow), "mode": d_of(mode),
        "path": d_of(path), "steer": d_of(steer), "aeb": d_of(aeb),
        "vmax": vmax, "dmin": dmin,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    a = ap.parse_args()
    rows = load(a.csv)
    if not rows:
        print("빈 파일")
        return

    dur = _f(rows[-1]["t"]) or 0.0
    moving = [r for r in rows if (_f(r["v"]) or 0) > 0.5]
    print(f"기록 {dur:.0f}s, {len(rows)} 행, 주행 {len(moving)/50:.0f}s")

    # 파이프라인 신선도 — 여기가 느리면 전부 늦는다
    sa = sorted(x for x in (_f(r["scan_age"]) for r in rows) if x is not None)
    oa = sorted(x for x in (_f(r["obs_age"]) for r in rows) if x is not None)
    if sa:
        print(f"\n파이프라인 지연 (scan 헤더 → 지금)")
        print(f"  중앙 {sa[len(sa)//2]*1000:.0f} ms, "
              f"95% {sa[int(len(sa)*.95)]*1000:.0f} ms, "
              f"최대 {sa[-1]*1000:.0f} ms")
    if oa:
        print(f"장애물 갱신 간격")
        print(f"  중앙 {oa[len(oa)//2]*1000:.0f} ms, "
              f"95% {oa[int(len(oa)*.95)]*1000:.0f} ms, "
              f"최대 {oa[-1]*1000:.0f} ms")

    steers = [
        _f(r["steer"]) for r in rows
        if r["mode"] == "GLOBAL" and _f(r["steer"]) is not None
    ]
    base = sorted(steers)[len(steers)//2] if steers else 0.0

    es = encounters(rows)
    print(f"\n장애물 조우 {len(es)} 회")
    got = [describe(e, i + 1, base) for i, e in enumerate(es)]

    real = [g for g in got if g["mode"] is not None]
    late = [g for g in got if g["aeb"] is not None and g["mode"] is None]
    print("\n" + "=" * 60)
    print(f"회피가 걸린 조우 {len(real)}, AEB 만 걸린 조우 {len(late)}")
    if real:
        for k, label in (("seen", "검출"), ("slow", "감속"),
                         ("mode", "모드"), ("path", "경로"),
                         ("steer", "조향")):
            v = [g[k] for g in real if g[k] is not None]
            if v:
                print(f"  {label} 평균 남은거리 {sum(v)/len(v):5.2f} m  "
                      f"(최소 {min(v):.2f})")


if __name__ == "__main__":
    main()
