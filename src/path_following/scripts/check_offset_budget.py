#!/usr/bin/env python3
"""실측 맵에서 회피 기동이 벽 안으로 들어가는 지점이 있는지 전수 검사.

    python3 src/path_following/scripts/check_offset_budget.py

레이스라인의 **모든** 점에 정면 장애물을 하나씩 놓아 보고, 거기서 계획되는
기동이 실제로 트랙 안에 들어가는지 본다. 단위 테스트는 계획기 로직을 보고,
이건 이 트랙에서 실제로 통하는지를 본다.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
    plan_maneuver,
)

ROOT = Path(__file__).resolve().parents[3]
BUDGET_STEP_M = 0.025


def load_map(path: Path):
    meta = yaml.safe_load(path.read_text())
    img = np.array(Image.open(path.parent / Path(meta["image"]).name))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occupied = (img < 100) | (np.abs(img.astype(int) - 205) < 30)
    occupied = np.flipud(occupied)
    clearance = distance_transform_edt(~occupied) * res
    return clearance, res, ox, oy


def make_blocked(clearance, res, ox, oy, inflation):
    h, w = clearance.shape

    def blocked(xs, ys):
        j = ((np.asarray(xs) - ox) / res).astype(np.int64)
        i = ((np.asarray(ys) - oy) / res).astype(np.int64)
        inside = (i >= 0) & (j >= 0) & (i < h) & (j < w)
        c = np.zeros(np.shape(xs), dtype=np.float64)
        c[inside] = clearance[i[inside], j[inside]]
        return ~inside | (c < inflation)

    return blocked


def side_budgets(xs, ys, blocked, cap):
    tx = np.roll(xs, -1) - np.roll(xs, 1)
    ty = np.roll(ys, -1) - np.roll(ys, 1)
    norm = np.hypot(tx, ty)
    norm[norm < 1e-9] = 1.0
    nx, ny = -ty / norm, tx / norm

    def one(sign):
        budget = np.zeros(len(xs))
        alive = np.ones(len(xs), dtype=bool)
        t = BUDGET_STEP_M
        while t <= cap + 1e-9:
            alive &= ~blocked(xs + sign * nx * t, ys + sign * ny * t)
            if not alive.any():
                break
            budget[alive] = t
            t += BUDGET_STEP_M
        return budget

    return one(+1.0), one(-1.0), nx, ny


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=None)
    ap.add_argument(
        "--raceline", default=str(ROOT / "src/path_following/config/raceline.csv")
    )
    ap.add_argument("--inflation", type=float, default=0.254)
    ap.add_argument("--speed", type=float, default=6.0)
    ap.add_argument("--obstacle-r", type=float, default=0.18)
    ap.add_argument("--detect-m", type=float, default=9.0)
    ap.add_argument("--max-offset", type=float, default=0.70)
    ap.add_argument("--step", type=float, default=0.10)
    ap.add_argument("--v-floor", type=float, default=2.0)
    ap.add_argument("--v-step", type=float, default=0.5)
    args = ap.parse_args()

    map_path = (
        Path(args.map)
        if args.map
        else sorted((ROOT / "maps").glob("*_rosmap.yaml"))[-1]
    )
    clearance, res, ox, oy = load_map(map_path)
    blocked = make_blocked(clearance, res, ox, oy, args.inflation)
    print(f"map      : {map_path.name}  res={res}  inflation={args.inflation}")

    rl = np.loadtxt(args.raceline, delimiter=",", skiprows=1)
    xs, ys = rl[:, 0], rl[:, 1]
    n = len(xs)
    seg = np.hypot(np.diff(xs, append=xs[0]), np.diff(ys, append=ys[0]))
    s_of = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
    total = float(seg.sum())
    print(f"raceline : {n} pts, {total:.1f} m")

    bl, br, nx, ny = side_budgets(xs, ys, blocked, args.max_offset)
    best = np.maximum(bl, br)
    print(
        f"예산     : 좌 min {bl.min():.2f} med {np.median(bl):.2f} | "
        f"우 min {br.min():.2f} med {np.median(br):.2f} | "
        f"좋은쪽 min {best.min():.2f} med {np.median(best):.2f}"
    )

    cfg = ManeuverConfig(
        half_width_m=vg.HALF_WIDTH_M,
        lateral_margin_m=0.25,
        max_offset_m=args.max_offset,
        a_lat_enter_mps2=3.0,
        a_lat_exit_mps2=2.0,
        a_lat_hard_mps2=4.5,
        enter_min_m=1.0,
        enter_max_m=0.18 * total,
        exit_min_m=1.5,
        exit_max_m=0.18 * total,
        hold_front_m=vg.FRONT_M + 0.20,
        hold_rear_m=vg.LENGTH_M + 0.30,
        merge_gap_m=3.0,
        v_plan_min_mps=1.5,
        max_steer_rad=0.3735,
        wheelbase_m=vg.WHEELBASE_M,
    )

    def idx_at(s):
        return int((s % total) / total * n) % n

    def budget_over(s0, s1):
        i0, i1 = idx_at(s0), idx_at(s1)
        if (s1 - s0) >= total:
            return bl.min(), br.min()
        if i0 <= i1:
            return bl[i0 : i1 + 1].min(), br[i0 : i1 + 1].min()
        return (
            min(bl[i0:].min(), bl[: i1 + 1].min()),
            min(br[i0:].min(), br[: i1 + 1].min()),
        )

    def xy_at(s, d):
        i = idx_at(s)
        return xs[i] + nx[i] * d, ys[i] + ny[i] * d

    def path_clear(m, s0):
        """계획된 d(s) 를 실제 맵에 찍어 본다 — 노드의 `_path_fully_clear`."""
        ds = 0.0
        while ds <= m.total_length_m:
            px, py = xy_at(s0 + ds, m.d_at(ds))
            if bool(blocked(np.array([px]), np.array([py]))[0]):
                return False
            ds += args.step
        return True

    def speeds(v0):
        """계획을 시도할 속도들. 길이가 v 에 선형이라 절반 속도면 절반 길이."""
        out = [v0]
        v = v0
        while v > args.v_floor + 1e-9:
            v = max(args.v_floor, v - args.v_step)
            out.append(v)
        return out

    first_ok = retry_ok = fell_back = refused = leaked = 0
    slowed: list[float] = []
    for i in range(n):
        s0 = s_of[i]
        obs = [ObstacleSD(s=args.detect_m, d=0.0, r=args.obstacle_r)]
        # 노드와 같은 창: 오프셋을 물고 있을 구간만
        ml, mr = budget_over(
            s0 + args.detect_m - cfg.hold_front_m,
            s0 + args.detect_m + cfg.hold_rear_m,
        )

        # 노드의 재시도 루프: 속도를 낮춰 가며 계획 → 점검사 → 방향 바꿔 다시
        outcome = "fallback"
        got_v = None
        for v_try in speeds(args.speed):
            forbid = 0
            for attempt in (1, 2):
                m = plan_maneuver(
                    obs,
                    cfg,
                    d_ego=0.0,
                    d_ego_prime=0.0,
                    v=v_try,
                    forbid_side=forbid,
                    max_left=ml,
                    max_right=mr,
                )
                if m is None:
                    break
                if path_clear(m, s0):
                    outcome = "first" if v_try == args.speed else "retry"
                    got_v = v_try
                    break
                forbid = m.side
            if outcome != "fallback":
                break
        else:
            outcome = "fallback"
        if outcome == "fallback" and got_v is None:
            # 어느 속도로도 계획 자체가 안 나오면 트랙이 좁은 것이다
            first = plan_maneuver(
                obs, cfg, d_ego=0.0, d_ego_prime=0.0, v=args.speed,
                max_left=ml, max_right=mr,
            )
            if first is None:
                outcome = "refused"
        if got_v is not None and got_v < args.speed:
            slowed.append(got_v)
        if outcome == "first":
            first_ok += 1
        elif outcome == "retry":
            retry_ok += 1
        elif outcome == "refused":
            refused += 1
        else:
            fell_back += 1

    print()
    print(f"검사 지점 : {n}")
    print(f"  풀속도 성공: {first_ok:4d} ({100 * first_ok / n:3.0f}%)")
    print(f"  감속 후 성공: {retry_ok:4d} ({100 * retry_ok / n:3.0f}%)")
    print(f"  계획 거부  : {refused:4d} ({100 * refused / n:3.0f}%)  → 감속/TRAILING")
    print(f"  FGM 폴백   : {fell_back:4d} ({100 * fell_back / n:3.0f}%)")
    ok = first_ok + retry_ok
    print(f"  => 기동으로 처리 {100 * ok / n:.0f}%, 벽으로 나간 계획 {leaked}")
    if slowed:
        arr = np.array(slowed)
        print(
            f"     감속한 경우 목표속도 median {np.median(arr):.1f} "
            f"min {arr.min():.1f} m/s"
        )
    return 0 if leaked == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
