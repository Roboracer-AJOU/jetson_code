#!/usr/bin/env python3
"""
centerline.csv + map.yaml → raceline.csv (맵 무관 파이프라인).

기본 방식은 **최소곡률 최적화**(minimum curvature). 코너 속도는
v_max = sqrt(a_lat / κ) 이므로 경로 곡률 제곱합을 줄이면 랩타임이 줄어든다.

  1. 센터라인 로드 → 픽셀 좌표 → 등간격 리샘플
  2. 각 점의 법선으로 좌/우 벽까지 거리 측정 → 횡오프셋 상·하한
  3. 라인을 x_i = p_i + α_i·n_i 로 두고
       min Σ‖x_{i-1} − 2x_i + x_{i+1}‖²   s.t.  lo_i ≤ α_i ≤ hi_i
     를 푼다. α 에 대해 볼록 QP 라 능동집합법으로 전역해를 구한다.
  4. 새 라인을 기준선으로 삼아 법선·경계를 다시 잡고 재수렴 (기본 2회)
  5. 최소 회전반경 보정 + 클리어런스 검증

`--method oio` 로 하면 기존 휴리스틱 Out-In-Out 을 쓴다. 이 트랙에서는
최소곡률이 O-I-O 보다 30%, 센터라인보다 21% 빨랐다.

사용:
  python3 generate_raceline_from_centerline.py
  python3 generate_raceline_from_centerline.py --margin-m 0.45   # 더 안전하게
  python3 generate_raceline_from_centerline.py --method oio
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from extract_centerline_from_map import (  # noqa: E402
    CFG as CENTERLINE_CFG,
    bilinear_sample,
    closed_normals,
    count_self_intersections,
    count_wall_crossings,
    load_map,
    path_length,
    pixel_to_world,
    relax_curvature,
    resample_closed,
    resolve_map_yaml,
    turn_angles_deg,
    world_to_pixel,
    write_csv,
)
from speed_profile import (  # noqa: E402
    UNMEASURED_WARNING,
    VEHICLE,
    add_speed_args,
    lap_time,
    path_curvature,
    profile_kwargs_from_args,
    report_profile,
    speed_profile,
    write_csv_xyv,
)

try:
    import scipy.sparse as sp
    import scipy.sparse.linalg as spl
    from scipy.ndimage import distance_transform_edt, gaussian_filter1d
except ImportError as exc:  # pragma: no cover
    print("Missing scipy:", exc, file=sys.stderr)
    sys.exit(1)


# ============================================================
# USER TUNING — 맵 바꿀 때 여기만 수정
# ============================================================
CFG = {
    "map_name": "cartographer_map_20260816_004549.yaml",
    "map_dir": CENTERLINE_CFG["map_dir"],
    "centerline_csv": os.path.join(script_dir, "..", "config", "centerline.csv"),
    "out_csv": os.path.join(script_dir, "..", "config", "raceline.csv"),
}



def load_centerline_csv(path: str):
    points = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except ValueError:
                continue  # 헤더
    return points


# ============================================================
# 트랙 폭 (횡오프셋 상·하한)
# ============================================================
def measure_track_widths(points, normals, free, margin_px: float, max_px: float = 200.0):
    """각 점에서 법선(+왼쪽/−오른쪽)으로 벽까지 거리 [px].

    반환: (lo, hi) — 허용 횡오프셋. hi>0=왼쪽 여유, lo<0=오른쪽 여유.
    margin_px 만큼은 벽에서 떼어 놓는다 (차량 반폭 + 안전여유).
    """
    h, w = free.shape
    lo = np.zeros(len(points))
    hi = np.zeros(len(points))
    for i, (base, nrm) in enumerate(zip(points, normals)):
        for sign in (1.0, -1.0):
            reach = 0.0
            t = 0.0
            while t < max_px:
                t += 0.5
                rr = int(round(base[0] + sign * t * nrm[0]))
                cc = int(round(base[1] + sign * t * nrm[1]))
                if not (0 <= rr < h and 0 <= cc < w) or free[rr, cc] == 0:
                    break
                reach = t
            usable = max(0.0, reach - margin_px)
            if sign > 0:
                hi[i] = usable
            else:
                lo[i] = -usable
    return lo, hi


# ============================================================
# 최소곡률 최적화
# ============================================================
def _circulant(n: int, stencil):
    """순환 밴드 행렬. stencil = [(offset, value), ...]"""
    idx = np.arange(n)
    rows, cols, vals = [], [], []
    for offset, value in stencil:
        rows.append(idx)
        cols.append((idx + offset) % n)
        vals.append(np.full(n, float(value)))
    return sp.csr_matrix(
        (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
        shape=(n, n),
    )


def solve_box_qp(G, c, lo, hi, max_iters: int = 40):
    """min ½·aᵀGa + cᵀa  s.t. lo ≤ a ≤ hi.

    G 가 대칭 준정부호라 볼록. 능동집합(projected Newton) + 백트래킹으로
    전역 최적해를 찾는다. G 는 5중대각+모서리라 희소 분해가 매우 빠르다.
    """
    n = G.shape[0]
    G_csc = G.tocsc()
    a = np.clip(np.zeros(n), lo, hi)

    def cost(x):
        return 0.5 * float(x @ (G @ x)) + float(c @ x)

    current = cost(a)
    for _ in range(max_iters):
        grad = G @ a + c
        # 경계에 붙어 있고 밖으로 나가려는 변수는 고정
        fixed = ((a <= lo + 1e-9) & (grad > 0)) | ((a >= hi - 1e-9) & (grad < 0))
        freed = ~fixed
        if not freed.any():
                break
        idx = np.where(freed)[0]
        rhs = -c[idx]
        if fixed.any():
            rhs = rhs - G_csc[idx][:, fixed] @ a[fixed]
        try:
            newton = spl.spsolve(G_csc[idx][:, idx].tocsc(), rhs)
        except Exception:
                break
        step = np.zeros(n)
        step[idx] = newton - a[idx]

        scale = 1.0
        improved = False
        for _ in range(30):
            trial = np.clip(a + scale * step, lo, hi)
            trial_cost = cost(trial)
            if trial_cost < current - 1e-14:
                a, current, improved = trial, trial_cost, True
                break
            scale *= 0.5
        if not improved or np.max(np.abs(step)) * scale < 1e-9:
                break
    return a


def minimum_curvature_offsets(reference, normals, lo, hi, w_length: float = 0.0):
    """기준선 위에서 곡률 제곱합을 최소화하는 횡오프셋 α.

    x_i = p_i + α_i·n_i 로 두면 2차차분이 α 의 아핀함수라 QP 가 된다.
    w_length > 0 이면 경로 길이(1차차분) 항을 섞어 살짝 최단경로 쪽으로 당긴다.
    """
    n = len(reference)
    D2 = _circulant(n, [(-1, 1.0), (0, -2.0), (1, 1.0)])
    curv_const = D2 @ reference
    A_row = D2 @ sp.diags(normals[:, 0])
    A_col = D2 @ sp.diags(normals[:, 1])

    G = 2.0 * ((A_row.T @ A_row) + (A_col.T @ A_col))
    c = 2.0 * (A_row.T @ curv_const[:, 0] + A_col.T @ curv_const[:, 1])

    if w_length > 0.0:
        D1 = _circulant(n, [(0, -1.0), (1, 1.0)])
        len_const = D1 @ reference
        B_row = D1 @ sp.diags(normals[:, 0])
        B_col = D1 @ sp.diags(normals[:, 1])
        G = G + 2.0 * w_length * ((B_row.T @ B_row) + (B_col.T @ B_col))
        c = c + 2.0 * w_length * (
            B_row.T @ len_const[:, 0] + B_col.T @ len_const[:, 1]
        )

    return solve_box_qp(G.tocsr(), np.asarray(c).ravel(), lo, hi)


def minimum_curvature_line(
    center,
    free,
    *,
    step_px: float,
    margin_px: float,
    iterations: int,
    w_length: float,
    verbose: bool = True,
):
    """최소곡률 라인. 매 반복마다 직전 해를 기준선으로 다시 선형화한다."""
    line = np.asarray(center, dtype=float).copy()
    for it in range(max(1, iterations)):
        normals = closed_normals(line)
        lo, hi = measure_track_widths(line, normals, free, margin_px)
        alpha = minimum_curvature_offsets(line, normals, lo, hi, w_length)
        line = resample_closed(line + alpha[:, None] * normals, step_px)
        if verbose:
            print(
                f"    mincurv iter {it + 1}/{iterations}: "
                f"|α|max={np.abs(alpha).max():.1f} px, "
                f"turn p99={np.percentile(turn_angles_deg(line), 99):.2f}°"
            )
    return line


# ============================================================
# Out-In-Out 휴리스틱 (--method oio)
# ============================================================
def heading_change(points, lookahead: int) -> np.ndarray:
    """각 점의 Δψ [rad]. 양수=왼쪽(법선 +방향)으로 휨."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < 2 * lookahead + 1:
        return np.zeros(n)
    prev = np.roll(pts, lookahead, axis=0)
    nxt = np.roll(pts, -lookahead, axis=0)
    v1 = pts - prev
    v2 = nxt - pts
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    dot = v1[:, 0] * v2[:, 0] + v1[:, 1] * v2[:, 1]
    return np.arctan2(-cross, dot)


def detect_corners(dpsi, threshold: float, min_len: int, merge_gap: int):
    """|Δψ|>threshold 인 구간을 코너로 묶는다. 반환: [(start, end, 'left'|'right')]."""
    n = len(dpsi)
    raw = []
    i = 0
    while i < n:
        if abs(dpsi[i]) <= threshold:
            i += 1
            continue
        side = "left" if dpsi[i] > 0 else "right"
        j = i
        while j < n and abs(dpsi[j]) > threshold and (
            ("left" if dpsi[j] > 0 else "right") == side
        ):
            j += 1
        if j - i >= min_len:
            raw.append((i, j, side))
        i = max(j, i + 1)

    if not raw:
        return []
    merged = [list(raw[0])]
    for start, end, side in raw[1:]:
        last = merged[-1]
        if side == last[2] and start - last[1] <= merge_gap:
            last[1] = end
        else:
            merged.append([start, end, side])
    return [(int(s), int(e), side) for s, e, side in merged]


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def build_offset_profile(
    n: int,
    lo,
    hi,
    corners,
    *,
    alpha_out: float,
    beta_in: float,
    entry_pts: int,
    exit_pts: int,
    apex_fraction: float,
):
    """코너별 Out–In–Out 목표 횡오프셋 [px]. 직선 구간은 0(센터)."""
    d = np.zeros(n)
    for i_start, i_end, side in corners:
        length = max(1, i_end - i_start)
        apex = i_start + int(apex_fraction * length)
        for k in range(i_start - entry_pts, i_end + exit_pts):
            i = k % n
            low, high = lo[i], hi[i]
            if side == "left":
                d_out = -alpha_out * abs(low)   # 왼쪽 코너 → 바깥은 오른쪽
                d_in = beta_in * high
            else:
                d_out = alpha_out * high
                d_in = -beta_in * abs(low)
            d_out = float(np.clip(d_out, low, high))
            d_in = float(np.clip(d_in, low, high))

            if k < i_start:
                value = _smoothstep((k - (i_start - entry_pts)) / entry_pts) * d_out
            elif k <= apex:
                frac = (k - i_start) / max(1, apex - i_start)
                value = d_out + 0.5 * (1.0 - np.cos(np.pi * frac)) * (d_in - d_out)
            elif k < i_end:
                frac = (k - apex) / max(1, i_end - 1 - apex)
                value = d_in + 0.5 * (1.0 - np.cos(np.pi * frac)) * (d_out - d_in)
            else:
                value = (1.0 - _smoothstep((k - i_end) / exit_pts)) * d_out

            if abs(value) > abs(d[i]):
                d[i] = float(np.clip(value, low, high))
    return d


def smooth_and_clamp(d, lo, hi, window: int):
    if window > 0:
        kernel = np.ones(2 * window + 1) / (2 * window + 1)
        padded = np.concatenate([d[-window:], d, d[:window]])
        d = np.convolve(padded, kernel, mode="valid")
    return np.clip(d, lo, hi)


def smooth_toward_target(
    target,
    free,
    *,
    step_px: float,
    min_clear_px: float,
    iters: int = 300,
    w_smooth: float = 0.30,
    w_target: float = 0.10,
):
    """목표선 근처를 유지하면서 곡률을 낮춘다. 벽에 가까워지는 이동은 버린다."""
    dist = distance_transform_edt(free > 0).astype(float)
    goal = resample_closed(target, step_px)
    pts = goal.copy()
    for _ in range(max(1, iters)):
        prev = np.roll(pts, 1, axis=0)
        nxt = np.roll(pts, -1, axis=0)
        cand = pts + w_smooth * (0.5 * (prev + nxt) - pts) + w_target * (goal - pts)
        ok = bilinear_sample(dist, cand[:, 0], cand[:, 1]) >= min_clear_px
        pts = np.where(ok[:, None], cand, pts)
    return resample_closed(pts, step_px)


# ============================================================
# 평가: 곡률 → 속도 프로파일 → 랩타임
# ============================================================
# ============================================================
# main
# ============================================================
def main() -> int:
    parser = argparse.ArgumentParser(
        description="centerline.csv → raceline.csv (최소곡률 최적화 / Out-In-Out)"
    )
    parser.add_argument(
        "--centerline", default=os.path.abspath(CFG["centerline_csv"]), help="입력 CSV"
    )
    parser.add_argument(
        "--map",
        default=resolve_map_yaml(CFG["map_name"], CFG["map_dir"]),
        help="map.yaml 경로",
    )
    parser.add_argument("--out", default=os.path.abspath(CFG["out_csv"]), help="출력 CSV")
    parser.add_argument(
        "--method",
        choices=("mincurv", "oio"),
        default="mincurv",
        help="mincurv=최소곡률 최적화(기본), oio=휴리스틱 Out-In-Out",
    )
    parser.add_argument(
        "--invert-free", action="store_true", help="어두운 픽셀을 도로로 해석"
    )
    parser.add_argument("--resample-step-m", type=float, default=0.05, help="점 간격 [m]")
    parser.add_argument(
        "--margin-m",
        type=float,
        default=0.35,
        help="벽에서 떼어 놓을 거리 [m] = 차량 반폭 + 안전여유. 최적해는 이 경계까지 붙는다",
    )
    parser.add_argument(
        "--min-clear-m", type=float, default=0.30, help="최종 검증용 최소 벽 거리 [m]"
    )
    parser.add_argument(
        "--min-radius-m",
        type=float,
        default=0.9,
        help="추종 가능한 최소 회전반경 [m]. 이보다 급한 코너만 국소적으로 편다. 0=비활성",
    )
    # --- mincurv ---
    parser.add_argument(
        "--iterations", type=int, default=2, help="[mincurv] 재선형화 반복"
    )
    parser.add_argument(
        "--w-length",
        type=float,
        default=0.0,
        help="[mincurv] 경로 길이 가중치. >0 이면 최단경로 쪽으로 살짝 당긴다",
    )
    # --- oio ---
    parser.add_argument("--lookahead-m", type=float, default=0.75, help="[oio] 코너 판정 lookahead [m]")
    parser.add_argument("--corner-thresh-deg", type=float, default=7.0, help="[oio] 코너 판정 |Δψ|")
    parser.add_argument("--min-corner-m", type=float, default=0.3, help="[oio] 코너 최소 길이 [m]")
    parser.add_argument("--merge-gap-m", type=float, default=2.5, help="[oio] 코너 병합 거리 [m]")
    parser.add_argument("--entry-m", type=float, default=1.8, help="[oio] 진입 전환 길이 [m]")
    parser.add_argument("--exit-m", type=float, default=1.8, help="[oio] 탈출 전환 길이 [m]")
    parser.add_argument("--alpha-out", type=float, default=0.50, help="[oio] 바깥 오프셋 비율")
    parser.add_argument("--beta-in", type=float, default=0.60, help="[oio] apex 오프셋 비율")
    parser.add_argument("--apex-fraction", type=float, default=0.60, help="[oio] delayed apex 비율")
    parser.add_argument("--d-smooth-m", type=float, default=1.2, help="[oio] 오프셋 스무딩 반창 [m]")
    parser.add_argument("--smooth-iters", type=int, default=300, help="[oio] 최종 스무딩 반복")
    # --- 속도 프로파일 (CSV 3번째 열). 기본값은 speed_profile.VEHICLE ---
    add_speed_args(parser)
    args = parser.parse_args()

    if not os.path.isfile(args.centerline):
        print(f"Centerline not found: {args.centerline}", file=sys.stderr)
        return 1
    if not os.path.isfile(args.map):
        print(f"Map not found: {args.map}", file=sys.stderr)
        return 1

    points_xy = load_centerline_csv(args.centerline)
    if len(points_xy) < 16:
        print("Centerline has too few points.", file=sys.stderr)
        return 1

    free, resolution, ox, oy, (height, width) = load_map(
        args.map, invert_free=args.invert_free
    )
    print(f"Centerline: {args.centerline} ({len(points_xy)} pts)")
    print(f"Map: {args.map} ({height}x{width}, res={resolution})")

    step_px = max(0.5, args.resample_step_m / resolution)
    per_pt_m = step_px * resolution
    min_clear_px = max(1.0, args.min_clear_m / resolution)
    margin_px = max(0.0, args.margin_m / resolution)

    center = resample_closed(
        [world_to_pixel(x, y, height, resolution, ox, oy) for x, y in points_xy],
        step_px,
    )
    n = len(center)
    lo0, hi0 = measure_track_widths(center, closed_normals(center), free, margin_px)
    width_m = (hi0 - lo0) * resolution
    print(
        f"  usable width (margin {args.margin_m} m 제외): "
        f"median={np.median(width_m):.2f} m min={width_m.min():.2f} m"
    )

    if args.method == "mincurv":
        print(f"  method=mincurv (곡률 제곱합 최소화, {args.iterations} iters)")
        race = minimum_curvature_line(
            center,
            free,
            step_px=step_px,
            margin_px=margin_px,
            iterations=args.iterations,
            w_length=args.w_length,
        )
    else:
        print("  method=oio (휴리스틱 Out-In-Out)")
        normals = closed_normals(center)
        lookahead = max(1, int(args.lookahead_m / per_pt_m))
        corners = detect_corners(
            heading_change(center, lookahead),
            np.radians(args.corner_thresh_deg),
            max(1, int(args.min_corner_m / per_pt_m)),
            max(1, int(args.merge_gap_m / per_pt_m)),
        )
        print(f"  corners: {len(corners)}")
        d = build_offset_profile(
            n,
            lo0,
            hi0,
            corners,
            alpha_out=args.alpha_out,
            beta_in=args.beta_in,
            entry_pts=max(1, int(args.entry_m / per_pt_m)),
            exit_pts=max(1, int(args.exit_m / per_pt_m)),
            apex_fraction=args.apex_fraction,
        )
        d = smooth_and_clamp(d, lo0, hi0, max(1, int(args.d_smooth_m / per_pt_m)))
        race = smooth_toward_target(
            center + d[:, None] * normals,
            free,
            step_px=step_px,
            min_clear_px=min_clear_px,
            iters=args.smooth_iters,
        )

    if args.min_radius_m > 0.0:
        race = relax_curvature(
            race,
            free,
            step_px=step_px,
            min_clear_px=min_clear_px,
            min_radius_px=args.min_radius_m / resolution,
        )

    dist = distance_transform_edt(free > 0)
    clear_m = bilinear_sample(dist, race[:, 0], race[:, 1]) * resolution
    turns = turn_angles_deg(race)
    wall_x = count_wall_crossings(race, free)
    self_ix = count_self_intersections(race)

    profile_kwargs = profile_kwargs_from_args(args, race, resolution)
    v_race, kappa, ds_race = speed_profile(race, resolution, **profile_kwargs)
    v_center, _, ds_center = speed_profile(center, resolution, **profile_kwargs)
    lap_race = lap_time(v_race, ds_race)
    lap_center = lap_time(v_center, ds_center)

    print(
        f"  raceline: {len(race)} pts, length={path_length(race) * resolution:.2f} m "
        f"(centerline {path_length(center) * resolution:.2f} m)"
    )
    print(
        f"  curvature max={kappa.max():.3f} 1/m (R_min={1.0 / max(kappa.max(), 1e-9):.2f} m), "
        f"turn |Δθ| max={turns.max():.2f}° p99={np.percentile(turns, 99):.2f}°"
    )
    print(
        f"  clearance min={clear_m.min():.3f} m median={np.median(clear_m):.3f} m"
    )
    report_profile(v_race, args, profile_kwargs["scale"])
    print(
        f"  est. lap={lap_race:.2f} s (centerline {lap_center:.2f} s, "
        f"{100.0 * (lap_center - lap_race) / lap_center:+.1f}%)"
    )
    print(f"  wall_crossings={wall_x}, self_intersections={self_ix}")
    if wall_x or self_ix:
        print("WARNING: 레이스라인이 벽을 지나거나 자기교차합니다.", file=sys.stderr)
        return 1
    if clear_m.min() < args.min_clear_m - 1e-6:
        print(
            f"WARNING: 최소 클리어런스 {clear_m.min():.3f} m < {args.min_clear_m} m. "
            "--margin-m 을 키우세요.",
            file=sys.stderr,
        )

    world = [pixel_to_world(r, c, height, resolution, ox, oy) for r, c in race]
    out_path = os.path.abspath(args.out)
    if args.speed:
        write_csv_xyv(out_path, [(x, y, v) for (x, y), v in zip(world, v_race)])
        print(f"Wrote {len(world)} points (x,y,v) → {out_path}")
    else:
        write_csv(out_path, world)
        print(f"Wrote {len(world)} points (x,y) → {out_path}")

    if args.speed and not VEHICLE["measured"]:
        print(UNMEASURED_WARNING, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
