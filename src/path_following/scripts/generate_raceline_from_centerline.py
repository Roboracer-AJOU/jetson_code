#!/usr/bin/env python3
"""
centerline.csv + map.yaml → raceline.csv (맵 무관: 어떤 트랙이든 동일 파이프라인으로 이상적 레이싱 라인 추출).

입력: centerline.csv (x,y, 맵 프레임), map.yaml (load_map → free_mask, resolution, origin).
출력: raceline.csv (x,y). 코너는 Out–In–Out, 직선/완만 구간은 센터라인.

특징:
  - 거리/구간은 [m] 또는 resolution 기반으로 환산 → 맵 해상도·크기에 무관.
  - W_pinch, 연속 코너 gap 등은 트랙 폭·길이에 맞게 미터 또는 비율로 적용.
  - 파라미터만 조정하면 좁은 트랙·넓은 트랙·S자·헤어핀 등 모두 대응.

사용 예:
  # CFG map_name 만 고치고 실행
  python3 scripts/generate_raceline_from_centerline.py
  # 또는 CLI
  python3 scripts/generate_raceline_from_centerline.py --centerline <centerline.csv> --map <map.yaml> --out <raceline.csv>
"""
import argparse
import csv
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import numpy as np
except ImportError:
    print("Missing dependency: numpy", file=sys.stderr)
    sys.exit(1)

# 기존 스크립트에서 재사용
from extract_centerline_from_map import (
    CFG as CENTERLINE_CFG,
    load_map,
    pixel_to_world,
    resample_polyline_by_arc_length,
    resolve_map_yaml,
    smooth_polyline,
)


# ============================================================
# USER TUNING — 맵 바꿀 때 여기만 수정
# ============================================================
CFG = {
    # maps/ 아래 yaml 파일명만 (절대경로 넣어도 됨)
    "map_name": "cartographer_map_20260805_215352.yaml",
    "map_dir": CENTERLINE_CFG["map_dir"],
    "centerline_csv": os.path.join(script_dir, "..", "config", "centerline.csv"),
    "out_csv": os.path.join(script_dir, "..", "config", "raceline.csv"),
}


def world_to_pixel(
    x: float, y: float, height: int, resolution: float, origin_x: float, origin_y: float
) -> tuple[float, float]:
    """맵 프레임 (x,y) → 이미지 (row, col). pixel_to_world의 역변환."""
    col = (x - origin_x) / resolution
    row = (height - 1) - (y - origin_y) / resolution
    return row, col


def load_centerline_csv(path: str) -> list[tuple[float, float]]:
    """CSV에서 (x, y) waypoints 로드. 'x,y' 또는 헤더 스킵."""
    pts: list[tuple[float, float]] = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)
    start = 0
    for i, r in enumerate(rows):
        if not r or r[0].strip().startswith("#"):
            start = i + 1
            continue
        if len(r) < 2:
            continue
        try:
            x = float(r[0].strip())
            y = float(r[1].strip())
            pts.append((x, y))
        except ValueError:
            if i == start and ("x" in (r[0] + r[1]).lower() or "m" in (r[0] + r[1]).lower()):
                start = i + 1
            continue
    return pts


def centerline_world_to_pixel(
    points_xy: list[tuple[float, float]],
    height: int, resolution: float, origin_x: float, origin_y: float,
) -> list[tuple[float, float]]:
    """Step 1: centerline (x,y) → (row, col) 리스트."""
    out = []
    for x, y in points_xy:
        r, c = world_to_pixel(x, y, height, resolution, origin_x, origin_y)
        out.append((r, c))
    return out


def get_tangent_normal(
    points: list[tuple[float, float]], lookahead: int
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Step 3: 각 점에서 접선 t, 법선 n (단위벡터). (row,col) 기준. n은 '왼쪽' 방향."""
    n_pts = len(points)
    result = []
    for i in range(n_pts):
        i_prev = max(0, i - lookahead)
        i_next = min(n_pts - 1, i + lookahead)
        if i_prev == i_next:
            # 끝점 등: 앞뒤 한 칸으로 근사
            i_prev = max(0, i - 1)
            i_next = min(n_pts - 1, i + 1)
        p_prev = points[i_prev]
        p_next = points[i_next]
        dr = p_next[0] - p_prev[0]
        dc = p_next[1] - p_prev[1]
        norm = np.sqrt(dr * dr + dc * dc)
        if norm < 1e-9:
            # 직선: 기본 접선 (col 증가 방향), 법선 (row 감소 = 위쪽)
            t = (0.0, 1.0)
            n = (-1.0, 0.0)
        else:
            t = (dr / norm, dc / norm)
            # 왼쪽 법선: (-dc, dr) in (row,col). row가 위로 갈수록 작아지므로 '왼쪽'은 (-dc, dr)
            n = (-dc / norm, dr / norm)
        result.append((t, n))
    return result


def get_track_widths(
    free_mask: np.ndarray,
    points: list[tuple[float, float]],
    tangents_normals: list[tuple[tuple[float, float], tuple[float, float]]],
    margin_px: float,
) -> tuple[list[float], list[float]]:
    """Step 4: 각 점에서 벽까지 거리. d_max = +왼쪽 여유(픽셀), d_min = -오른쪽 여유(픽셀). margin 적용."""
    height, width = free_mask.shape
    d_max_list = []
    d_min_list = []
    for i, (r, c) in enumerate(points):
        _, n = tangents_normals[i]
        perp_r, perp_c = n[0], n[1]
        # +n 방향으로 벽까지 (왼쪽)
        k1 = 0
        while True:
            rr = int(round(r + (k1 + 1) * perp_r))
            cc = int(round(c + (k1 + 1) * perp_c))
            if rr < 0 or rr >= height or cc < 0 or cc >= width:
                break
            if free_mask[rr, cc] == 0:
                break
            k1 += 1
        # -n 방향으로 벽까지 (오른쪽)
        k2 = 0
        while True:
            rr = int(round(r - (k2 + 1) * perp_r))
            cc = int(round(c - (k2 + 1) * perp_c))
            if rr < 0 or rr >= height or cc < 0 or cc >= width:
                break
            if free_mask[rr, cc] == 0:
                break
            k2 += 1
        w_left = float(k1)
        w_right = float(k2)
        d_max_list.append(max(0.0, w_left - margin_px))
        d_min_list.append(-max(0.0, w_right - margin_px))
    return d_min_list, d_max_list


def discrete_heading_change(
    points: list[tuple[float, float]], L: int, closed: bool = True
) -> list[float]:
    """각 점에서 헤딩 변화량 Δψ [rad]. v1=p[i]-p[i-L], v2=p[i+L]-p[i], Δψ=∠(v1,v2).
    양수=좌회전. tangent_lookahead와 같은 L 사용 권장."""
    n = len(points)
    if n < 2 * L + 1:
        return [0.0] * n
    pts = np.array(points, dtype=float)
    dpsi = np.zeros(n)
    for i in range(n):
        i_prev = (i - L) % n if closed else max(0, i - L)
        i_next = (i + L) % n if closed else min(n - 1, i + L)
        v1 = pts[i] - pts[i_prev]
        v2 = pts[i_next] - pts[i]
        n1 = np.sqrt(np.sum(v1 ** 2)) + 1e-12
        n2 = np.sqrt(np.sum(v2 ** 2)) + 1e-12
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        dpsi[i] = np.arctan2(cross, dot + 1e-12)
    return dpsi.tolist()


def discrete_curvature(points: list[tuple[float, float]], closed: bool = True) -> list[float]:
    """각 점에서 이산 곡률 κ (rad/px). 양수=좌회전. (보조용, apex 위치 등)."""
    n = len(points)
    if n < 3:
        return [0.0] * n
    pts = np.array(points, dtype=float)
    kappa = np.zeros(n)
    for i in range(n):
        i0 = (i - 1) % n if closed else max(0, i - 1)
        i1 = i
        i2 = (i + 1) % n if closed else min(n - 1, i + 1)
        v1 = pts[i1] - pts[i0]
        v2 = pts[i2] - pts[i1]
        L1 = np.sqrt(np.sum(v1 ** 2)) + 1e-12
        L2 = np.sqrt(np.sum(v2 ** 2)) + 1e-12
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        angle = np.arctan2(cross, dot + 1e-12)
        arc = 0.5 * (L1 + L2)
        kappa[i] = angle / arc if arc > 1e-9 else 0.0
    return kappa.tolist()


def detect_corners(
    dpsi: list[float], delta_psi_thresh: float, min_corner_len: int
) -> list[tuple[int, int, str]]:
    """Step 5: |Δψ| > delta_psi_thresh [rad] 인 연속 구간을 코너로. [(i_start, i_end, 'left'|'right'), ...]."""
    n = len(dpsi)
    corners = []
    i = 0
    while i < n:
        if abs(dpsi[i]) <= delta_psi_thresh:
            i += 1
            continue
        side = "left" if dpsi[i] > 0 else "right"
        j = i
        while j < n and (abs(dpsi[j]) > delta_psi_thresh or j - i < min_corner_len):
            if abs(dpsi[j]) > delta_psi_thresh:
                side = "left" if dpsi[j] > 0 else "right"
            j += 1
            if j - i > 500:
                break
        i_start = max(0, i - 2)
        i_end = min(n, j + 2)
        if i_end - i_start >= min_corner_len:
            corners.append((i_start, i_end, side))
        i = j
    return corners


def merge_corners(
    corners: list[tuple[int, int, str]], n: int, merge_gap: int
) -> list[tuple[int, int, str]]:
    """인접/겹치는 코너 구간을 하나로 합침. 코너 하나당 하나의 라인만 나오게."""
    if not corners:
        return []
    # 구간을 (i_start, i_end, direction) 순으로 정렬 (시작 인덱스 기준)
    sorted_corners = sorted(corners, key=lambda c: c[0])
    merged: list[tuple[int, int, str]] = []
    cur_start, cur_end, cur_side = sorted_corners[0]

    for i in range(1, len(sorted_corners)):
        n_start, n_end, n_side = sorted_corners[i]
        gap = n_start - cur_end
        if gap <= merge_gap or (cur_end >= n_start):
            cur_end = max(cur_end, n_end)
        else:
            merged.append((cur_start, cur_end, cur_side))
            cur_start, cur_end, cur_side = n_start, n_end, n_side

    merged.append((cur_start, cur_end, cur_side))
    return merged


def _smoothstep(t: float) -> float:
    """C2 스무스 스텝 (0→1). 곡률 연속에 유리."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _get_d_out_at(
    d_min: list[float], d_max: list[float], direction: str, alpha_out: float, i: int
) -> float:
    """인덱스 i에서 바깥쪽 오프셋. 좌회전=바깥은 오른쪽(d_min), 우회전=바깥은 왼쪽(d_max)."""
    d_lo, d_hi = d_min[i], d_max[i]
    if direction == "left":
        d_out = -alpha_out * abs(d_lo)
    else:
        d_out = alpha_out * d_hi
    return float(np.clip(d_out, d_lo, d_hi))


def _corner_width_ratio(d_min: list[float], d_max: list[float], i_start: int, i_end: int) -> float:
    """기준 B: 코너 구간에서 '오프셋 최대치/폭' 비율. 폭 = d_max + |d_min|, 최대치 = min(d_max, |d_min|)."""
    ratios = []
    for i in range(i_start, min(i_end, len(d_min))):
        dm, dx = d_min[i], d_max[i]
        w = dx + abs(dm)
        if w < 1e-9:
            continue
        half = min(dx, abs(dm))
        ratios.append(half / w)
    return float(np.mean(ratios)) if ratios else 0.0


def _corner_max_dpsi(dpsi: list[float], i_start: int, i_end: int) -> float:
    """코너 구간 내 최대 |Δψ| [rad] (뾰족한 V자 판별용)."""
    if i_end <= i_start:
        return 0.0
    return float(np.max(np.abs(dpsi[i_start:i_end])))


def _corner_mean_width(d_min: list[float], d_max: list[float], i_start: int, i_end: int) -> float:
    """코너 구간 평균 트랙 폭 W = d_max + |d_min| [px]."""
    w = []
    for i in range(i_start, min(i_end, len(d_min))):
        w.append(d_max[i] + abs(d_min[i]))
    return float(np.mean(w)) if w else 0.0


def _global_median_width(d_min: list[float], d_max: list[float]) -> float:
    """전체 경로의 트랙 폭 중앙값 [px]. 맵마다 스케일이 다르므로 W_pinch 자동화용."""
    w = [d_max[i] + abs(d_min[i]) for i in range(len(d_min))]
    return float(np.median(w)) if w else 0.0


def offset_profile_corner(
    i_start: int, i_end: int, direction: str,
    d_min: list[float], d_max: list[float],
    kappa: list[float],
    alpha_out: float, beta_in: float,
    extend_back: int,
    extend_fwd: int,
    n: int,
    apex_fraction: float = 0.65,
    maintain_out_approach: bool = False,
    maintain_out_runout: bool = False,
    prev_corner_end: int | None = None,
    next_corner_start: int | None = None,
) -> list[tuple[int, float]]:
    """Step 6: 한 코너에서 Out–In–Out. 연속 같은 방향이면 갭 전체를 아웃으로 채움(인-아웃-인 제거)."""
    corner_len = i_end - i_start
    apex_offset = int(apex_fraction * corner_len)
    apex_i = i_start + min(apex_offset, corner_len - 1)
    if maintain_out_approach and prev_corner_end is not None:
        i_start_eff = max(0, prev_corner_end)
    else:
        i_start_eff = max(0, i_start - extend_back)
    if maintain_out_runout and next_corner_start is not None:
        i_end_eff = min(n, next_corner_start)
    else:
        i_end_eff = min(n, i_end + extend_fwd)
    d_out_at_start = _get_d_out_at(d_min, d_max, direction, alpha_out, i_start)
    d_out_at_end = _get_d_out_at(d_min, d_max, direction, alpha_out, i_end - 1)

    out_pairs = []
    for i in range(i_start_eff, i_end_eff):
        d_lo = d_min[i]
        d_hi = d_max[i]
        if direction == "left":
            d_out = -alpha_out * abs(d_lo)
            d_in = beta_in * d_hi
        else:
            d_out = alpha_out * d_hi
            d_in = -beta_in * abs(d_lo)
        d_out = np.clip(d_out, d_lo, d_hi)
        d_in = np.clip(d_in, d_lo, d_hi)

        if i < i_start:
            if maintain_out_approach:
                d = d_out_at_start
            else:
                span = max(1, i_start - i_start_eff)
                frac = _smoothstep((i - i_start_eff) / span)
                d = frac * d_out_at_start
        elif i <= apex_i:
            seg_len = max(1, apex_i - i_start)
            frac = (i - i_start) / seg_len
            frac = 0.5 * (1.0 - np.cos(np.pi * frac))
            d = d_out + frac * (d_in - d_out)
        elif i < i_end:
            seg_len = max(1, i_end - 1 - apex_i)
            frac = (i - apex_i) / seg_len
            frac = 0.5 * (1.0 - np.cos(np.pi * frac))
            d = d_in + frac * (d_out - d_in)
        else:
            if maintain_out_runout:
                d = d_out_at_end
            else:
                span = max(1, i_end_eff - i_end)
                frac = _smoothstep((i - i_end) / span)
                d = (1.0 - frac) * d_out_at_end
        d = np.clip(d, d_lo, d_hi)
        out_pairs.append((i, float(d)))
    return out_pairs


def build_full_offset(
    n: int,
    d_min: list[float], d_max: list[float],
    corners: list[tuple[int, int, str]],
    kappa: list[float],
    dpsi: list[float],
    alpha_out: float, beta_in: float,
    extend_back: int,
    extend_fwd: int,
    m_per_pt: float,
    width_ratio_thresh: float = 0.2,
    sharp_delta_psi_thresh: float = 0.26,
    beta_sharp: float = 0.62,
    W_pinch: float = 15.0,
    centerline_if_sharp: float = 0.0,
    apex_fraction: float = 0.65,
    same_dir_gap_m: float = 4.0,
) -> list[float]:
    """Step 6: 모든 코너 오프셋. 연속 같은 방향 코너는 gap 전체를 아웃으로 채움(인-아웃-인 제거)."""
    d = [0.0] * n
    for idx, (i_start, i_end, direction) in enumerate(corners):
        ratio = _corner_width_ratio(d_min, d_max, i_start, i_end)
        if ratio < width_ratio_thresh:
            continue
        max_dpsi = _corner_max_dpsi(dpsi, i_start, i_end)
        if centerline_if_sharp > 0 and max_dpsi > centerline_if_sharp:
            continue
        p_start, p_end, p_dir = corners[idx - 1] if idx > 0 else (None, None, None)
        n_start, n_end, n_dir = corners[idx + 1] if idx + 1 < len(corners) else (None, None, None)
        gap_m_prev = (i_start - p_end) * m_per_pt if p_end is not None else float("inf")
        gap_m_next = (n_start - i_end) * m_per_pt if n_start is not None else float("inf")
        maintain_out_approach = (
            p_end is not None and p_dir == direction and gap_m_prev <= same_dir_gap_m
        )
        maintain_out_runout = (
            n_start is not None and n_dir == direction and gap_m_next <= same_dir_gap_m
        )
        W_avg = _corner_mean_width(d_min, d_max, i_start, i_end)
        scale = 1.0
        if W_pinch > 0 and W_avg < W_pinch:
            scale = W_avg / W_pinch
        alpha_eff = alpha_out * scale
        beta_eff = beta_in * scale
        if max_dpsi > sharp_delta_psi_thresh:
            beta_eff = beta_sharp * scale
        prev_end = int(p_end) if p_end is not None else None
        next_start = int(n_start) if n_start is not None else None
        pairs = offset_profile_corner(
            i_start, i_end, direction, d_min, d_max, kappa, alpha_eff, beta_eff,
            extend_back, extend_fwd, n,
            apex_fraction=apex_fraction,
            maintain_out_approach=maintain_out_approach,
            maintain_out_runout=maintain_out_runout,
            prev_corner_end=prev_end if maintain_out_approach else None,
            next_corner_start=next_start if maintain_out_runout else None,
        )
        for i, di in pairs:
            if 0 <= i < n:
                d[i] = di
    return d


def _gaussian_kernel(radius: int, sigma: float | None = None) -> np.ndarray:
    """1D 가우시안 커널 (정규화). radius=양쪽 반창."""
    if sigma is None or sigma <= 0:
        sigma = max(0.5, radius / 1.5)
    x = np.arange(-radius, radius + 1, dtype=float)
    w = np.exp(-0.5 * (x / sigma) ** 2)
    return w / np.sum(w)


def smooth_and_clamp_d(
    d: list[float], d_min: list[float], d_max: list[float], window: int, closed: bool = True
) -> list[float]:
    """Step 7: d에 가우시안 가중 이동평균 후 [d_min, d_max] 클램프. 원호처럼 스무스하게."""
    n = len(d)
    if window < 1 or n < window * 2 + 1:
        return [np.clip(d[i], d_min[i], d_max[i]) for i in range(n)]
    kernel = _gaussian_kernel(window)
    arr = np.array(d, dtype=float)
    d_min_arr = np.array(d_min)
    d_max_arr = np.array(d_max)
    out = np.zeros_like(arr)
    for i in range(n):
        acc = 0.0
        wsum = 0.0
        for k, w in enumerate(kernel):
            j = i + k - window
            if closed:
                j = j % n
                if j < 0:
                    j += n
            else:
                if j < 0 or j >= n:
                    continue
            acc += w * arr[j]
            wsum += w
        if wsum > 1e-12:
            out[i] = acc / wsum
        else:
            out[i] = arr[i]
    out = np.clip(out, d_min_arr, d_max_arr)
    return out.tolist()


def apply_offset(
    points: list[tuple[float, float]],
    tangents_normals: list[tuple[tuple[float, float], tuple[float, float]]],
    d: list[float],
) -> list[tuple[float, float]]:
    """Step 8: p_race[i] = p_center[i] + d[i] * n[i] (픽셀 공간)."""
    out = []
    for i, (r, c) in enumerate(points):
        _, n = tangents_normals[i]
        nr, nc = n[0], n[1]
        di = d[i] if i < len(d) else 0.0
        r_new = r + di * nr
        c_new = c + di * nc
        out.append((r_new, c_new))
    return out


def smooth_polyline_closed(points: list[tuple[float, float]], window: int) -> list[tuple[float, float]]:
    """닫힌 경로 이동평균 스무딩 (인덱스 랩). 원호형 스무스 곡선 유지."""
    n = len(points)
    if window < 1 or n < window * 2 + 1:
        return list(points)
    pts = np.array(points, dtype=float)
    out = np.zeros_like(pts)
    for i in range(n):
        acc = np.zeros(2)
        cnt = 0
        for j in range(i - window, i + window + 1):
            k = j % n
            if k < 0:
                k += n
            acc += pts[k]
            cnt += 1
        out[i] = acc / cnt
    return [tuple(out[i]) for i in range(n)]


def main():
    parser = argparse.ArgumentParser(
        description="Centerline CSV + map YAML → raceline CSV (outside-apex-outside heuristic)"
    )
    parser.add_argument(
        "--centerline",
        default=os.path.abspath(CFG["centerline_csv"]),
        help="Input centerline CSV (x,y)",
    )
    default_map = resolve_map_yaml(CFG["map_name"], CFG["map_dir"])
    parser.add_argument(
        "--map",
        default=default_map,
        help=f"Map YAML path (default from CFG map_name={CFG['map_name']!r})",
    )
    parser.add_argument(
        "--out",
        default=os.path.abspath(CFG["out_csv"]),
        help="Output raceline CSV path",
    )
    parser.add_argument(
        "--invert-free",
        action="store_true",
        help="맵 해석: 밝은 쪽=도로 (센터라인 스크립트와 동일하게)",
    )
    parser.add_argument("--resample-step", type=float, default=1.5, help="픽셀 기준 리샘플 간격")
    parser.add_argument("--smooth-window", type=int, default=5, help="센터라인 스무딩 창 크기")
    parser.add_argument("--tangent-lookahead", type=int, default=15, help="접선/법선 계산 lookahead")
    parser.add_argument("--margin", type=float, default=3.0, help="벽에서 margin(px) 제외. 2~6 권장")
    parser.add_argument("--delta-psi-thresh", type=float, default=0.12, help="코너 판정: |Δψ|>이 값(rad)")
    parser.add_argument("--min-corner-len", type=int, default=5, help="코너 최소 길이(포인트)")
    parser.add_argument("--merge-gap-m", type=float, default=2.5, help="인접 코너 머지 거리 [m]")
    parser.add_argument("--entry-m", type=float, default=1.9, help="진입 구간 길이 [m]. 길수록 아웃 구간 뚜렷")
    parser.add_argument("--exit-m", type=float, default=1.9, help="탈출 구간 길이 [m]. 길수록 아웃 구간 뚜렷")
    parser.add_argument("--width-ratio-thresh", type=float, default=0.2, help="오프셋최대치/폭 < 이 값이면 센터라인")
    parser.add_argument("--sharp-delta-psi-thresh", type=float, default=0.26, help="max|Δψ|>이면 뾰족 코너, β_sharp")
    parser.add_argument("--centerline-if-sharp", type=float, default=0.0, help="max|Δψ|>이 값(rad)이면 해당 코너만 O-I-O 생략·센터라인. 0=비활성(모든 코너에 레이싱라인)")
    parser.add_argument("--apex-fraction", type=float, default=0.68, help="delayed apex 비율. 레이싱라인에선 0.65~0.7")
    parser.add_argument("--beta-sharp", type=float, default=0.72, help="뾰족(V자) 코너 apex 스케일. 클수록 인 더 깊이")
    parser.add_argument("--W-pinch", type=float, default=0.0, help="폭(px)<이 값이면 α,β 감쇠. 0=자동(폭 중앙값 25%%)")
    parser.add_argument("--alpha-out", type=float, default=0.68, help="바깥쪽(진입/탈출) 오프셋. 0.65~0.75=아웃 뚜렷")
    parser.add_argument("--beta-in", type=float, default=0.84, help="안쪽(apex) 오프셋. 0.8~0.9=인 뚜렷")
    parser.add_argument("--same-dir-gap-m", type=float, default=4.0, help="연속 같은 방향 코너 gap [m] 이하면 갭 전체 아웃")
    parser.add_argument("--d-smooth-window", type=int, default=0, help="d[i] 가우시안 스무딩 반창. 0=자동")
    parser.add_argument(
        "--start-at-zero",
        action="store_true",
        help="출력 첫 점을 (0,0)으로 회전",
    )
    parser.add_argument(
        "--origin-x", type=float, default=None,
        help="출력 좌표 원점 x (빼기)",
    )
    parser.add_argument(
        "--origin-y", type=float, default=None,
        help="출력 좌표 원점 y (빼기)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.centerline):
        print(f"Centerline not found: {args.centerline}", file=sys.stderr)
        return 1
    if not os.path.isfile(args.map):
        print(f"Map not found: {args.map}", file=sys.stderr)
        return 1

    print("Step 0: Loading inputs...")
    points_xy = load_centerline_csv(args.centerline)
    if len(points_xy) < 3:
        print("Centerline has fewer than 3 points.", file=sys.stderr)
        return 1
    free_mask, resolution, origin_x, origin_y, (height, width) = load_map(
        args.map, invert_free=args.invert_free
    )
    print(f"  Centerline: {args.centerline} ({len(points_xy)} pts)")
    print(f"  Map: {args.map} ({height}x{width}, res={resolution})")
    print(f"  Out: {args.out}")

    points_px = centerline_world_to_pixel(
        points_xy, height, resolution, origin_x, origin_y
    )

    points_px = resample_polyline_by_arc_length(
        points_px, step=args.resample_step, closed=True
    )
    if args.smooth_window > 0:
        points_px = smooth_polyline(points_px, window=args.smooth_window)
    print(f"  After resample(step={args.resample_step}) + smooth(window={args.smooth_window}): {len(points_px)} pts")

    tangents_normals = get_tangent_normal(points_px, args.tangent_lookahead)
    d_min, d_max = get_track_widths(
        free_mask, points_px, tangents_normals, args.margin
    )
    print(f"  Track width: d_min/d_max per point (margin={args.margin}px)")

    n = len(points_px)
    arc_step_px = getattr(args, "resample_step", 1.5)
    m_per_pt = arc_step_px * resolution
    if m_per_pt < 1e-9:
        m_per_pt = 0.05
    entry_pts = max(1, int(getattr(args, "entry_m", 1.9) / m_per_pt))
    exit_pts = max(1, int(getattr(args, "exit_m", 1.9) / m_per_pt))
    merge_gap_m = getattr(args, "merge_gap_m", 2.5)
    merge_gap = max(20, int(merge_gap_m / m_per_pt))

    L = max(1, getattr(args, "tangent_lookahead", 15))
    dpsi = discrete_heading_change(points_px, L, closed=True)
    delta_psi_th = getattr(args, "delta_psi_thresh", 0.12)
    corners = detect_corners(dpsi, delta_psi_th, args.min_corner_len)
    corners = merge_corners(corners, n, merge_gap)
    print(f"  Corners (after merge): {len(corners)} (Δψ_th={np.degrees(args.delta_psi_thresh):.1f}°, entry_pts={entry_pts}, exit_pts={exit_pts}, merge_gap_m={merge_gap_m})")
    if len(corners) == 0:
        print("  WARNING: No corners detected. Try --delta-psi-thresh 0.08 or lower. Output will be centerline.")

    kappa = discrete_curvature(points_px, closed=True)
    W_pinch_arg = getattr(args, "W_pinch", 0.0)
    W_pinch_eff = (0.25 * _global_median_width(d_min, d_max)) if W_pinch_arg <= 0 else W_pinch_arg

    d = build_full_offset(
        n, d_min, d_max, corners, kappa, dpsi,
        args.alpha_out, args.beta_in,
        extend_back=entry_pts,
        extend_fwd=exit_pts,
        m_per_pt=m_per_pt,
        width_ratio_thresh=getattr(args, "width_ratio_thresh", 0.2),
        sharp_delta_psi_thresh=getattr(args, "sharp_delta_psi_thresh", 0.26),
        beta_sharp=getattr(args, "beta_sharp", 0.72),
        W_pinch=W_pinch_eff,
        centerline_if_sharp=getattr(args, "centerline_if_sharp", 0.0),
        apex_fraction=getattr(args, "apex_fraction", 0.68),
        same_dir_gap_m=getattr(args, "same_dir_gap_m", 4.0),
    )

    smooth_win = getattr(args, "d_smooth_window", 0)
    if smooth_win <= 0:
        smooth_win = max(18, min(42, n // 100))
    d = smooth_and_clamp_d(d, d_min, d_max, smooth_win, closed=True)

    race_px = apply_offset(points_px, tangents_normals, d)
    race_smooth_win = max(5, min(15, n // 350))
    if race_smooth_win > 0 and len(race_px) >= race_smooth_win * 2 + 1:
        race_px = smooth_polyline_closed(race_px, race_smooth_win)

    race_xy = []
    for (r, c) in race_px:
        x, y = pixel_to_world(r, c, height, resolution, origin_x, origin_y)
        race_xy.append((x, y))

    # 원점 옵션
    if args.origin_x is not None and args.origin_y is not None:
        race_xy = [(x - args.origin_x, y - args.origin_y) for x, y in race_xy]
        print(f"  Origin shift: ({args.origin_x}, {args.origin_y})")
    if args.start_at_zero and len(race_xy) >= 2:
        def dist0(p):
            return p[0] ** 2 + p[1] ** 2
        i0 = min(range(len(race_xy)), key=lambda i: dist0(race_xy[i]))
        x0, y0 = race_xy[i0]
        race_xy = [(x - x0, y - y0) for x, y in (race_xy[i0:] + race_xy[:i0])]
        print("  start_at_zero: first point (0,0)")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in race_xy:
            w.writerow([x, y])
    print(f"Wrote {len(race_xy)} points to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
