#!/usr/bin/env python3
"""
맵 YAML+이미지에서 주행 통로 중심선을 CSV(x,y)로 뽑는 보강 버전.

핵심 개선 (기존 스켈레톤+greedy NN 방식 대비):
- polar DT 센터라인: 인필드 중심에서 방사형으로 벽 이격(DT) 최대점 샘플
- 행/열 스캔: 각 free 구간에서 DT 최대점 + 비주행 침범 없는 greedy 연결
- 얇은 free 다리 제거(opening) + 벽 gap 보수로 인필드 지름길 억제
- 후보 경로 검증: 자기교차·비주행 침범·최소 둘레 미달 경로 자동 제외
- greedy NN 폴백(순서 뒤섞임) 제거

출력 좌표: map 프레임 x=origin_x+col*res, y=origin_y+(H-1-row)*res

예:
  # CFG map_name 만 고치고 실행
  python3 extract_centerline_from_map.py
  # 또는 CLI로 덮어쓰기
  python3 extract_centerline_from_map.py --map /path/to/map.yaml --out ../config/out.csv
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import (
    binary_closing,
    binary_dilation,
    binary_opening,
    label as ndi_label,
)

try:
    from scipy.ndimage import distance_transform_edt, maximum_filter
except ImportError as e:
    print("Missing scipy:", e, file=sys.stderr)
    sys.exit(1)

try:
    from skimage.morphology import skeletonize
except ImportError as e:
    print("Install scikit-image: pip install scikit-image", e, file=sys.stderr)
    sys.exit(1)


# ============================================================
# USER TUNING — 맵 바꿀 때 여기만 수정
# ============================================================
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..", ".."))
_DEFAULT_MAP_DIR = os.path.join(_WS_ROOT, "maps")

CFG = {
    # maps/ 아래 yaml 파일명만 (절대경로 넣어도 됨)
    "map_name": "cartographer_map_20260803_202601.yaml",
    "map_dir": _DEFAULT_MAP_DIR,
    "out_csv": os.path.join(_SCRIPT_DIR, "..", "config", "centerline.csv"),
}


def resolve_map_yaml(map_name: str, map_dir: str = "") -> str:
    """CFG map_name → 절대경로. 절대경로면 그대로."""
    name = str(map_name).strip()
    if not name:
        raise ValueError("map_name is empty — CFG['map_name'] 에 yaml 파일명을 넣으세요.")
    if os.path.isabs(name):
        if not os.path.isfile(name):
            raise FileNotFoundError(f"map yaml not found: {name}")
        return os.path.abspath(name)
    base = str(map_dir).strip() or _DEFAULT_MAP_DIR
    cand = os.path.abspath(os.path.join(base, name))
    if not os.path.isfile(cand):
        raise FileNotFoundError(
            f"map yaml not found: {cand}\n"
            f"  CFG map_name={name!r}, map_dir={base}"
        )
    return cand


def _resolve_map_image_path(yaml_path: str, image_field: str) -> str:
    """YAML image 경로가 다른 머신 절대경로여도 yaml 옆 png를 우선 찾음."""
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    if os.path.isabs(image_field) and os.path.isfile(image_field):
        return image_field
    rel = os.path.join(yaml_dir, image_field)
    if os.path.isfile(rel):
        return rel
    base = os.path.basename(image_field)
    local = os.path.join(yaml_dir, base)
    if os.path.isfile(local):
        return local
    stem = os.path.splitext(os.path.basename(yaml_path))[0]
    for ext in (".png", ".pgm", ".bmp"):
        cand = os.path.join(yaml_dir, stem + ext)
        if os.path.isfile(cand):
            return cand
    return rel if not os.path.isabs(image_field) else image_field


def _row_run_center(free_inds: np.ndarray) -> float | None:
    if len(free_inds) == 0:
        return None
    gaps = np.diff(free_inds) > 1
    run_starts = np.concatenate([[0], np.where(gaps)[0] + 1])
    run_ends = np.concatenate([np.where(gaps)[0] + 1, [len(free_inds)]])
    best_len = 0
    best_mid = None
    for i in range(len(run_starts)):
        s, e = run_starts[i], run_ends[i]
        if e - s > best_len:
            best_len = e - s
            best_mid = (free_inds[s] + free_inds[e - 1]) / 2.0
    return float(best_mid) if best_mid is not None else None


def _all_row_run_centers(
    free_inds: np.ndarray,
    min_len: int = 4,
    values: np.ndarray | None = None,
) -> list[float]:
    if len(free_inds) == 0:
        return []
    gaps = np.diff(free_inds) > 1
    run_starts = np.concatenate([[0], np.where(gaps)[0] + 1])
    run_ends = np.concatenate([np.where(gaps)[0] + 1, [len(free_inds)]])
    mids: list[float] = []
    for i in range(len(run_starts)):
        s, e = run_starts[i], run_ends[i]
        if e - s < min_len:
            continue
        if values is None:
            mids.append(float((free_inds[s] + free_inds[e - 1]) / 2.0))
        else:
            seg = values[free_inds[s]: free_inds[e - 1] + 1]
            best = int(np.argmax(seg))
            mids.append(float(free_inds[s] + best))
    return mids


def _estimate_ring_center(free: np.ndarray, img_gray: np.ndarray | None = None) -> tuple[float, float]:
    h, w = free.shape
    if img_gray is not None:
        dark = img_gray < 80
        if dark.sum() > 20:
            ys, xs = np.where(dark)
            return float(ys.mean()), float(xs.mean())
    occupied = ~free.astype(bool)
    labeled, n = ndi_label(occupied, structure=np.ones((3, 3), dtype=int))
    if n > 1:
        sizes = np.bincount(labeled.ravel())
        border = np.zeros_like(occupied, dtype=bool)
        border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
        best_area = 0
        best_c = (h / 2.0, w / 2.0)
        for lab in range(1, n + 1):
            comp = labeled == lab
            if np.any(comp & border):
                continue
            area = int(comp.sum())
            if area > best_area:
                best_area = area
                ys, xs = np.where(comp)
                best_c = (float(ys.mean()), float(xs.mean()))
        return best_c
    ys, xs = np.where(free > 0)
    return float(ys.mean()), float(xs.mean())


def polar_dt_centerline(
    free: np.ndarray,
    img_gray: np.ndarray | None = None,
    n_angles: int = 360,
) -> list:
    """링 중심에서 방사형으로 DT 최대점을 샘플 → 도로 정중앙 폐루프."""
    dt = distance_transform_edt(free > 0)
    h, w = free.shape
    cy, cx = _estimate_ring_center(free, img_gray)
    max_r = float(np.hypot(h, w))
    points: list[tuple[float, float]] = []
    for k in range(n_angles):
        ang = 2.0 * np.pi * k / n_angles
        dr = np.sin(ang)
        dc = np.cos(ang)
        best_d = -1.0
        best_pt = None
        for r in np.linspace(0.0, max_r, int(max_r * 2.0)):
            rr = int(round(cy + r * dr))
            cc = int(round(cx + r * dc))
            if not (0 <= rr < h and 0 <= cc < w) or free[rr, cc] == 0:
                continue
            val = float(dt[rr, cc])
            if val > best_d:
                best_d = val
                best_pt = (float(rr), float(cc))
        if best_pt is not None and best_d > 0.5:
            points.append(best_pt)
    points = merge_near_points(points, tol=2.5)
    ordered = order_points_greedy_validated(points, free)
    if len(ordered) >= 8:
        return ordered
    return order_points_by_angle(points)


def segment_stays_in_free(
    p0: tuple[float, float],
    p1: tuple[float, float],
    free: np.ndarray,
    samples_per_px: float = 1.5,
) -> bool:
    h, w = free.shape
    dr = p1[0] - p0[0]
    dc = p1[1] - p0[1]
    steps = max(2, int(np.hypot(dr, dc) * samples_per_px))
    for t in np.linspace(0.0, 1.0, steps):
        rr = int(round(p0[0] + t * dr))
        cc = int(round(p0[1] + t * dc))
        if not (0 <= rr < h and 0 <= cc < w) or free[rr, cc] == 0:
            return False
    return True


def count_segments_crossing_occupied(
    path: list,
    free: np.ndarray,
    closed: bool = True,
) -> int:
    if len(path) < 2:
        return 0
    n = len(path)
    bad = 0
    limit = n if closed else n - 1
    for i in range(limit):
        j = (i + 1) % n if closed else i + 1
        if not segment_stays_in_free(path[i], path[j], free):
            bad += 1
    return bad


def remove_thin_bridges(free: np.ndarray, bridge_px: int) -> np.ndarray:
    """얇은 free 다리(인필드 지름길) 제거 후 최대 연결요소만 유지."""
    if bridge_px < 2:
        return free
    k = int(max(2, bridge_px))
    struct = np.ones((k, k), dtype=bool)
    opened = binary_opening(free.astype(bool), structure=struct)
    labeled, n = ndi_label(opened, structure=np.ones((3, 3), dtype=int))
    if n <= 1:
        return opened.astype(np.uint8)
    sizes = np.bincount(labeled.ravel())
    best = int(np.argmax(sizes[1:]) + 1)
    return (labeled == best).astype(np.uint8)


def estimate_min_corridor_px(free: np.ndarray) -> float:
    dt = distance_transform_edt(free > 0)
    vals = dt[free > 0]
    if vals.size == 0:
        return 4.0
    return float(max(4.0, np.percentile(vals, 20) * 2.0))


def order_points_greedy_validated(
    points: list[tuple[float, float]],
    free: np.ndarray,
) -> list:
    """가까운 점부터 이으되, 비주행을 가로지르는 간선은 건너뜀."""
    pts_u = list(dict.fromkeys((float(a), float(b)) for a, b in points))
    if len(pts_u) < 3:
        return pts_u
    remaining = set(pts_u)
    start = min(remaining, key=lambda p: (p[0], p[1]))
    path = [start]
    remaining.remove(start)
    cur = start
    while remaining:
        ranked = sorted(
            remaining,
            key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2,
        )
        nxt = None
        for cand in ranked[:48]:
            if segment_stays_in_free(cur, cand, free):
                nxt = cand
                break
        if nxt is None:
            break
        path.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    if len(remaining) == 0 and len(path) >= 3:
        if segment_stays_in_free(path[-1], path[0], free):
            return path
    return []


def load_yaml_map(
    yaml_path: str,
    invert_free: bool | None,
    free_thresh: float,
    unknown_as_occupied: bool,
    unknown_low: int,
    unknown_high: int,
):
    """invert_free=None 이면 auto: 흰도로/검도로 후보 중 점수 높은 것 선택."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img_path = _resolve_map_image_path(yaml_path, str(meta["image"]))
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    negate = int(meta.get("negate", 0))
    max_val = 255.0

    img = np.array(Image.open(img_path).convert("L"))
    if negate:
        img = 255 - img

    if invert_free is not None:
        free = _free_mask_single(
            img,
            invert_free,
            free_thresh,
            max_val,
            unknown_as_occupied,
            unknown_low,
            unknown_high,
        )
        mode = "bright_road" if invert_free else "dark_road"
        return free, resolution, origin_x, origin_y, img.shape, mode

    free_dark = _free_mask_single(
        img,
        False,
        free_thresh,
        max_val,
        unknown_as_occupied,
        unknown_low,
        unknown_high,
    )
    free_bright = _free_mask_single(
        img,
        True,
        free_thresh,
        max_val,
        unknown_as_occupied,
        unknown_low,
        unknown_high,
    )
    s_dark = _score_free_mask(free_dark)
    s_bright = _score_free_mask(free_bright)
    print(f"  auto scores: dark_road={s_dark:.4f}, bright_road={s_bright:.4f}")
    if s_bright >= s_dark:
        print("  → chosen: bright pixels = drivable (typical Cartographer / map_server)")
        return free_bright, resolution, origin_x, origin_y, img.shape, "bright_road(auto)"
    print("  → chosen: dark pixels = drivable")
    return free_dark, resolution, origin_x, origin_y, img.shape, "dark_road(auto)"


def _free_mask_single(
    img: np.ndarray,
    invert_free: bool,
    free_thresh: float,
    max_val: float,
    unknown_as_occupied: bool,
    unknown_low: int,
    unknown_high: int,
) -> np.ndarray:
    if unknown_as_occupied:
        mid = np.logical_and(img >= unknown_low, img <= unknown_high)
    else:
        mid = np.zeros_like(img, dtype=bool)

    if invert_free:
        road = img >= (1.0 - free_thresh) * max_val
    else:
        road = img <= free_thresh * max_val
    if unknown_as_occupied:
        road = np.logical_and(road, ~mid)
    return road.astype(np.uint8)


def _score_free_mask(free: np.ndarray) -> float:
    s = int(free.sum())
    if s < 50:
        return -1e9
    labeled, n = ndi_label(free, structure=np.ones((3, 3), dtype=int))
    if n <= 0:
        return -1e9
    sizes = np.bincount(labeled.ravel())
    if len(sizes) <= 1:
        return -1e9
    largest = int(sizes[1:].max())
    largest_ratio = largest / (s + 1e-9)
    fill = free.mean()
    score = largest_ratio - 0.2 * max(0, n - 1)
    if fill > 0.88:
        score -= 1.5
    if fill < 0.001:
        score -= 2.0
    return float(score)


def morph_cleanup(free: np.ndarray, close_iters: int, open_iters: int) -> np.ndarray:
    struct = np.ones((3, 3), dtype=bool)
    m = free.astype(bool)
    for _ in range(close_iters):
        m = binary_closing(m, structure=struct)
    for _ in range(open_iters):
        m = binary_opening(m, structure=struct)
    return m.astype(np.uint8)


def repair_broken_wall_barriers(
    free: np.ndarray,
    *,
    dilate_iters: int,
    close_radius: int,
    close_iters: int,
    keep_largest_free: bool,
) -> np.ndarray:
    """
    occupied(비주행)을 팽창·closing하여 맵 상 틈을 메운 뒤 free = ~occupied 로 재정의.
    내벽이 끊긴 도넛 맵에서 인필드 지름길이 생기는 것을 줄임.
    """
    if dilate_iters <= 0 and close_radius <= 0:
        return free
    struct3 = np.ones((3, 3), dtype=bool)
    occupied = ~(free.astype(bool))
    for _ in range(max(0, dilate_iters)):
        occupied = binary_dilation(occupied, structure=struct3)
    if close_radius > 0:
        k = 2 * close_radius + 1
        struct_k = np.ones((k, k), dtype=bool)
        for _ in range(max(1, close_iters)):
            occupied = binary_closing(occupied, structure=struct_k)
    repaired = (~occupied).astype(np.uint8)
    if not keep_largest_free:
        return repaired
    labeled, n = ndi_label(repaired, structure=np.ones((3, 3), dtype=int))
    if n <= 1:
        return repaired
    sizes = np.bincount(labeled.ravel())
    best = int(np.argmax(sizes[1:]) + 1)
    return (labeled == best).astype(np.uint8)


def corridor_from_distance(
    free: np.ndarray,
    min_dt_px: float,
    max_dt_px: float,
) -> np.ndarray:
    """벽까지 거리 dt: 너무 안쪽(두꺼운 중심)·너무 바깥(가장자리) 제거."""
    dt_occ = distance_transform_edt(free > 0)
    m = (dt_occ >= min_dt_px).astype(np.uint8) * free
    if max_dt_px > 0:
        m = (dt_occ <= max_dt_px).astype(np.uint8) * m
    return m.astype(np.uint8)


def pixel_to_world(
    row: float,
    col: float,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[float, float]:
    x = origin_x + float(col) * resolution
    y = origin_y + (height - 1 - float(row)) * resolution
    return x, y


def skeleton_to_graph(skel: np.ndarray):
    pts = np.argwhere(skel > 0)
    pt_set = set(tuple(p) for p in pts)

    def neighbors(r, c, s):
        return [
            (r + dr, c + dc)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if (dr != 0 or dc != 0) and (r + dr, c + dc) in s
        ]

    return pt_set, neighbors


def prune_skeleton_tips(pt_set: set, neighbors) -> set:
    out = set(pt_set)
    while True:
        to_remove = [p for p in out if len(neighbors(p[0], p[1], out)) <= 1]
        if not to_remove:
            break
        for p in to_remove:
            out.discard(p)
    return out


def extract_cycle_from_start(start, pt_set: set, neighbors) -> list:
    path = [start]
    cur = start
    prev = None
    for _ in range(len(pt_set) + 5):
        ne = [n for n in neighbors(cur[0], cur[1], pt_set) if n != prev]
        if not ne:
            break
        if len(ne) == 1:
            nxt = ne[0]
        else:
            if prev is not None:
                dr0 = cur[0] - prev[0]
                dc0 = cur[1] - prev[1]
                best = None
                best_dot = -2.0
                for n in ne:
                    dr1 = n[0] - cur[0]
                    dc1 = n[1] - cur[1]
                    norm = np.sqrt(dr1 * dr1 + dc1 * dc1) + 1e-9
                    dot = (dr0 * dr1 + dc0 * dc1) / norm
                    if dot > best_dot:
                        best_dot = dot
                        best = n
                nxt = best if best is not None else ne[0]
            else:
                nxt = ne[0]
        if nxt == start and len(path) > 2:
            break
        prev, cur = cur, nxt
        path.append(cur)
        if len(path) > len(pt_set) + 2:
            break
    return path


def path_arc_length(path: list, closed: bool) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        dr = path[i + 1][0] - path[i][0]
        dc = path[i + 1][1] - path[i][1]
        total += np.sqrt(dr * dr + dc * dc)
    if closed and len(path) >= 3:
        dr = path[0][0] - path[-1][0]
        dc = path[0][1] - path[-1][1]
        total += np.sqrt(dr * dr + dc * dc)
    return total


def extract_largest_cycle(pt_set: set, neighbors) -> list:
    if not pt_set:
        return []
    pts_list = list(pt_set)
    n_try = min(200, len(pts_list))
    if n_try < 1:
        return []
    indices = (
        np.linspace(0, len(pts_list) - 1, n_try, dtype=int)
        if len(pts_list) > 1
        else np.array([0], dtype=int)
    )
    starts = [pts_list[int(i)] for i in indices]
    starts.insert(0, min(pts_list, key=lambda p: (p[0], p[1])))
    best_path: list = []
    best_len = 0.0
    for start in starts:
        path = extract_cycle_from_start(start, pt_set, neighbors)
        if len(path) < 3:
            continue
        length = path_arc_length(path, closed=True)
        if length > best_len:
            best_len = length
            best_path = path

    return best_path


def fallback_order_component(pts: list[tuple[int, int]]) -> list:
    """(사용 안 함) greedy NN — 자기교차 경로를 만들어 제외."""
    pts_u = list(dict.fromkeys((int(a), int(b)) for a, b in pts))
    n = len(pts_u)
    if n < 3:
        return pts_u
    remaining = set(pts_u)
    start = min(remaining)
    path = [start]
    remaining.remove(start)
    cur = start
    while remaining:
        nxt = min(
            remaining,
            key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2,
        )
        path.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    return path


def _orient(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, c, eps: float = 1e-9) -> bool:
    return (
        min(a[0], b[0]) - eps <= c[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= c[1] <= max(a[1], b[1]) + eps
    )


def _segments_intersect(p1, p2, p3, p4) -> bool:
    # 공유 끝점은 인접 세그먼트로 간주
    if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
        return False
    o1 = _orient(p1, p2, p3)
    o2 = _orient(p1, p2, p4)
    o3 = _orient(p3, p4, p1)
    o4 = _orient(p3, p4, p2)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    if abs(o1) < 1e-9 and _on_segment(p1, p2, p3):
        return True
    if abs(o2) < 1e-9 and _on_segment(p1, p2, p4):
        return True
    if abs(o3) < 1e-9 and _on_segment(p3, p4, p1):
        return True
    if abs(o4) < 1e-9 and _on_segment(p3, p4, p2):
        return True
    return False


def count_self_intersections(path: list, closed: bool = True) -> int:
    if len(path) < 4:
        return 0
    n = len(path)
    segs: list[tuple] = []
    for i in range(n):
        j = (i + 1) % n if closed else i + 1
        if not closed and j >= n:
            break
        segs.append((path[i], path[j]))
    hits = 0
    for i in range(len(segs)):
        for k in range(i + 1, len(segs)):
            if closed and i == 0 and k == len(segs) - 1:
                continue
            if _segments_intersect(segs[i][0], segs[i][1], segs[k][0], segs[k][1]):
                hits += 1
    return hits


def merge_near_points(points: list, tol: float = 1.5) -> list:
    if len(points) <= 1:
        return points
    pts = np.array(points, dtype=float)
    used = np.zeros(len(pts), dtype=bool)
    out: list[tuple[float, float]] = []
    for i in range(len(pts)):
        if used[i]:
            continue
        near = np.sum((pts - pts[i]) ** 2, axis=1) <= tol * tol
        used[near] = True
        m = np.mean(pts[near], axis=0)
        out.append((float(m[0]), float(m[1])))
    return out


def order_points_by_angle(points: list) -> list:
    """폐곡선용: 무게중심 기준 각도 정렬."""
    if len(points) < 3:
        return points
    pts = np.array(points, dtype=float)
    cen = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 0] - cen[0], pts[:, 1] - cen[1])
    order = np.argsort(angles)
    return [(float(pts[i, 0]), float(pts[i, 1])) for i in order]


def midpoint_scan_centerline(free: np.ndarray, img_gray: np.ndarray | None = None) -> list:
    """행·열 스캔으로 통로 정중앙(DT 최대) 점 수집."""
    height, width = free.shape
    dt = distance_transform_edt(free > 0)
    min_run = max(4, int(estimate_min_corridor_px(free) * 0.35))
    points: list[tuple[float, float]] = []
    for r in range(height):
        inds = np.where(free[r, :] > 0)[0]
        for c_mid in _all_row_run_centers(inds, min_run, dt[r, :]):
            points.append((float(r), c_mid))
    for c in range(width):
        inds = np.where(free[:, c] > 0)[0]
        for r_mid in _all_row_run_centers(inds, min_run, dt[:, c]):
            points.append((r_mid, float(c)))
    points = merge_near_points(points, tol=2.0)
    ordered = order_points_greedy_validated(points, free)
    if len(ordered) >= 8:
        return ordered
    polar = polar_dt_centerline(free, img_gray)
    if len(polar) >= 8:
        return polar
    return order_points_by_angle(points)


def clamp_path_to_corridor(
    points: list,
    free: np.ndarray,
    dist_to_wall: np.ndarray,
    min_clear_px: float = 2.0,
) -> list:
    h, w = free.shape
    out: list[tuple[float, float]] = []
    for r, c in points:
        ri, ci = int(round(r)), int(round(c))
        if 0 <= ri < h and 0 <= ci < w and free[ri, ci] > 0:
            out.append((float(r), float(c)))
            continue
        best_d = -1.0
        best_pt = (float(r), float(c))
        for dr in range(-12, 13):
            for dc in range(-12, 13):
                nr, nc = ri + dr, ci + dc
                if 0 <= nr < h and 0 <= nc < w and free[nr, nc] > 0:
                    d = float(dist_to_wall[nr, nc])
                    if d > best_d:
                        best_d = d
                        best_pt = (float(nr), float(nc))
        out.append(best_pt)
    if min_clear_px <= 0:
        return out
    refined: list[tuple[float, float]] = []
    for r, c in out:
        ri, ci = int(round(r)), int(round(c))
        if 0 <= ri < h and 0 <= ci < w and dist_to_wall[ri, ci] >= min_clear_px:
            refined.append((r, c))
            continue
        best_d = -1.0
        best_pt = (r, c)
        for dr in range(-10, 11):
            for dc in range(-10, 11):
                nr, nc = ri + dr, ci + dc
                if 0 <= nr < h and 0 <= nc < w and free[nr, nc] > 0:
                    d = float(dist_to_wall[nr, nc])
                    if d >= min_clear_px and d > best_d:
                        best_d = d
                        best_pt = (float(nr), float(nc))
        refined.append(best_pt)
    return refined


def refine_centerline_path(
    raw: list,
    free: np.ndarray,
    dist_to_wall: np.ndarray,
    min_clear_px: float = 1.5,
) -> list:
    """이미 유효한 경로는 그대로 두고, 벗어난 점만 국소 보정."""
    if _score_centerline_candidate(raw, free, dist_to_wall) > -1e8:
        return raw
    return clamp_path_to_corridor(raw, free, dist_to_wall, min_clear_px=min_clear_px)


def estimate_track_perimeter_px(free: np.ndarray) -> float:
    area = float(free.sum())
    if area < 20:
        return 40.0
    return float(2.0 * np.sqrt(np.pi * area))


def _path_radial_stats(path: list) -> tuple[float, float, float]:
    pts = np.array(path, dtype=float)
    cen = pts.mean(axis=0)
    radii = np.linalg.norm(pts - cen, axis=1)
    return (
        float(np.median(radii)),
        float(np.percentile(radii, 90)),
        float(np.max(radii)),
    )


def _spur_excursion_px(path: list) -> float:
    """가까운 점이 경로상 멀리 떨어져 있으면 그 사이 lobe(꼬임/스퍼) 깊이."""
    if len(path) < 20:
        return 0.0
    pts = np.array(path, dtype=float)
    n = len(pts)
    worst = 0.0
    max_span = min(80, n // 3)
    for i in range(n):
        for span in range(10, max_span):
            j = (i + span) % n
            if float(np.linalg.norm(pts[i] - pts[j])) > 3.5:
                continue
            if j > i:
                mid = pts[i : j + 1]
            else:
                mid = np.vstack([pts[i:], pts[: j + 1]])
            if len(mid) < 5:
                continue
            dmax = float(np.max(np.linalg.norm(mid - pts[i], axis=1)))
            if dmax > worst:
                worst = dmax
    return worst


def _score_centerline_candidate(
    path: list,
    free: np.ndarray,
    dist_to_wall: np.ndarray,
) -> float:
    if len(path) < 8:
        return -1e9
    ix = count_self_intersections(path, closed=True)
    if ix > 0:
        return -1e9
    bad_seg = count_segments_crossing_occupied(path, free, closed=True)
    if bad_seg > 0:
        return -1e9
    h, w = free.shape
    clears: list[float] = []
    on_road = 0
    for r, c in path:
        ri, ci = int(round(r)), int(round(c))
        if 0 <= ri < h and 0 <= ci < w and free[ri, ci] > 0:
            on_road += 1
            clears.append(float(dist_to_wall[ri, ci]))
    if on_road < max(8, int(0.85 * len(path))):
        return -1e9
    min_clear = min(clears) if clears else 0.0
    mean_clear = float(np.mean(clears)) if clears else 0.0
    length = path_arc_length(path, closed=True)
    min_len = 0.45 * estimate_track_perimeter_px(free)
    if length < min_len:
        return -1e9

    # 루프클로저 스퍼/꼬임: 긴 경로·바깥 tip·out-and-back lobe 를 강하게 감점
    median_r, p90_r, max_r = _path_radial_stats(path)
    target_len = max(min_len, 2.0 * np.pi * median_r)
    length_term = min(length, target_len * 1.08) * 0.20
    excess_len = max(0.0, length - target_len * 1.08)
    tip_pen = max(0.0, max_r - p90_r) * 18.0
    spur_pen = max(0.0, _spur_excursion_px(path) - 8.0) * 12.0

    return (
        min_clear * 12.0
        + mean_clear * 8.0
        + length_term
        - excess_len * 0.55
        - tip_pen
        - spur_pen
        + len(path) * 0.02
    )


def _cycle_min_clearance(path: list, dist_to_wall: np.ndarray) -> float:
    vals: list[float] = []
    h, w = dist_to_wall.shape
    for r, c in path:
        ri, ci = int(round(r)), int(round(c))
        if 0 <= ri < h and 0 <= ci < w:
            vals.append(float(dist_to_wall[ri, ci]))
    return min(vals) if vals else 0.0


def extract_best_valid_cycle(
    pt_set: set,
    neighbors,
    free: np.ndarray,
    dist_to_wall: np.ndarray,
) -> list:
    """스켈레톤 그래프에서 비주행 침범·자기교차 없는 최적 폐루프."""
    if not pt_set:
        return []
    for cand in (prune_skeleton_tips(pt_set, neighbors), pt_set):
        if len(cand) < 8:
            continue
        path = extract_largest_cycle(cand, neighbors)
        if len(path) < 8:
            continue
        if count_self_intersections(path, closed=True) > 0:
            continue
        if count_segments_crossing_occupied(path, free, closed=True) > 0:
            continue
        return path
    return []


def extract_longest_loop_per_skel_component(
    skel: np.ndarray,
    free: np.ndarray | None = None,
    dist_to_wall: np.ndarray | None = None,
) -> list:
    if skel.sum() == 0:
        return []
    if free is None:
        free = (skel > 0).astype(np.uint8)
    if dist_to_wall is None:
        dist_to_wall = distance_transform_edt(free > 0)
    labeled, n_comp = ndi_label(skel > 0, structure=np.ones((3, 3), dtype=int))
    best_path: list = []
    best_score = -1e18
    for i in range(1, n_comp + 1):
        comp = (labeled == i).astype(np.uint8)
        pt_set, ngb = skeleton_to_graph(comp)
        if not pt_set:
            continue
        pruned = prune_skeleton_tips(pt_set, ngb)
        for cand in (pruned, pt_set):
            if len(cand) < 8:
                continue
            path = extract_best_valid_cycle(cand, ngb, free, dist_to_wall)
            if len(path) < 8:
                continue
            score = (
                _cycle_min_clearance(path, dist_to_wall) * 30.0
                + path_arc_length(path, closed=True) * 0.05
            )
            if score > best_score:
                best_score = score
                best_path = path
    return best_path


def medial_skeleton_free(free: np.ndarray) -> np.ndarray:
    """free 영역 전체 스켈레톤(ring 통로에서 단일 폐곡선에 가까움)."""
    if free.sum() == 0:
        return free
    return skeletonize(free.astype(bool)).astype(np.uint8)


def ridge_medial_thin(free: np.ndarray, use_ridge: bool = True) -> np.ndarray:
    """거리 변환 ridge 위 스켈레톤 — 통로 중앙선만 남김."""
    if free.sum() == 0:
        return free
    dt = distance_transform_edt(free > 0)
    max_dt = maximum_filter(dt, size=5, mode="nearest")
    ridge = (dt > 0) & (dt >= max_dt - 0.5)
    ridge = ridge.astype(np.uint8) * free
    if ridge.sum() < 8:
        return medial_skeleton_free(free)
    sk = skeletonize(ridge.astype(bool)).astype(np.uint8)
    if int(sk.sum()) < 12:
        return medial_skeleton_free(free)
    if not use_ridge:
        return medial_skeleton_free(free)
    return sk


def finalize_centerline_path(
    path: list,
    free: np.ndarray,
    *,
    step_px: float,
    smooth_window: int,
) -> list:
    """리샘플·스무딩 후 self-intersection 이 생기면 스무딩을 단계적으로 줄인다."""
    if len(path) < 3:
        return path
    resampled = resample_closed_polyline_rc(path, step_px)
    if smooth_window <= 0:
        return resampled
    for win in range(smooth_window, -1, -1):
        cand = smooth_ma(resampled, win) if win > 0 else resampled
        if count_self_intersections(cand, closed=True) == 0:
            if count_segments_crossing_occupied(cand, free, closed=True) == 0:
                return cand
    return resampled


def choose_best_centerline(
    free: np.ndarray,
    *,
    use_ridge: bool = True,
    include_full_skeleton: bool = True,
    img_gray: np.ndarray | None = None,
    step_px: float = 1.0,
    smooth_window: int = 3,
) -> tuple[list, str]:
    """여러 방법 후보 중 자기교차·스퍼·비주행 침범 없는 폐루프 선택."""
    dist_to_wall = distance_transform_edt(free > 0)
    candidates: list[tuple[str, list]] = []

    polar = polar_dt_centerline(free, img_gray)
    if len(polar) >= 8:
        candidates.append(("polar_dt", polar))

    mid = midpoint_scan_centerline(free, img_gray)
    if len(mid) >= 8:
        candidates.append(("midpoint_scan", mid))

    if use_ridge:
        skel = ridge_medial_thin(free, use_ridge=True)
        graph_path = extract_longest_loop_per_skel_component(skel, free, dist_to_wall)
        if len(graph_path) >= 8:
            candidates.append(("ridge_graph", graph_path))

    if include_full_skeleton or not use_ridge:
        full_skel = medial_skeleton_free(free)
        full_path = extract_longest_loop_per_skel_component(full_skel, free, dist_to_wall)
        if len(full_path) >= 8:
            candidates.append(("full_skeleton", full_path))

    best_path: list = []
    best_score = -1e18
    best_name = ""
    for name, raw in candidates:
        path = refine_centerline_path(raw, free, dist_to_wall, min_clear_px=1.5)
        # 최종 출력과 동일하게 리샘플·스무딩한 뒤 점수 (스무딩으로 꼬이는 후보 배제)
        finalized = finalize_centerline_path(
            path, free, step_px=step_px, smooth_window=smooth_window
        )
        score = _score_centerline_candidate(finalized, free, dist_to_wall)
        if score > best_score:
            best_score = score
            best_path = finalized
            best_name = name

    if best_path:
        return best_path, best_name
    return [], "empty"


def resample_closed_polyline_rc(points: list, step_px: float) -> list:
    if len(points) < 3 or step_px <= 0:
        return points
    pts = np.array(points, dtype=float)
    n = len(pts)
    seg = np.sqrt(np.sum((np.roll(pts, -1, axis=0) - pts) ** 2, axis=1))
    total = float(seg.sum())
    if total < 1e-9:
        return points
    t = 0.0
    out: list[tuple[float, float]] = []
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    while t < total - 1e-9:
        i = int(np.searchsorted(cum, t, side="right") - 1)
        i = max(0, min(i, n - 1))
        p0, p1 = pts[i], pts[(i + 1) % n]
        seglen = float(np.linalg.norm(p1 - p0)) + 1e-9
        frac = (t - cum[i]) / seglen if cum[i + 1] > cum[i] else 0.0
        pt = (1 - frac) * p0 + frac * p1
        out.append((float(pt[0]), float(pt[1])))
        t += step_px
    return out if len(out) >= 3 else points


def smooth_ma(points: list, window: int) -> list:
    if window < 1 or len(points) < window * 2 + 1:
        return points
    pts = np.array(points, dtype=float)
    n = len(pts)
    out = np.zeros_like(pts)
    w = window
    for i in range(n):
        idx = np.arange(-w, w + 1) + i
        idx = idx % n
        out[i] = pts[idx].mean(axis=0)
    return [(float(out[i, 0]), float(out[i, 1])) for i in range(n)]


def write_csv(path: str, rows: list[tuple[float, float]]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in rows:
            w.writerow([f"{float(x):.12g}", f"{float(y):.12g}"])


def load_map(yaml_path: str, invert_free: bool = False):
    """YAML free_thresh 반영. unknown 구간은 막힌 것으로 보고 free 마스크 생성."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    free_thresh = float(meta.get("free_thresh", 0.196))
    free, resolution, origin_x, origin_y, shape, _mode = load_yaml_map(
        yaml_path,
        invert_free=invert_free,
        free_thresh=free_thresh,
        unknown_as_occupied=True,
        unknown_low=40,
        unknown_high=240,
    )
    return free, resolution, origin_x, origin_y, shape


def resample_polyline_by_arc_length(points: list, step: float, closed: bool = False) -> list:
    """polyline을 호길이 기준 step 간격으로 리샘플링(픽셀). closed=True면 끝→시작 포함."""
    if len(points) <= 1 or step <= 0:
        return points
    pts = np.array(points, dtype=float)
    n = len(pts)
    seg_lens = np.sqrt(np.sum((pts[1:] - pts[:-1]) ** 2, axis=1))
    if closed and n >= 3:
        close_len = np.sqrt(np.sum((pts[0] - pts[-1]) ** 2))
        seg_lens = np.append(seg_lens, close_len)
    cum = np.zeros(len(seg_lens) + 1)
    cum[1:] = np.cumsum(seg_lens)
    total = cum[-1]
    if total < 1e-9:
        return points
    out = []
    t = 0.0
    n_seg = len(seg_lens)
    while t < total - 1e-9:
        i = np.searchsorted(cum, t, side="right") - 1
        i = max(0, min(i, n_seg - 1))
        if cum[i + 1] - cum[i] < 1e-9:
            frac = 0.0
        else:
            frac = (t - cum[i]) / (cum[i + 1] - cum[i])
        if closed and i == n_seg - 1:
            pt = (1 - frac) * pts[-1] + frac * pts[0]
        else:
            pt = (1 - frac) * pts[i] + frac * pts[i + 1]
        out.append((float(pt[0]), float(pt[1])))
        t += step
    if not out:
        out = [tuple(pts[0])]
    return out


def smooth_polyline(points: list, window: int = 5) -> list:
    """열린 폴리라인: 구간별 이동평균(레이싱라인 등에서 사용)."""
    if len(points) < window * 2 + 1 or window < 1:
        return points
    pts = np.array(points, dtype=float)
    n = len(pts)
    out = np.zeros_like(pts)
    for i in range(n):
        lo = max(0, i - window)
        hi = min(n, i + window + 1)
        out[i] = np.mean(pts[lo:hi], axis=0)
    return [tuple(out[i]) for i in range(n)]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Robust centerline CSV from ROS map (any brightness / trinary)."
    )
    default_map = resolve_map_yaml(CFG["map_name"], CFG["map_dir"])
    parser.add_argument(
        "--map",
        default=default_map,
        help=f"Path to map.yaml (default from CFG map_name={CFG['map_name']!r})",
    )
    parser.add_argument(
        "--out",
        default=os.path.abspath(CFG["out_csv"]),
    )
    parser.add_argument(
        "--invert-free",
        default="auto",
        choices=["auto", "0", "1"],
        help="auto=guess, 0=dark road, 1=bright road (typical Cartographer)",
    )
    parser.add_argument("--free-thresh", type=float, default=0.25)
    parser.add_argument("--unknown-low", type=int, default=40)
    parser.add_argument("--unknown-high", type=int, default=240)
    parser.add_argument(
        "--no-unknown-mask",
        action="store_true",
        help="Treat grayscale as binary only (no unknown band)",
    )
    parser.add_argument("--close-iters", type=int, default=1)
    parser.add_argument("--open-iters", type=int, default=1)
    parser.add_argument(
        "--wall-dilate-iters",
        type=int,
        default=1,
        help="비주행 팽창 횟수(얇은 벽 연결). 끊김 보정 끄려면 0.",
    )
    parser.add_argument(
        "--wall-gap-close",
        type=int,
        default=6,
        metavar="R",
        help="끊긴 벽 메우기 (2R+1) closing 반경(px). 0=비활성.",
    )
    parser.add_argument(
        "--wall-gap-close-iters",
        type=int,
        default=2,
        help="벽 closing 반복 횟수.",
    )
    parser.add_argument(
        "--no-keep-largest-free",
        action="store_true",
        help="보수 후 모든 주행 연결요소 유지(기본은 최대 요소만).",
    )
    parser.add_argument("--min-dt-px", type=float, default=1.0, help="Min clearance from wall (px)")
    parser.add_argument(
        "--max-dt-px",
        type=float,
        default=0.0,
        help="Max clearance (0=disable). Narrow corridor maps can try 80~120",
    )
    parser.add_argument(
        "--resample-step-m",
        type=float,
        default=0.05,
        help="Resample spacing in meters (map resolution applied)",
    )
    parser.add_argument(
        "--use-ridge",
        action="store_true",
        help="(레거시) ridge 외 full skeleton 후보도 추가",
    )
    parser.add_argument(
        "--no-ridge",
        action="store_true",
        help="ridge 대신 free 전체 skeleton만 사용",
    )
    parser.add_argument("--smooth-window", type=int, default=3, help="폐곡선 이동평균(0=끔)")
    args = parser.parse_args()

    if args.invert_free == "auto":
        inv = None
    elif args.invert_free == "1":
        inv = True
    else:
        inv = False

    unknown_occ = not args.no_unknown_mask

    print(f"Map: {os.path.abspath(args.map)}")
    with open(args.map, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img_path = _resolve_map_image_path(args.map, str(meta["image"]))
    img_gray = np.array(Image.open(img_path).convert("L"))
    if int(meta.get("negate", 0)):
        img_gray = 255 - img_gray

    free, resolution, ox, oy, (height, width), mode = load_yaml_map(
        args.map,
        invert_free=inv,
        free_thresh=args.free_thresh,
        unknown_as_occupied=unknown_occ,
        unknown_low=args.unknown_low,
        unknown_high=args.unknown_high,
    )
    print(
        f"  mode={mode}, free_pixels={int(free.sum())}, image={height}x{width}, res={resolution}"
    )

    free = morph_cleanup(free, args.close_iters, args.open_iters)
    print(f"  after morph: free_pixels={int(free.sum())}")

    free = repair_broken_wall_barriers(
        free,
        dilate_iters=args.wall_dilate_iters,
        close_radius=args.wall_gap_close,
        close_iters=args.wall_gap_close_iters,
        keep_largest_free=not args.no_keep_largest_free,
    )
    print(f"  after wall-gap repair: free_pixels={int(free.sum())}")

    bridge_px = max(4, int(estimate_min_corridor_px(free) * 0.55))
    free_bridged = remove_thin_bridges(free, bridge_px)
    if int(free_bridged.sum()) >= max(50, int(0.35 * free.sum())):
        free = free_bridged
        print(f"  after thin-bridge removal ({bridge_px}px): free_pixels={int(free.sum())}")

    if args.min_dt_px > 0 or args.max_dt_px > 0:
        free2 = corridor_from_distance(free, args.min_dt_px, args.max_dt_px)
        if int(free2.sum()) >= 20:
            free = free2
            print(f"  after corridor(dt): free_pixels={int(free.sum())}")

    skel = ridge_medial_thin(free, use_ridge=not args.no_ridge)
    print(f"  skeleton pixels={int(skel.sum())}")

    step_px = max(0.5, args.resample_step_m / resolution)
    path_rc, method = choose_best_centerline(
        free,
        use_ridge=not args.no_ridge,
        include_full_skeleton=True,
        img_gray=img_gray,
        step_px=step_px,
        smooth_window=args.smooth_window,
    )
    if len(path_rc) < 3:
        print("ERROR: could not extract a closed loop.", file=sys.stderr)
        return 1
    print(
        f"  chosen method={method}, points={len(path_rc)}, "
        f"self_ix={count_self_intersections(path_rc)}, "
        f"spur={_spur_excursion_px(path_rc):.1f}px"
    )

    world = [
        pixel_to_world(r, c, height, resolution, ox, oy) for r, c in path_rc
    ]
    out_path = os.path.abspath(args.out)
    write_csv(out_path, world)
    print(f"Wrote {len(world)} points → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
