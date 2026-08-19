#!/usr/bin/env python3
"""
맵 YAML+이미지 → 벽과 벽 사이 중앙선(centerline) CSV(x,y).

파이프라인:
  1. ROS 맵 규약으로 free 마스크 생성 (unknown = 벽)
  2. 최대 연결요소 + 모폴로지 정리
  3. skeletonize → 가지(tip) 제거 → 그래프
  4. 인필드 섬을 감싸는 폐루프를 그래프에서 추출
  5. 등간격 리샘플 + "클리어런스 유지 리지 스무딩"
     - 법선 방향 거리변환(DT) 최대점으로 당김 = 벽-벽 중앙
     - 라플라시안 스무딩으로 꺾임 제거
     - 매 갱신마다 최소 클리어런스 검사 → 벽을 뚫지 못함

출력 좌표: map 프레임 x=origin_x+col*res, y=origin_y+(H-1-row)*res

사용:
  python3 extract_centerline_from_map.py
  python3 extract_centerline_from_map.py --map /path/map.yaml --out ../config/centerline.csv
"""
from __future__ import annotations

import argparse
import csv
import heapq
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image

try:
    from scipy.ndimage import (
        binary_closing,
        binary_fill_holes,
        binary_opening,
        distance_transform_edt,
        gaussian_filter,
        label as ndi_label,
    )
except ImportError as exc:  # pragma: no cover
    print("Missing scipy:", exc, file=sys.stderr)
    sys.exit(1)

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

# 차량 치수는 런타임 노드와 같은 정의를 쓴다 (ROS 의존 없는 순수 모듈).
_pkg_dir = os.path.join(os.path.dirname(_script_dir), "path_following")
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

import vehicle_geometry as vg  # noqa: E402

from speed_profile import (  # noqa: E402
    UNMEASURED_WARNING,
    VEHICLE,
    add_speed_args,
    lap_time,
    profile_kwargs_from_args,
    report_profile,
    speed_profile,
    write_csv_xyv,
)

try:
    from skimage.morphology import skeletonize
except ImportError as exc:  # pragma: no cover
    print("Install scikit-image: pip install scikit-image", exc, file=sys.stderr)
    sys.exit(1)


# ============================================================
# USER TUNING — 맵 바꿀 때 여기만 수정
# ============================================================
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..", ".."))
_DEFAULT_MAP_DIR = os.path.join(_WS_ROOT, "maps")

# ROS 맵 규약의 unknown 픽셀값 205 에 해당하는 점유도 = 1 - 205/255.
# free_thresh 가 이 값 이상이면 unknown 이 free 로 넘어온다.
UNKNOWN_OCC = 1.0 - 205.0 / 255.0

CFG = {
    # maps/ 아래 yaml 파일명만 (절대경로 넣어도 됨)
    "map_name": "cartographer_map_20260820_014643_rosmap.yaml",
    "map_dir": _DEFAULT_MAP_DIR,
    "out_csv": os.path.join(_SCRIPT_DIR, "..", "config", "centerline.csv"),
}

_NEIGHBORS_8 = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
]


# ============================================================
# 경로 / 맵 로딩
# ============================================================
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
        raise FileNotFoundError(f"map yaml not found: {cand} (map_name={name!r})")
    return cand


def _resolve_map_image_path(yaml_path: str, image_field: str) -> str:
    """YAML image 가 다른 머신 절대경로여도 yaml 옆 이미지를 우선 사용."""
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    if os.path.isabs(image_field) and os.path.isfile(image_field):
        return image_field
    rel = os.path.join(yaml_dir, image_field)
    if os.path.isfile(rel):
        return rel
    same_name = os.path.join(yaml_dir, os.path.basename(image_field))
    if os.path.isfile(same_name):
        return same_name
    raise FileNotFoundError(f"map image not found for {yaml_path}: {image_field}")


def load_map(yaml_path: str, invert_free: bool = False):
    """ROS 맵 규약 free 마스크.

    occ = (255-p)/255  (negate=1 이면 p/255)
    free  : occ < free_thresh
    unknown/occupied : 벽으로 취급 (invert_free=True 면 어두운 쪽을 도로로)

    반환: (free[H,W] uint8, resolution, origin_x, origin_y, (H, W))
    """
    with open(yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    img_path = _resolve_map_image_path(yaml_path, str(meta["image"]))
    gray = np.array(Image.open(img_path).convert("L")).astype(np.float32)

    resolution = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    origin_x, origin_y = float(origin[0]), float(origin[1])
    free_thresh = float(meta.get("free_thresh", 0.196))
    # unknown 픽셀(205)의 점유도가 정확히 1 - 205/255 = 0.196 이라, 임계가 이보다
    # 크면 미탐색 영역이 전부 도로로 잡힌다. 그러면 트랙 밖으로 도로가 새어
    # 나가 인필드 섬이 사라지고 폐루프를 못 찾는다. map_saver_cli 가 0.25 를
    # 써 놓는 일이 있어서 여기서 막는다.
    if free_thresh > UNKNOWN_OCC + 1e-9:
        print(
            f"  WARNING: free_thresh={free_thresh} 는 unknown(205) 을 도로로 셉니다. "
            f"{UNKNOWN_OCC} 로 낮춰서 진행합니다 (맵 YAML 을 고치는 게 좋습니다)."
        )
        free_thresh = UNKNOWN_OCC

    occ = gray / 255.0 if int(meta.get("negate", 0)) else (255.0 - gray) / 255.0
    free = occ < free_thresh
    if invert_free:
        free = ~free

    h, w = free.shape
    return free.astype(np.uint8), resolution, origin_x, origin_y, (h, w)


def pixel_to_world(row, col, height, resolution, origin_x, origin_y):
    return (
        origin_x + float(col) * resolution,
        origin_y + (float(height) - 1.0 - float(row)) * resolution,
    )


def world_to_pixel(x, y, height, resolution, origin_x, origin_y):
    return (
        (float(height) - 1.0) - (float(y) - origin_y) / resolution,
        (float(x) - origin_x) / resolution,
    )


def write_csv(path: str, rows) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x, y in rows:
            writer.writerow([f"{float(x):.6f}", f"{float(y):.6f}"])


# ============================================================
# 마스크 정리
# ============================================================
def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    labeled, n = ndi_label(mask > 0, structure=np.ones((3, 3), dtype=int))
    if n <= 1:
        return (mask > 0).astype(np.uint8)
    sizes = np.bincount(labeled.ravel())
    sizes[0] = 0
    return (labeled == int(np.argmax(sizes))).astype(np.uint8)


def clean_free_mask(free: np.ndarray, close_iters: int, open_iters: int) -> np.ndarray:
    """스캔 노이즈 정리 후 주행 가능한 최대 연결영역만 남긴다."""
    struct = np.ones((3, 3), dtype=bool)
    out = keep_largest_component(free)
    if close_iters > 0:
        out = binary_closing(out.astype(bool), structure=struct, iterations=close_iters)
    else:
        out = out.astype(bool)
    if open_iters > 0:
        out = binary_opening(out, structure=struct, iterations=open_iters)
    return keep_largest_component(out.astype(np.uint8))


def largest_island_centroid(free: np.ndarray):
    """free 안의 최대 구멍(인필드 섬) 무게중심. 없으면 None."""
    filled = binary_fill_holes(free > 0)
    holes = filled & (free == 0)
    if not holes.any():
        return None
    labeled, n = ndi_label(holes, structure=np.ones((3, 3), dtype=int))
    if n < 1:
        return None
    sizes = np.bincount(labeled.ravel())
    sizes[0] = 0
    rows, cols = np.where(labeled == int(np.argmax(sizes)))
    return (float(rows.mean()), float(cols.mean()))


# ============================================================
# 스켈레톤 → 폐루프
# ============================================================
def skeleton_nodes(free: np.ndarray) -> set:
    """스켈레톤에서 가지(tip)를 모두 쳐낸 노드 집합. 링이면 사이클만 남는다."""
    skel = skeletonize(free.astype(bool))
    nodes = set(map(tuple, np.argwhere(skel)))

    def degree(p, pool):
        r, c = p
        return sum(1 for dr, dc in _NEIGHBORS_8 if (r + dr, c + dc) in pool)

    while True:
        tips = [p for p in nodes if degree(p, nodes) <= 1]
        if not tips:
            break
        for p in tips:
            nodes.discard(p)
    return nodes


def _adjacency(nodes: set) -> dict:
    adj = {}
    for r, c in nodes:
        adj[(r, c)] = [
            (r + dr, c + dc)
            for dr, dc in _NEIGHBORS_8
            if (r + dr, c + dc) in nodes
        ]
    return adj


def _shortest_path(adj: dict, src, dst, banned_edge):
    """banned_edge 를 뺀 그래프에서 src→dst 최단경로 (Dijkstra)."""
    dist = {src: 0.0}
    prev = {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, math.inf) + 1e-12:
            continue
        if u == dst:
            break
        for v in adj[u]:
            if (u, v) == banned_edge or (v, u) == banned_edge:
                continue
            nd = d + math.hypot(v[0] - u[0], v[1] - u[1])
            if nd < dist.get(v, math.inf) - 1e-12:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None, 0.0
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    path.reverse()
    return path, dist[dst]


def polygon_contains(polygon, point) -> bool:
    """(row, col) 폴리곤 내부 판정 (ray casting)."""
    py, px = point
    inside = False
    n = len(polygon)
    for i in range(n):
        y1, x1 = polygon[i]
        y2, x2 = polygon[(i + 1) % n]
        if (x1 > px) != (x2 > px):
            y_at = y1 + (px - x1) * (y2 - y1) / (x2 - x1 + 1e-18)
            if y_at > py:
                inside = not inside
    return inside


def extract_ring_cycle(free: np.ndarray, island_rc, seed_count: int = 24) -> list:
    """인필드 섬을 감싸는 가장 긴 스켈레톤 폐루프.

    각 시드에서 간선 하나를 끊고 최단경로로 되돌아오는 기본 사이클을 만든 뒤,
    섬을 감싸는 것 중 가장 긴 것을 고른다.
    """
    nodes = skeleton_nodes(free)
    if len(nodes) < 16:
        return []
    adj = _adjacency(nodes)
    ordered = sorted(nodes)
    stride = max(1, len(ordered) // max(1, seed_count))

    best_cycle: list = []
    best_len = 0.0
    for k in range(0, len(ordered), stride):
        a = ordered[k]
        if not adj[a]:
            continue
        b = adj[a][0]
        path, plen = _shortest_path(adj, a, b, (a, b))
        if path is None or len(path) < 20:
            continue
        total = plen + math.hypot(b[0] - a[0], b[1] - a[1])
        if island_rc is not None and not polygon_contains(path, island_rc):
            continue
        if total > best_len:
            best_len = total
            best_cycle = path

    if best_cycle:
        return best_cycle

    # 섬이 없거나 포함 판정 실패 → 가장 긴 사이클
    for k in range(0, len(ordered), stride):
        a = ordered[k]
        if not adj[a]:
            continue
        b = adj[a][0]
        path, plen = _shortest_path(adj, a, b, (a, b))
        if path is None or len(path) < 20:
            continue
        total = plen + math.hypot(b[0] - a[0], b[1] - a[1])
        if total > best_len:
            best_len = total
            best_cycle = path
    return best_cycle


# ============================================================
# 폴리라인 유틸 (row, col 기준)
# ============================================================
def bilinear_sample(field: np.ndarray, rows, cols):
    """실수 좌표에서 스칼라장 보간. rows/cols 는 배열 또는 스칼라."""
    h, w = field.shape
    r = np.clip(np.asarray(rows, dtype=float), 0.0, h - 1.001)
    c = np.clip(np.asarray(cols, dtype=float), 0.0, w - 1.001)
    r0 = r.astype(int)
    c0 = c.astype(int)
    fr = r - r0
    fc = c - c0
    return (
        field[r0, c0] * (1 - fr) * (1 - fc)
        + field[r0 + 1, c0] * fr * (1 - fc)
        + field[r0, c0 + 1] * (1 - fr) * fc
        + field[r0 + 1, c0 + 1] * fr * fc
    )


def signed_area_world(world) -> float:
    """월드 xy 폐곡선의 부호면적 [m²]. 양수 = 반시계(CCW)."""
    w = np.asarray(world, dtype=float)
    x, y = w[:, 0], w[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def enforce_direction(centerline, direction, height, resolution, ox, oy):
    """월드 진행방향이 `direction` 이 되도록 픽셀 폐루프의 순서를 맞춘다."""
    world = [pixel_to_world(r, c, height, resolution, ox, oy) for r, c in centerline]
    area = signed_area_world(world)
    want_ccw = direction == "ccw"
    if (area > 0.0) == want_ccw:
        print(f"  direction={direction} (부호면적 {area:+.1f} m², 그대로)")
        return centerline
    print(f"  direction={direction} (부호면적 {area:+.1f} m² → 순서 뒤집음)")
    return np.asarray(centerline)[::-1].copy()


def resample_closed(points, step: float) -> np.ndarray:
    """폐곡선을 등간격(step)으로 리샘플."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3 or step <= 0:
        return pts
    seg = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    total = float(seg.sum())
    if total < 1e-9:
        return pts
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    count = max(16, int(round(total / step)))
    targets = np.linspace(0.0, total, count, endpoint=False)
    idx = np.clip(np.searchsorted(cum, targets, side="right") - 1, 0, len(pts) - 1)
    frac = (targets - cum[idx]) / (seg[idx] + 1e-12)
    nxt = (idx + 1) % len(pts)
    return pts[idx] * (1.0 - frac[:, None]) + pts[nxt] * frac[:, None]


def closed_normals(points: np.ndarray) -> np.ndarray:
    """폐곡선 각 점의 단위 법선 (row, col)."""
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True) + 1e-12
    return np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)


def turn_angles_deg(points) -> np.ndarray:
    """각 정점의 헤딩 변화량 |Δθ| [deg]."""
    pts = np.asarray(points, dtype=float)
    prev = np.roll(pts, 1, axis=0)
    nxt = np.roll(pts, -1, axis=0)
    a0 = np.arctan2(pts[:, 0] - prev[:, 0], pts[:, 1] - prev[:, 1])
    a1 = np.arctan2(nxt[:, 0] - pts[:, 0], nxt[:, 1] - pts[:, 1])
    diff = (a1 - a0 + np.pi) % (2.0 * np.pi) - np.pi
    return np.abs(np.degrees(diff))


def path_length(points) -> float:
    pts = np.asarray(points, dtype=float)
    return float(np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1).sum())


def segment_in_free(p0, p1, free: np.ndarray) -> bool:
    h, w = free.shape
    dr = p1[0] - p0[0]
    dc = p1[1] - p0[1]
    steps = max(2, int(math.hypot(dr, dc) * 2.0))
    for t in np.linspace(0.0, 1.0, steps):
        rr = int(round(p0[0] + t * dr))
        cc = int(round(p0[1] + t * dc))
        if not (0 <= rr < h and 0 <= cc < w) or free[rr, cc] == 0:
            return False
    return True


def count_wall_crossings(points, free: np.ndarray) -> int:
    pts = list(points)
    n = len(pts)
    return sum(
        1 for i in range(n) if not segment_in_free(pts[i], pts[(i + 1) % n], free)
    )


def _segments_intersect(a, b, c, d) -> bool:
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    d1 = cross(c, d, a)
    d2 = cross(c, d, b)
    d3 = cross(a, b, c)
    d4 = cross(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def count_self_intersections(points) -> int:
    pts = [tuple(map(float, p)) for p in points]
    n = len(pts)
    if n < 5:
        return 0
    total = 0
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            c, d = pts[j], pts[(j + 1) % n]
            if _segments_intersect(a, b, c, d):
                total += 1
    return total


# ============================================================
# 핵심: 클리어런스 유지 리지 스무딩
# ============================================================
def smooth_on_ridge(
    points,
    free: np.ndarray,
    *,
    step_px: float,
    min_clear_px: float,
    iters: int = 120,
    search_px: float = 6.0,
    final_iters: int = 400,
    dt_blur_px: float = 2.0,
):
    """벽-벽 중앙을 지키면서 부드럽게 만든다.

    각 반복에서
      1) 라플라시안 스무딩으로 꺾임을 줄이고
      2) 법선 방향 DT 최대점(=통로 한가운데)으로 끌어당긴다.
    반복이 진행될수록 중앙 당김을 줄이고 스무딩을 키워 잔떨림을 없앤다.
    새 위치의 클리어런스가 min_clear_px 미만이면 이동을 버려 벽을 넘지 못한다.
    """
    dist = distance_transform_edt(free > 0).astype(float)
    ridge = gaussian_filter(dist, dt_blur_px) if dt_blur_px > 0 else dist

    pts = resample_closed(points, step_px)
    offsets = np.linspace(-search_px, search_px, int(search_px * 4) + 1)
    d_off = offsets[1] - offsets[0]

    for it in range(max(1, iters)):
        frac = it / max(1, iters - 1)
        w_smooth = 0.30 + 0.35 * frac
        w_center = 0.55 * (1.0 - 0.85 * frac)

        normals = closed_normals(pts)
        prev = np.roll(pts, 1, axis=0)
        nxt = np.roll(pts, -1, axis=0)
        base = pts + w_smooth * (0.5 * (prev + nxt) - pts)

        # 법선 방향으로 DT 프로파일 샘플 → 최대점 (서브픽셀 보간)
        rr = base[:, 0][:, None] + offsets[None, :] * normals[:, 0][:, None]
        cc = base[:, 1][:, None] + offsets[None, :] * normals[:, 1][:, None]
        profile = bilinear_sample(ridge, rr, cc)
        best = np.argmax(profile, axis=1)
        rows = np.arange(len(pts))
        shift = offsets[best].astype(float)
        interior = (best > 0) & (best < len(offsets) - 1)
        if interior.any():
            y0 = profile[rows[interior], best[interior] - 1]
            y1 = profile[rows[interior], best[interior]]
            y2 = profile[rows[interior], best[interior] + 1]
            den = y0 - 2.0 * y1 + y2
            safe = np.abs(den) > 1e-9
            adj = np.zeros_like(den)
            adj[safe] = 0.5 * (y0[safe] - y2[safe]) / den[safe] * d_off
            shift[interior] += adj

        cand = base + (w_center * shift)[:, None] * normals
        ok = bilinear_sample(dist, cand[:, 0], cand[:, 1]) >= min_clear_px
        base_ok = bilinear_sample(dist, base[:, 0], base[:, 1]) >= min_clear_px
        moved = np.where(ok[:, None], cand, np.where(base_ok[:, None], base, pts))
        pts = resample_closed(moved, step_px)

    # 마무리: 중앙 당김 없이 제약 스무딩만 (잔떨림 제거)
    for _ in range(max(0, final_iters)):
        prev = np.roll(pts, 1, axis=0)
        nxt = np.roll(pts, -1, axis=0)
        cand = pts + 0.25 * (0.5 * (prev + nxt) - pts)
        ok = bilinear_sample(dist, cand[:, 0], cand[:, 1]) >= min_clear_px
        pts = np.where(ok[:, None], cand, pts)

    return resample_closed(pts, step_px)


def relax_curvature(
    points,
    free: np.ndarray,
    *,
    step_px: float,
    min_clear_px: float,
    min_radius_px: float,
    iters: int = 600,
    spread: int = 4,
):
    """최소 회전반경을 못 지키는 구간만 국소적으로 펴준다.

    차량이 못 도는 코너는 추종 자체가 불가능하므로, 곡률이 한계를 넘는 점
    주변에만 강한 스무딩을 걸어 반경을 키운다. 여기서도 클리어런스 미달
    이동은 버리므로 벽을 넘지 않는다.
    """
    dist = distance_transform_edt(free > 0).astype(float)
    pts = resample_closed(points, step_px)
    max_turn_deg = math.degrees(step_px / max(1e-6, min_radius_px))

    for _ in range(max(1, iters)):
        hot = turn_angles_deg(pts) > max_turn_deg
        if not hot.any():
            break
        weight = np.zeros(len(pts))
        for k in range(-spread, spread + 1):
            weight = np.maximum(weight, np.roll(hot.astype(float), k))
        prev = np.roll(pts, 1, axis=0)
        nxt = np.roll(pts, -1, axis=0)
        cand = pts + (0.45 * weight)[:, None] * (0.5 * (prev + nxt) - pts)
        ok = bilinear_sample(dist, cand[:, 0], cand[:, 1]) >= min_clear_px
        pts = resample_closed(np.where(ok[:, None], cand, pts), step_px)
    return pts


def path_center_ratio(points, free: np.ndarray, local_px: int = 8) -> float:
    """클리어런스 / 국소 최대 클리어런스 평균. 1에 가까울수록 통로 한가운데."""
    dist = distance_transform_edt(free > 0)
    h, w = free.shape
    ratios = []
    for r, c in points:
        ri, ci = int(round(r)), int(round(c))
        if not (0 <= ri < h and 0 <= ci < w) or free[ri, ci] == 0:
            ratios.append(0.0)
            continue
        d = float(dist[ri, ci])
        window = dist[
            max(0, ri - local_px) : ri + local_px + 1,
            max(0, ci - local_px) : ci + local_px + 1,
        ]
        dmax = float(window.max()) if window.size else d
        ratios.append(d / dmax if dmax > 1e-9 else 0.0)
    return float(np.mean(ratios)) if ratios else 0.0


# ============================================================
# main
# ============================================================
def build_centerline(
    free_raw: np.ndarray,
    resolution: float,
    *,
    close_iters: int,
    open_iters: int,
    step_m: float,
    min_clear_m: float,
    smooth_iters: int,
    min_radius_m: float = 0.0,
    verbose: bool = True,
):
    """free 마스크 → (centerline_rc, cleaned_free). 실패 시 (빈 리스트, free)."""
    free = clean_free_mask(free_raw, close_iters, open_iters)
    if verbose:
        print(f"  free pixels: raw={int(free_raw.sum())} cleaned={int(free.sum())}")

    island = largest_island_centroid(free)
    if verbose:
        print(f"  infield island centroid(rc)={island}")

    cycle = extract_ring_cycle(free, island)
    if len(cycle) < 16:
        return [], free
    if verbose:
        print(
            f"  skeleton cycle: {len(cycle)} px, "
            f"length={path_length(cycle) * resolution:.2f} m"
        )

    step_px = max(0.5, step_m / resolution)
    min_clear_px = max(1.0, min_clear_m / resolution)
    centerline = smooth_on_ridge(
        cycle,
        free,
        step_px=step_px,
        min_clear_px=min_clear_px,
        iters=smooth_iters,
    )
    if min_radius_m > 0.0:
        centerline = relax_curvature(
            centerline,
            free,
            step_px=step_px,
            min_clear_px=min_clear_px,
            min_radius_px=min_radius_m / resolution,
        )
    return centerline, free


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROS 맵에서 벽-벽 중앙선 CSV 추출 (skeleton loop + ridge smoothing)"
    )
    default_map = resolve_map_yaml(CFG["map_name"], CFG["map_dir"])
    parser.add_argument("--map", default=default_map, help="map.yaml 경로")
    parser.add_argument("--out", default=os.path.abspath(CFG["out_csv"]), help="출력 CSV")
    parser.add_argument(
        "--invert-free",
        action="store_true",
        help="어두운 픽셀을 도로로 해석 (기본은 ROS 규약대로 밝은 쪽이 도로)",
    )
    parser.add_argument("--close-iters", type=int, default=2, help="free closing 반복")
    parser.add_argument("--open-iters", type=int, default=1, help="free opening 반복")
    parser.add_argument(
        "--resample-step-m", type=float, default=0.05, help="웨이포인트 간격 [m]"
    )
    parser.add_argument(
        "--min-clear-m",
        type=float,
        # 실측 반폭 + 5 cm. 값은 이전 기본값 0.20 과 같다 — 여기를 올리면 좁은
        # 구간에서 폐루프를 못 찾는 일이 생기므로 건드리지 않는다.
        # 센터라인은 레이싱라인의 시드이고, 코너 스윕폭까지 감안한 진짜 여유는
        # generate_raceline_from_centerline.py 가 곡률별로 강제한다.
        default=round(vg.HALF_WIDTH_M + 0.05, 3),
        help="벽에서 유지할 최소 거리 [m]. 이보다 가까워지는 이동은 버림",
    )
    parser.add_argument(
        "--smooth-iters", type=int, default=120, help="리지 스무딩 반복 횟수"
    )
    parser.add_argument(
        "--direction",
        choices=("ccw", "cw"),
        default="ccw",
        help="주행 진행방향(월드 xy 기준 회전방향). 스켈레톤 사이클은 방향이 "
        "임의로 나오므로 여기서 강제한다",
    )
    parser.add_argument(
        "--min-radius-m",
        type=float,
        default=0.9,
        help="추종 가능한 최소 회전반경 [m]. 이보다 급한 코너만 국소적으로 편다. 0=비활성",
    )
    # --- 속도 프로파일 (CSV 3번째 열). 기본값은 speed_profile.VEHICLE ---
    add_speed_args(parser)
    args = parser.parse_args()

    print(f"Map: {os.path.abspath(args.map)}")
    free_raw, resolution, ox, oy, (height, width) = load_map(
        args.map, invert_free=args.invert_free
    )
    print(f"  image={height}x{width}, res={resolution}, origin=({ox}, {oy})")

    centerline, free = build_centerline(
        free_raw,
        resolution,
        close_iters=args.close_iters,
        open_iters=args.open_iters,
        step_m=args.resample_step_m,
        min_clear_m=args.min_clear_m,
        smooth_iters=args.smooth_iters,
        min_radius_m=args.min_radius_m,
    )
    if len(centerline) < 16:
        print("ERROR: 폐루프 중앙선을 찾지 못했습니다.", file=sys.stderr)
        return 1

    turns = turn_angles_deg(centerline)
    dist = distance_transform_edt(free > 0)
    clear_m = bilinear_sample(dist, centerline[:, 0], centerline[:, 1]) * resolution
    wall_x = count_wall_crossings(centerline, free)
    self_ix = count_self_intersections(centerline)
    print(
        f"  centerline: {len(centerline)} pts, "
        f"length={path_length(centerline) * resolution:.2f} m"
    )
    print(
        f"  turn |Δθ| max={turns.max():.2f}° p99={np.percentile(turns, 99):.2f}° "
        f"mean={turns.mean():.2f}°"
    )
    print(
        f"  clearance min={clear_m.min():.3f} m median={np.median(clear_m):.3f} m, "
        f"center_ratio={path_center_ratio(centerline, free):.3f}"
    )
    # 센터라인을 그대로 주행할 수도 있으므로(레이싱라인 fallback), 차체가
    # 실제로 들어가는지 알려준다. 여기서 걸리면 그 구간은 트랙 자체가 좁다는
    # 뜻이라 스크립트로 해결되지 않는다 — 통과 속도를 낮춰야 한다.
    need = vg.HALF_WIDTH_M + 0.05
    tight = int(np.count_nonzero(clear_m < need - 1e-6))
    if tight:
        print(
            f"  NOTE: {tight}/{len(centerline)} 점이 차체 반폭+5cm"
            f"({need:.2f} m) 보다 벽에 가깝습니다 — 그 구간은 트랙이 좁습니다."
        )
    print(f"  wall_crossings={wall_x}, self_intersections={self_ix}")
    if wall_x or self_ix:
        print("WARNING: 경로가 벽을 지나거나 자기교차합니다.", file=sys.stderr)

    # 스켈레톤 폐루프는 어느 쪽으로 감기는지가 시드 노드와 인접리스트 순서에
    # 달려 있어서, 같은 트랙을 다시 뽑아도 방향이 뒤집힐 수 있다. 뒤집히면
    # 경로추종이 헤딩오차 180° 를 보고 즉시 풀락을 때린다. 여기서 못 박는다.
    # 속도 프로파일보다 먼저 뒤집어야 감속/가속 패스가 진행방향에 맞는다.
    centerline = enforce_direction(centerline, args.direction, height, resolution, ox, oy)
    world = [
        pixel_to_world(r, c, height, resolution, ox, oy) for r, c in centerline
    ]
    out_path = os.path.abspath(args.out)
    if args.speed:
        kwargs = profile_kwargs_from_args(args, centerline, resolution)
        v, _, ds = speed_profile(centerline, resolution, **kwargs)
        report_profile(v, args, kwargs["scale"])
        print(f"  est. lap={lap_time(v, ds):.2f} s")
        write_csv_xyv(out_path, [(x, y, vi) for (x, y), vi in zip(world, v)])
        print(f"Wrote {len(world)} points (x,y,v) → {out_path}")
        if not VEHICLE["measured"]:
            print(UNMEASURED_WARNING, file=sys.stderr)
    else:
        write_csv(out_path, world)
        print(f"Wrote {len(world)} points (x,y) → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
