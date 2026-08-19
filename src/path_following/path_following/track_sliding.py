"""폐곡선 CSV 상의 투영 + 슬라이딩 윈도우 (local_planner · stanley 공용)."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np

_DEFAULT_CSV_NAMES = ("raceline.csv", "centerline.csv")

# ============================================================
# 주행 라인 기본값 — local_planner / stanley 공통
#   "raceline"   : config/raceline.csv (Out-In-Out 레이싱 라인)
#   "centerline" : config/centerline.csv (벽-벽 중앙)
#   "auto"       : raceline 이 있으면 raceline, 없으면 centerline
# 일회성 전환은 런치 인자를 쓰는 게 편하다: `track:=centerline`
# ============================================================
DEFAULT_TRACK = "raceline"

# ============================================================
# 주행 방향 — local_planner / stanley 공통
#   True  : CSV 를 역순으로 (현재 raceline/centerline 이 이쪽)
#   False : CSV 저장 순서 그대로
# 두 노드가 달라지면 stanley 는 정방향으로 달리는데 플래너의 Frenet s 는
# 역방향이 되어, 선감속·곡률 예측·rejoin 이 전부 차 뒤쪽을 본다. 그래서
# 개별 노드에 값을 박지 말고 반드시 여기 하나만 고친다.
# 값이 틀리면 stanley 기동 직후 hdg_err 가 ~180° 로 뜬다.
# ============================================================
DEFAULT_REVERSE_TRACK = False

_TRACK_FILES = {
    "raceline": ("raceline.csv",),
    "centerline": ("centerline.csv",),
    "auto": _DEFAULT_CSV_NAMES,
}


def _config_roots() -> list[Path]:
    """설치본(share) → 소스 트리 순서로 config 디렉터리 후보."""
    roots: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        roots.append(Path(get_package_share_directory("path_following")) / "config")
    except Exception:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        if parent.name == "path_following" and (parent / "package.xml").is_file():
            roots.append(parent / "config")
            break

    out: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        root = root.resolve()
        if root not in seen:
            seen.add(root)
            out.append(root)
    return out


def resolve_csv_path(csv_param: str, track: str = "") -> str:
    """주행 라인 CSV 경로 결정.

    우선순위: csv_path 파라미터(절대경로) > track 이름 > DEFAULT_TRACK.
    track 은 "raceline" | "centerline" | "auto".
    """
    p = (csv_param or "").strip()
    if p:
        return p

    name = (track or "").strip().lower() or DEFAULT_TRACK
    if name.endswith(".csv"):  # track 에 파일명을 바로 넣은 경우
        wanted = (name,)
    elif name in _TRACK_FILES:
        wanted = _TRACK_FILES[name]
    else:
        raise ValueError(
            f"알 수 없는 track={track!r}. "
            f"{'|'.join(_TRACK_FILES)} 또는 *.csv 파일명을 쓰세요."
        )

    for root in _config_roots():
        for fname in wanted:
            cand = root / fname
            if cand.is_file():
                return str(cand)

    raise FileNotFoundError(
        f"path_following/config/ 에서 {', '.join(wanted)} 을 찾지 못했습니다 "
        f"(track={name}). scripts 로 생성하거나 config/ 에 CSV 를 넣은 뒤 "
        "`colcon build --packages-select path_following` 하세요."
    )


def param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


def load_csv_xyv(path: str):
    """CSV → (points, speeds).

    3번째 열이 있으면 웨이포인트별 목표 속도 [m/s] 로 읽는다.
    없으면 speeds 는 None (구형 x,y CSV 하위호환).
    """
    pts: List[Tuple[float, float]] = []
    speeds: List[float] = []
    have_speed = True
    with open(path, "r", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))

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
        except ValueError:
            if i == start and (
                "x" in (r[0] + r[1]).lower() or "m" in (r[0] + r[1]).lower()
            ):
                start = i + 1
            continue
        pts.append((x, y))
        v = float("nan")
        if len(r) >= 3:
            try:
                v = float(r[2].strip())
            except ValueError:
                v = float("nan")
        if v != v or v < 0.0:  # NaN 또는 음수 → 속도 열 없음으로 취급
            have_speed = False
        speeds.append(v)

    return pts, (speeds if (have_speed and speeds) else None)


def load_csv_xy(path: str) -> List[Tuple[float, float]]:
    """x,y 만. 3번째 열이 있어도 무시한다 (기존 호출부 호환)."""
    return load_csv_xyv(path)[0]


def apply_track_direction(
    points: List[Tuple[float, float]], reverse: bool
) -> List[Tuple[float, float]]:
    """폐곡선 CSV 진행 방향 반전 (로컬 pose yaw 와 경로 tangent 불일치 시)."""
    if not reverse or len(points) < 2:
        return points
    return list(reversed(points))


def apply_track_direction_scalars(values, reverse: bool):
    """속도 등 웨이포인트 정렬 배열을 points 와 같은 방향으로 뒤집는다."""
    if values is None or not reverse or len(values) < 2:
        return values
    return list(reversed(values))


def _closest_point_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float]:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-14:
        return ax, ay, 0.0
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    return ax + t * abx, ay + t * aby, t


def lateral_distance_to_closed_polyline(
    mx: float, my: float, pts: List[Tuple[float, float]]
) -> float:
    """
    맵 평면에서 점 (mx,my) 과 폐폴리라인(pts) 사이 최단 거리(m).
    트랙 코리도 필터: 레이스라인에 가깝지 않으면(벽 등) 큰 값.
    """
    n = len(pts)
    if n < 2:
        return float("inf")
    xy = np.asarray(pts, dtype=np.float64)
    ax, ay = xy[:, 0], xy[:, 1]
    bx, by = np.roll(ax, -1), np.roll(ay, -1)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = np.divide(
        (mx - ax) * abx + (my - ay) * aby,
        ab2,
        out=np.zeros_like(ab2),
        where=ab2 >= 1e-14,
    )
    t = np.clip(t, 0.0, 1.0)
    qx = ax + t * abx
    qy = ay + t * aby
    d2 = (mx - qx) ** 2 + (my - qy) ** 2
    return float(math.sqrt(d2.min()))


class LoopTrackSliding:
    """맵 평면 폐선 궤적 + 앵커 검색 폭으로 슬라이딩 N점."""

    def __init__(
        self,
        points: List[Tuple[float, float]],
        path_window_size: int,
        path_anchor_half_width: int,
    ) -> None:
        if len(points) < 2:
            raise ValueError("LoopTrackSliding needs ≥2 points")
        self.points = points
        self.path_window_size = max(10, int(path_window_size))
        self.path_anchor_half_width = max(30, int(path_anchor_half_width))
        self._track_anchor_seg = 0
        self._anchor_initialized = False

    def reset_anchor(self) -> None:
        self._anchor_initialized = False
        self._track_anchor_seg = 0

    def closest_projection_on_loop(self, mx: float, my: float) -> Tuple[float, float, int]:
        pts = self.points
        n = len(pts)
        half = self.path_anchor_half_width

        def eval_seg(i: int) -> Tuple[float, float, float]:
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % n]
            qx, qy, _t = _closest_point_on_segment(mx, my, ax, ay, bx, by)
            d2 = (mx - qx) ** 2 + (my - qy) ** 2
            return qx, qy, d2

        best_qx, best_qy = 0.0, 0.0
        best_seg = 0
        best_d2 = float("inf")

        if not self._anchor_initialized:
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i
            self._anchor_initialized = True
        else:
            for k in range(-half, half + 1):
                i = (self._track_anchor_seg + k) % n
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i

        if best_d2 > 100.0:
            best_d2 = float("inf")
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i

        self._track_anchor_seg = best_seg
        return (best_qx, best_qy, best_seg)

    def sliding_xy(self, mx: float, my: float) -> List[Tuple[float, float]]:
        n = len(self.points)
        px, py, seg_i = self.closest_projection_on_loop(mx, my)
        w = min(self.path_window_size, n)
        out: List[Tuple[float, float]] = [(px, py)]
        for k in range(1, w):
            out.append(self.points[(seg_i + k) % n])
        return out
