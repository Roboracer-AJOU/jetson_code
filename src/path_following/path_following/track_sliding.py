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


class SegmentGeometry:
    """폐폴리라인의 세그먼트 기하. 점 위치와 무관한 부분만 담는다.

    `ax, ay` 는 각 세그먼트 시작점, `abx, aby` 는 세그먼트 벡터, `ab2` 는 그
    길이제곱이다. 전부 폴리라인만으로 정해지므로 한 번 만들면 끝이다.
    """

    __slots__ = ("n", "ax", "ay", "abx", "aby", "ab2", "ok", "zeros")

    def __init__(self, pts: List[Tuple[float, float]]) -> None:
        xy = np.asarray(pts, dtype=np.float64)
        self.n = int(xy.shape[0])
        self.ax = np.ascontiguousarray(xy[:, 0])
        self.ay = np.ascontiguousarray(xy[:, 1])
        self.abx = np.roll(self.ax, -1) - self.ax
        self.aby = np.roll(self.ay, -1) - self.ay
        self.ab2 = self.abx * self.abx + self.aby * self.aby
        self.ok = self.ab2 >= 1e-14
        self.zeros = np.zeros_like(self.ab2)


# 폴리라인은 기동 중 안 바뀌므로 사실상 한두 개다. 리스트 **객체 자체** 를
# 같이 들고 있어야 한다 — id 만 키로 쓰면 리스트가 해제된 뒤 같은 주소에
# 다른 리스트가 앉았을 때 남의 기하를 돌려주게 된다.
_GEOM_CACHE: dict = {}
_GEOM_CACHE_MAX = 4


def segment_geometry(pts: List[Tuple[float, float]]) -> SegmentGeometry:
    """`pts` 의 세그먼트 기하 (캐시).

    이게 없으면 `lateral_distance_to_closed_polyline` 이 호출마다 750점
    리스트를 ndarray 로 변환하고 roll 두 번에 곱셈까지 다시 한다. 그 값들은
    레이스라인에서만 정해지는데, 실측에서 호출 1회 549 µs 중 대부분이
    거기였다 (플래너 40 Hz × 장애물 수만큼 도는 자리다).
    """
    key = id(pts)
    hit = _GEOM_CACHE.get(key)
    if hit is not None and hit[0] is pts:
        return hit[1]
    geom = SegmentGeometry(pts)
    if len(_GEOM_CACHE) >= _GEOM_CACHE_MAX:
        _GEOM_CACHE.clear()
    _GEOM_CACHE[key] = (pts, geom)
    return geom


def lateral_distance_to_closed_polyline(
    mx: float, my: float, pts: List[Tuple[float, float]]
) -> float:
    """
    맵 평면에서 점 (mx,my) 과 폐폴리라인(pts) 사이 최단 거리(m).
    트랙 코리도 필터: 레이스라인에 가깝지 않으면(벽 등) 큰 값.
    """
    if len(pts) < 2:
        return float("inf")
    g = segment_geometry(pts)
    ax, ay, abx, aby = g.ax, g.ay, g.abx, g.aby
    t = np.divide(
        (mx - ax) * abx + (my - ay) * aby,
        g.ab2,
        out=g.zeros.copy(),
        where=g.ok,
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
        self._offsets: np.ndarray | None = None
        self._all: np.ndarray | None = None

    def reset_anchor(self) -> None:
        self._anchor_initialized = False
        self._track_anchor_seg = 0

    def closest_projection_on_loop(self, mx: float, my: float) -> Tuple[float, float, int]:
        """앵커 주변에서 가장 가까운 세그먼트 투영. (qx, qy, seg_i).

        예전엔 세그먼트마다 파이썬 함수를 부르는 루프였다 (앵커폭 120 이면
        241 회, 33 Hz 기준 358 µs). 같은 계산을 numpy 한 번으로 접는다.

        인덱스 순서는 예전 루프와 **똑같이** `k = -half … +half` 로 만든다.
        `d2 < best_d2` 는 동점일 때 먼저 본 것을 남기고 `argmin` 도 첫 번째를
        돌려주므로, 순서가 같으면 고르는 세그먼트도 같다. 앵커는 다음 주기
        탐색 범위를 정하므로 여기서 하나만 어긋나도 궤적이 갈린다.
        """
        pts = self.points
        n = len(pts)
        g = segment_geometry(pts)

        def search(idx: np.ndarray) -> Tuple[float, float, int, float]:
            ax = g.ax[idx]
            ay = g.ay[idx]
            abx = g.abx[idx]
            aby = g.aby[idx]
            ab2 = g.ab2[idx]
            t = np.divide(
                (mx - ax) * abx + (my - ay) * aby,
                ab2,
                out=np.zeros_like(ab2),
                where=g.ok[idx],
            )
            np.clip(t, 0.0, 1.0, out=t)
            qx = ax + t * abx
            qy = ay + t * aby
            d2 = (mx - qx) ** 2 + (my - qy) ** 2
            k = int(np.argmin(d2))
            return float(qx[k]), float(qy[k]), int(idx[k]), float(d2[k])

        if not self._anchor_initialized:
            best_qx, best_qy, best_seg, best_d2 = search(self._all_idx(n))
            self._anchor_initialized = True
        else:
            half = self.path_anchor_half_width
            idx = (self._track_anchor_seg + self._window_offsets(half)) % n
            best_qx, best_qy, best_seg, best_d2 = search(idx)

        if best_d2 > 100.0:
            best_qx, best_qy, best_seg, best_d2 = search(self._all_idx(n))

        self._track_anchor_seg = best_seg
        return (best_qx, best_qy, best_seg)

    def _window_offsets(self, half: int) -> np.ndarray:
        if self._offsets is None or self._offsets.size != 2 * half + 1:
            self._offsets = np.arange(-half, half + 1, dtype=np.int64)
        return self._offsets

    def _all_idx(self, n: int) -> np.ndarray:
        if self._all is None or self._all.size != n:
            self._all = np.arange(n, dtype=np.int64)
        return self._all

    def sliding_xy(self, mx: float, my: float) -> List[Tuple[float, float]]:
        n = len(self.points)
        px, py, seg_i = self.closest_projection_on_loop(mx, my)
        w = min(self.path_window_size, n)
        out: List[Tuple[float, float]] = [(px, py)]
        for k in range(1, w):
            out.append(self.points[(seg_i + k) % n])
        return out
