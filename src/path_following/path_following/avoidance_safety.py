"""회피 경로 충돌검사 + 회피 구간 속도 정책.

`local_planner_node` 가 만든 회피 경로가 실제로 갈 수 있는 길인지 보고, 그
회피를 몇 m/s 로 하면 되는지 계산한다. 노드에서 분리한 이유는 순수 함수라
ROS 없이 단위 검증이 되기 때문이다.

두 가지를 다룬다.

1. **경로 충돌검사** (`InflatedMap`, `first_blocked_index`)
   회피 경로는 FGM 목표점 너머로 직선 연장된다. 그 연장 구간은 아무도 검사한
   적이 없어서 코너에서는 그대로 벽을 향한다. 맵과 장애물로 잘라낸다.

2. **회피 속도** (`avoid_speed_limit`)
   회피 중에는 글로벌 CSV 속도가 의미 없다. CSV 속도는 "장애물이 없는 레이싱
   라인을 이 곡률로 돈다" 는 가정으로 뽑은 값인데, 회피는 그 라인을 벗어나
   훨씬 급한 조향을 하기 때문이다.
"""

from __future__ import annotations

import math

import numpy as np

from path_following import vehicle_geometry as vg

try:  # 인플레이션용. 없으면 맵 검사만 끄고 장애물 검사는 계속 동작한다.
    from scipy.ndimage import distance_transform_edt

    _HAVE_SCIPY = True
except Exception:  # pragma: no cover
    _HAVE_SCIPY = False


# =====================================================================
# 1. 맵 기반 충돌검사
# =====================================================================
class InflatedMap:
    """OccupancyGrid 를 차폭만큼 부풀려 놓고 "여기 갈 수 있나" 를 답한다.

    매 주기 반경 검사를 하면 40 Hz 에서 부담이라, 맵이 올 때 한 번
    distance transform 을 돌려 "장애물까지 거리" 격자를 만들어 둔다. 이후
    조회는 배열 인덱싱 한 번이다.
    """

    def __init__(
        self,
        grid,
        inflation_m: float,
        occupied_thresh: int = 50,
        include_unknown: bool = True,
    ):
        info = grid.info
        self.res = float(info.resolution)
        self.ox = float(info.origin.position.x)
        self.oy = float(info.origin.position.y)
        self.w = int(info.width)
        self.h = int(info.height)
        self.inflation_m = float(inflation_m)

        data = np.asarray(grid.data, dtype=np.int16).reshape(self.h, self.w)
        # 경로검사(include_unknown=True): unknown(-1) 도 막힌 것으로 본다. 맵
        # 밖으로 나가는 경로를 허용하면 안 되고, 미지 영역을 낙관하는 건 회피에서
        # 제일 위험하다.
        # AEB 맵필터(include_unknown=False): 반대로 unknown 은 "벽이 아님" 으로
        # 둬야 한다. 여기서는 "이 빔이 이미 아는 벽이냐" 를 묻는 것이라,
        # 미지 영역을 벽으로 쳐 버리면 그 자리에 새로 나타난 장애물을 놓친다.
        occupied = data >= occupied_thresh
        if include_unknown:
            occupied |= data < 0

        if _HAVE_SCIPY and self.res > 1e-9:
            # 자유 공간의 각 셀에서 가장 가까운 점유 셀까지 거리 [m]
            self.clearance = (
                distance_transform_edt(~occupied).astype(np.float32) * self.res
            )
        else:  # pragma: no cover
            self.clearance = np.where(occupied, 0.0, 1e3).astype(np.float32)

    def clearance_at(self, x: float, y: float) -> float:
        """맵 좌표 (x, y) 에서 가장 가까운 벽까지 거리 [m]. 맵 밖은 0."""
        j = int((x - self.ox) / self.res)
        i = int((y - self.oy) / self.res)
        if i < 0 or j < 0 or i >= self.h or j >= self.w:
            return 0.0
        return float(self.clearance[i, j])

    def blocked(self, x: float, y: float) -> bool:
        return self.clearance_at(x, y) < self.inflation_m

    def blocked_many(self, xs, ys):
        """`blocked` 의 벡터판. 맵 밖은 막힌 것으로 본다.

        오프셋 예산을 깔 때 레이스라인 전 점 × 후보 거리를 훑어야 해서,
        점마다 파이썬 호출을 하면 맵이 올 때마다 눈에 띄게 걸린다.
        """
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        j = ((xs - self.ox) / self.res).astype(np.int64)
        i = ((ys - self.oy) / self.res).astype(np.int64)
        inside = (i >= 0) & (j >= 0) & (i < self.h) & (j < self.w)
        clear = np.zeros(xs.shape, dtype=np.float32)
        clear[inside] = self.clearance[i[inside], j[inside]]
        return ~inside | (clear < self.inflation_m)


def first_blocked_index(
    points,
    inflated_map: InflatedMap | None,
    obstacle_disks=None,
    start_index: int = 0,
    min_clearance_m: float | None = None,
) -> int:
    """경로에서 처음으로 못 가는 점의 인덱스. 전부 통과면 len(points).

    points: [(x, y), ...] 맵 좌표
    obstacle_disks: [(x, y, radius), ...] 맵 좌표. 이미 차폭이 더해진 반경.
    start_index: 검사 시작점. 차량 현재 위치(0번)는 보통 건너뛴다 — 이미
        거기 서 있는데 "막혔다" 고 해봐야 할 수 있는 게 없고, 위치 오차로
        벽에 붙어 보이면 회피를 영영 포기하게 된다.
    min_clearance_m: 통과 기준 여유 [m]. None 이면 팽창반경.

        `start_index` 의 논리를 영역으로 넓히는 손잡이다. 0번만 봐주는 건
        점 하나(≈5 cm)만 봐주는 것이라, 차가 이미 좁은 데 들어와 있으면
        1번 점에서 똑같이 막힌다 — 실측에서 재합류가 이렇게 계속 기각됐다.
        호출부가 "차가 지금 서 있는 자리만큼" 을 기준으로 내려 주면, 그
        자리에서 빠져나가는 경로를 시작점이 좁다는 이유로 거부하지 않는다.
    """
    disks = obstacle_disks or []
    thr = (
        None
        if inflated_map is None
        else (
            inflated_map.inflation_m
            if min_clearance_m is None
            else float(min_clearance_m)
        )
    )
    for idx in range(max(0, start_index), len(points)):
        px, py = points[idx]
        if thr is not None and inflated_map.clearance_at(px, py) < thr:
            return idx
        for ox, oy, orad in disks:
            if (px - ox) ** 2 + (py - oy) ** 2 < orad * orad:
                return idx
    return len(points)


def trim_back(points, cut_index: int, backoff_m: float) -> int:
    """충돌점 cut_index 에서 backoff_m 만큼 더 물러난 유지 개수.

    마지막 점이 통과 한계에 딱 붙어 있으면 Stanley 가 그 점을 겨냥하다
    긁는다. 여유를 두고 끝낸다.
    """
    keep = min(cut_index, len(points))
    acc = 0.0
    while keep > 1 and acc < backoff_m:
        x0, y0 = points[keep - 1]
        x1, y1 = points[keep - 2]
        acc += math.hypot(x1 - x0, y1 - y0)
        keep -= 1
    return max(1, keep)


# =====================================================================
# 2. 회피 속도 정책
# =====================================================================
class AvoidSpeedParams:
    """회피 속도 계산에 쓰는 차량 물리값.

    `scripts/speed_profile.py` 의 VEHICLE 과 같은 성격이지만 여기가 더
    보수적이어야 한다. 저쪽은 미리 아는 매끈한 레이싱라인이고, 이쪽은
    실시간 센서로 급하게 만든 경로다.
    """

    def __init__(
        self,
        a_lat: float = 4.0,
        a_brake: float = 3.0,
        safety_factor: float = 0.7,
        standoff_m: float = 0.35,
        # 실측 치수. local_planner 가 같은 값으로 덮어쓰지만, 기본값이
        # 실차와 다르면 이 모듈만 따로 쓰는 곳에서 조용히 틀린다.
        ego_half_width_m: float = vg.HALF_WIDTH_M,  # 이전 0.17
        ego_front_m: float = vg.FRONT_M,            # 이전 0.30
        lateral_margin_m: float = 0.10,
        # "지나갈 수 있다" 로 판정할 때만 추가로 요구하는 횡여유. 정지거리
        # 한계를 통째로 면제하는 판단이라 최소 여유보다 조금 더 받는다.
        pass_clear_extra_m: float = 0.15,
        v_min: float = 0.6,
        v_max: float = 8.0,
    ):
        self.a_lat = max(0.1, a_lat)
        self.a_brake = max(0.1, a_brake)
        self.safety_factor = min(1.0, max(0.05, safety_factor))
        self.standoff_m = max(0.0, standoff_m)
        self.ego_half_width_m = max(0.01, ego_half_width_m)
        self.ego_front_m = max(0.0, ego_front_m)
        self.lateral_margin_m = max(0.0, lateral_margin_m)
        self.pass_clear_extra_m = max(0.0, pass_clear_extra_m)
        self.v_min = max(0.0, v_min)
        self.v_max = max(self.v_min, v_max)


def maneuver_speed_limit(
    target_fwd_m: float, target_lat_m: float, p: AvoidSpeedParams
) -> float:
    """회피 조향 자체가 만드는 횡가속도 한계 [m/s].

    FGM 목표점까지를 원호로 보면 곡률은 pure-pursuit 과 같이
    kappa = 2*|lat| / L^2 (L = 목표까지 직선거리). v = sqrt(a_lat / kappa) 다.
    많이 틀수록(=lat 이 클수록, 목표가 가까울수록) 느려진다.
    """
    l2 = target_fwd_m * target_fwd_m + target_lat_m * target_lat_m
    if l2 < 1e-6 or abs(target_lat_m) < 1e-4:
        return p.v_max
    kappa = 2.0 * abs(target_lat_m) / l2
    if kappa < 1e-6:
        return p.v_max
    return math.sqrt(p.a_lat / kappa)


def passing_clearance_m(y_base: float, target_lat_m: float) -> float:
    """회피 기동을 끝까지 수행했을 때 이 장애물과 벌어질 최소 횡거리 [m].

    자차의 횡위치는 지금(0) 에서 FGM 목표의 횡오프셋(`target_lat_m`) 까지
    지나간다. 장애물의 y 가 그 구간 **안** 에 있으면 언젠가 정면으로 지나므로
    여유는 0 이다. 밖에 있으면 두 끝점 중 가까운 쪽까지가 최소 여유다.

    부호가 핵심이다. 왼쪽으로 틀고 있는데 장애물이 오른쪽이면 멀어지지만,
    장애물도 왼쪽이면 오히려 다가간다. |y| 만 봐서는 이 둘을 구분 못 한다.
    """
    lo, hi = min(0.0, target_lat_m), max(0.0, target_lat_m)
    if lo <= y_base <= hi:
        return 0.0
    return min(abs(y_base), abs(y_base - target_lat_m))


def _obstacle_speed_limit(
    x_base: float,
    y_base: float,
    radius: float,
    v_obs_along: float,
    p: AvoidSpeedParams,
    target_lat_m: float = 0.0,
    passing: bool = False,
    path_lat_at=None,
) -> float | None:
    """장애물 하나가 거는 속도 한계 [m/s]. 진로 밖이면 None.

    "회피가 실패해도 부딪히기 전에 멈출 수 있는" 속도. 정지 장애물이면
    v = sqrt(2*a*d) 그대로고, 움직이는 장애물이면 그만큼 덜 줄여도 되므로
    상대속도 기준이 되어 v_obs 가 더해진다. 앞차와 같은 속도면 안 줄여도 된다.

    `passing=True` 는 지금 실제로 회피 조향 중이라는 뜻이다. 그때는 정면
    충돌이 아니라 **옆으로 지나가는** 상황이므로 판정 기준이 달라진다.
    """
    lateral_need = radius + p.ego_half_width_m + p.lateral_margin_m
    # 옆에 나란히(추월·스치기)만 제외한다. 전방이면 |y| 가 커 보여도
    # 레이싱라인이 굽어서 그런 것이라, "이미 피했다" 로 보면 코너 앞 콘을
    # CSV 속도로 들이받는다. 접근 구간에서 전방은 무조건 감속.
    beside = x_base < max(p.ego_front_m + p.standoff_m, 0.8)
    if beside and abs(y_base) >= lateral_need:
        return None

    # 회피 조향 중이고, 그 기동을 끝내면 확실히 옆으로 비켜 지나간다면
    # 정지거리 한계를 걸지 않는다. 걸면 "피해서 지나갈 건데도 정면 충돌
    # 기준으로 제동" 하는 꼴이라, 옆을 스치는 내내 v_min 으로 기어간다.
    # (실측: 회피 중 배율이 0.17 에 붙어 0.49 m/s 까지 떨어졌다가, 장애물이
    #  시야에서 빠지는 순간 다시 튀어 나갔다.)
    # 조향 자체의 횡가속도 한계(maneuver)는 그대로 살아 있으므로 속도가
    # 무제한이 되지는 않는다.
    #
    # `path_lat_at` 이 있으면 "이 장애물 옆을 지날 때 우리 경로가 어디 있는지"
    # 를 직접 물어본다. 미리 계획한 기동은 그 답을 정확히 알고 있다.
    # 없을 때만 FGM 식 근사(지금→목표 사이를 훑는다)로 떨어진다. 계획 기동에
    # 그 근사를 쓰면 장애물이 그 구간 안에 있다는 이유로 여유를 0 으로 보고,
    # 실제로는 진작 비켜서 지나가는데도 정면충돌 기준으로 제동하게 된다.
    if passing:
        if path_lat_at is not None:
            clearance = abs(y_base - float(path_lat_at(x_base)))
        else:
            clearance = passing_clearance_m(y_base, target_lat_m)
        if clearance >= lateral_need + p.pass_clear_extra_m:
            return None

    gap = x_base - radius - p.ego_front_m - p.standoff_m
    if gap <= 0.0:
        return p.v_min
    return max(0.0, v_obs_along) + math.sqrt(2.0 * p.a_brake * gap)


def avoid_speed_limit(
    static_obstacles,
    dynamic_obstacles,
    ego_speed_mps: float,
    target_fwd_m: float,
    target_lat_m: float,
    p: AvoidSpeedParams,
    laser_to_base_x_m: float = 0.0,
    include_maneuver: bool = True,
    passing: bool | None = None,
    path_lat_at=None,
) -> tuple[float, str]:
    """목표 속도 [m/s] 와 그걸 정한 이유.

    static_obstacles:  flat [id, x, y, r, ...]        (laser frame)
    dynamic_obstacles: flat [id, x, y, vx, vy, r, ...] (laser frame, 상대속도)

    한계를 여러 개 구해서 제일 낮은 걸 쓴다.
      - maneuver : 회피 조향의 횡가속도 (급하게 틀수록 감속)
      - obstacle : 회피 실패 시 정지 가능 속도 (가까울수록 감속)
    마지막에 safety_factor 를 곱한다. 센서 지연, FGM 목표 흔들림, Stanley
    추종 오차처럼 위 계산이 모르는 오차를 덮는 몫이다.

    include_maneuver=False 는 아직 회피를 시작하지 않은 접근 구간(GLOBAL)용.
    조향을 안 하고 있으니 횡가속도 한계는 의미가 없고, 거리 기반 선감속만
    건다. 이게 있어야 회피 모드로 넘어가는 순간 속도가 계단으로 안 떨어진다.

    `passing` 은 "옆으로 지나가는 중이라 정지거리 한계를 면제해도 되는가" 다.
    기본은 include_maneuver 를 따라간다 — FGM 을 쫓을 때는 조향 중인 것과
    지나가는 중인 것이 같은 말이기 때문이다. 하지만 **미리 계획한 횡오프셋
    기동**에서는 둘이 갈린다. 그때 조향 한계는 계획이 이미 봤으므로
    include_maneuver=False 여야 하지만, 옆으로 비켜 지나가는 것은 사실이라
    면제는 받아야 한다. 묶어 두면 회피 내내 정면충돌 기준으로 제동이 걸려
    기어가게 된다.
    """
    if passing is None:
        passing = include_maneuver
    limits: list[tuple[float, str]] = []
    if include_maneuver:
        limits.append(
            (maneuver_speed_limit(target_fwd_m, target_lat_m, p), "maneuver")
        )

    for k in range(0, max(0, len(static_obstacles) - 3), 4):
        x = float(static_obstacles[k + 1]) + laser_to_base_x_m
        y = float(static_obstacles[k + 2])
        r = float(static_obstacles[k + 3])
        v = _obstacle_speed_limit(
            x,
            y,
            r,
            0.0,
            p,
            target_lat_m=target_lat_m,
            passing=passing,
            path_lat_at=path_lat_at,
        )
        if v is not None:
            limits.append((v, "static"))

    for k in range(0, max(0, len(dynamic_obstacles) - 5), 6):
        x = float(dynamic_obstacles[k + 1])
        y = float(dynamic_obstacles[k + 2])
        vx = float(dynamic_obstacles[k + 3])
        vy = float(dynamic_obstacles[k + 4])
        r = float(dynamic_obstacles[k + 5])
        rng = math.hypot(x, y)
        if rng < 1e-3:
            continue
        # closing = +면 접근. 상대속도라 앞차의 절대속도는 ego - closing.
        closing = -(x * vx + y * vy) / rng
        v_obs_along = max(0.0, float(ego_speed_mps) - closing)
        v = _obstacle_speed_limit(
            x + laser_to_base_x_m,
            y,
            r,
            v_obs_along,
            p,
            target_lat_m=target_lat_m,
            passing=passing,
            path_lat_at=path_lat_at,
        )
        if v is not None:
            limits.append((v, "dynamic"))

    if not limits:  # 진로에 아무것도 없음
        return p.v_max, "clear"

    raw, reason = min(limits, key=lambda t: t[0])
    v = raw * p.safety_factor
    v = min(p.v_max, max(p.v_min, v))
    return v, reason
