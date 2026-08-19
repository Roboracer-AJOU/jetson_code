"""장애물 게이트 — local_planner 전용 (static 출력 필터)."""
from __future__ import annotations

import math
from typing import List, Tuple

from path_following.track_sliding import lateral_distance_to_closed_polyline


def _lateral_bound(
    corridor_active: bool,
    lateral_abs_max_m: float,
    lateral_abs_max_corridor_m: float,
) -> float:
    """레이저 프레임 |y| 상한.

    이 게이트는 차 진행축 기준 직선 튜브라서 곡선 구간과 헤딩 오차에 약하다.
    반경 10 m 코너에서는 레이스라인 정중앙 장애물도 3 m 앞에서 |y|=0.45 m 가
    되고, 직선에서도 헤딩이 5° 틀어지면 5 m 앞에서 0.44 m 다. 좁게 잡으면
    회피를 시작해야 할 거리에서 장애물이 통째로 사라진다.

    레이스라인 코리도 검사는 맵 좌표로 하므로 곡선에서도 정확하다. 그게
    도는 상황이면 여기서는 넓은 sanity bound 만 걸고 판단을 코리도에 맡긴다.
    코리도를 못 쓸 때만 예전의 좁은 튜브로 보수적으로 막는다.
    """
    return lateral_abs_max_corridor_m if corridor_active else lateral_abs_max_m


def _outside_corridor(
    x: float,
    y: float,
    r: float,
    *,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
) -> bool:
    """레이스라인에서 벗어나 무시해도 되는 장애물인가.

    좌표는 클러스터의 최근접점이라 물체의 한쪽 끝이다. 반경을 빼지 않으면
    레이스라인을 물고 있는 큰 장애물도 중심이 코리도 밖이면 통과시킨다.
    """
    mapped = laser_to_map(x, y)
    if mapped is None:
        return True
    mx, my = mapped
    lat = lateral_distance_to_closed_polyline(mx, my, track_pts)
    return (lat - abs(r)) > corridor_max_lat_m


def filter_obstacles_laser_frame(
    obstacle_data: list,
    *,
    forward_min_m: float,
    forward_max_m: float,
    lateral_abs_max_m: float,
    corridor_enable: bool,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
    require_corridor_tf: bool = True,
    lateral_abs_max_corridor_m: float | None = None,
) -> list:
    """
    /static_obstacles [id,x,y,r,...] (laser) → planner 게이트 통과분만.
    laser_to_map: (lx, ly) -> (mx, my) or None if TF unavailable.
    require_corridor_tf=True 이고 코리도 ON인데 TF 없으면 [] (벽 오검으로 회피 진입 방지).
    """
    if len(obstacle_data) < 4:
        return []

    if corridor_enable and require_corridor_tf and laser_to_map is None:
        return []

    corridor_active = bool(corridor_enable and track_pts and laser_to_map is not None)
    lat_bound = _lateral_bound(
        corridor_active,
        lateral_abs_max_m,
        lateral_abs_max_m
        if lateral_abs_max_corridor_m is None
        else lateral_abs_max_corridor_m,
    )

    out: list = []
    n = len(obstacle_data) // 4
    for i in range(n):
        base = 4 * i
        oid = obstacle_data[base]
        x = float(obstacle_data[base + 1])
        y = float(obstacle_data[base + 2])
        r = float(obstacle_data[base + 3])

        if x < forward_min_m or x > forward_max_m:
            continue
        if abs(y) > lat_bound:
            continue

        if corridor_active and _outside_corridor(
            x,
            y,
            r,
            corridor_max_lat_m=corridor_max_lat_m,
            track_pts=track_pts,
            laser_to_map=laser_to_map,
        ):
            continue

        out.extend([float(oid), x, y, r])

    return out

def filter_obstacles_for_exit(
    obstacle_data: list,
    *,
    pass_rear_x_m: float,
    lateral_abs_max_m: float,
    corridor_enable: bool,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
    lateral_abs_max_corridor_m: float | None = None,
) -> list:
    """
    AVOID 해제/remain 용: 전방 min 제한 없이(후방까지), 코리도는 유지.
    벽(raw)을 그대로 쓰면 옆 벽 때문에 영원히 AVOID에 남는다.
    """
    if len(obstacle_data) < 4:
        return []
    if corridor_enable and laser_to_map is None:
        return []

    corridor_active = bool(corridor_enable and track_pts and laser_to_map is not None)
    lat_bound = _lateral_bound(
        corridor_active,
        lateral_abs_max_m,
        lateral_abs_max_m
        if lateral_abs_max_corridor_m is None
        else lateral_abs_max_corridor_m,
    )

    out: list = []
    n = len(obstacle_data) // 4
    for i in range(n):
        base = 4 * i
        oid = obstacle_data[base]
        x = float(obstacle_data[base + 1])
        y = float(obstacle_data[base + 2])
        r = float(obstacle_data[base + 3])

        if abs(y) > lat_bound:
            continue
        # 이미 충분히 뒤로 간 것은 제외
        if (x - r) <= pass_rear_x_m:
            continue

        if corridor_active and _outside_corridor(
            x,
            y,
            r,
            corridor_max_lat_m=corridor_max_lat_m,
            track_pts=track_pts,
            laser_to_map=laser_to_map,
        ):
            continue

        out.extend([float(oid), x, y, r])

    return out

def _pack_dynamic_as_static_gate(dynamic_data: list) -> list:
    """/dynamic_obstacles [id,x,y,vx,vy,r,...] → [id,x,y,r,...] (거리 게이트용)."""
    if len(dynamic_data) < 6:
        return []
    out: list = []
    n = len(dynamic_data) // 6
    for i in range(n):
        base = 6 * i
        out.extend(
            [
                float(dynamic_data[base]),
                float(dynamic_data[base + 1]),
                float(dynamic_data[base + 2]),
                float(dynamic_data[base + 5]),
            ]
        )
    return out

def filter_dynamic_obstacles_laser_frame(
    dynamic_data: list,
    *,
    forward_min_m: float,
    forward_max_m: float,
    lateral_abs_max_m: float,
    corridor_enable: bool,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
    require_corridor_tf: bool = True,
    lateral_abs_max_corridor_m: float | None = None,
) -> list:
    """
    /dynamic_obstacles [id,x,y,vx,vy,r,...] (laser pos, map vel) → planner 게이트 통과분.
    """
    if len(dynamic_data) < 6:
        return []

    if corridor_enable and require_corridor_tf and laser_to_map is None:
        return []

    corridor_active = bool(corridor_enable and track_pts and laser_to_map is not None)
    lat_bound = _lateral_bound(
        corridor_active,
        lateral_abs_max_m,
        lateral_abs_max_m
        if lateral_abs_max_corridor_m is None
        else lateral_abs_max_corridor_m,
    )

    out: list = []
    n = len(dynamic_data) // 6
    for i in range(n):
        base = 6 * i
        oid = dynamic_data[base]
        x = float(dynamic_data[base + 1])
        y = float(dynamic_data[base + 2])
        vx = float(dynamic_data[base + 3])
        vy = float(dynamic_data[base + 4])
        r = float(dynamic_data[base + 5])

        if x < forward_min_m or x > forward_max_m:
            continue
        if abs(y) > lat_bound:
            continue

        if corridor_active and _outside_corridor(
            x,
            y,
            r,
            corridor_max_lat_m=corridor_max_lat_m,
            track_pts=track_pts,
            laser_to_map=laser_to_map,
        ):
            continue

        out.extend([float(oid), x, y, vx, vy, r])

    return out

def filter_dynamic_obstacles_for_exit(
    dynamic_data: list,
    *,
    pass_rear_x_m: float,
    lateral_abs_max_m: float,
    corridor_enable: bool,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
    lateral_abs_max_corridor_m: float | None = None,
) -> list:
    if len(dynamic_data) < 6:
        return []
    if corridor_enable and laser_to_map is None:
        return []

    corridor_active = bool(corridor_enable and track_pts and laser_to_map is not None)
    lat_bound = _lateral_bound(
        corridor_active,
        lateral_abs_max_m,
        lateral_abs_max_m
        if lateral_abs_max_corridor_m is None
        else lateral_abs_max_corridor_m,
    )

    out: list = []
    n = len(dynamic_data) // 6
    for i in range(n):
        base = 6 * i
        oid = dynamic_data[base]
        x = float(dynamic_data[base + 1])
        y = float(dynamic_data[base + 2])
        vx = float(dynamic_data[base + 3])
        vy = float(dynamic_data[base + 4])
        r = float(dynamic_data[base + 5])

        if abs(y) > lat_bound:
            continue
        if (x - r) <= pass_rear_x_m:
            continue

        if corridor_active and _outside_corridor(
            x,
            y,
            r,
            corridor_max_lat_m=corridor_max_lat_m,
            track_pts=track_pts,
            laser_to_map=laser_to_map,
        ):
            continue

        out.extend([float(oid), x, y, vx, vy, r])

    return out

def closest_dynamic_obstacle_surface_m(
    dynamic_data: list,
    *,
    forward_cone_rad: float | None = None,
    min_forward_x_m: float = 0.0,
    lateral_abs_max_m: float | None = None,
    laser_to_base_x_m: float = 0.0,
) -> float:
    return closest_obstacle_surface_m(
        _pack_dynamic_as_static_gate(dynamic_data),
        forward_cone_rad=forward_cone_rad,
        min_forward_x_m=min_forward_x_m,
        lateral_abs_max_m=lateral_abs_max_m,
        laser_to_base_x_m=laser_to_base_x_m,
    )

def closest_dynamic_obstacle_speed_mps(
    dynamic_data: list,
    *,
    forward_cone_rad: float | None = None,
    min_forward_x_m: float = 0.0,
    lateral_abs_max_m: float | None = None,
    laser_to_base_x_m: float = 0.0,
) -> tuple[float, float, float]:
    """
    최근접 동적 장애: (표면거리 m, 속력 m/s, closing m/s).

    vx,vy 는 laser-frame 상대속도.
    closing = -(x*vx + y*vy)/r  (+면 가까워짐/위협, -면 멀어짐).
    """
    if len(dynamic_data) < 6:
        return float("inf"), 0.0, 0.0

    best_d = float("inf")
    best_speed = 0.0
    best_closing = 0.0
    n = len(dynamic_data) // 6
    for i in range(n):
        base = 6 * i
        x = float(dynamic_data[base + 1])
        y = float(dynamic_data[base + 2])
        vx = float(dynamic_data[base + 3])
        vy = float(dynamic_data[base + 4])
        r = float(dynamic_data[base + 5])
        xb = x + laser_to_base_x_m
        if xb < min_forward_x_m:
            continue
        if lateral_abs_max_m is not None and abs(y) > lateral_abs_max_m:
            continue
        if forward_cone_rad is not None:
            if xb <= 0.0:
                continue
            angle = math.atan2(y, xb)
            if abs(angle) > forward_cone_rad:
                continue
        d = math.hypot(xb, y) - r
        if d < best_d:
            best_d = max(0.0, d)
            best_speed = math.hypot(vx, vy)
            rng = math.hypot(x, y)
            if rng > 1e-3:
                best_closing = -(x * vx + y * vy) / rng
            else:
                best_closing = 0.0

    if best_d == float("inf"):
        return float("inf"), 0.0, 0.0
    return best_d, best_speed, best_closing

def closest_obstacle_surface_m(
    obstacle_data: list,
    *,
    forward_cone_rad: float | None = None,
    min_forward_x_m: float = 0.0,
    lateral_abs_max_m: float | None = None,
    laser_to_base_x_m: float = 0.0,
) -> float:
    """
    필터된 장애 목록에서 전방 콘·거리 기준 최근접 표면 거리(m).
    laser_to_base_x_m > 0 이면 laser→base_link 전방 오프셋을 더해
    base_link 기준 거리로 근사한다.
    """
    if len(obstacle_data) < 4:
        return float("inf")
    n = len(obstacle_data) // 4
    best = float("inf")
    for i in range(n):
        x = float(obstacle_data[4 * i + 1])
        y = float(obstacle_data[4 * i + 2])
        r = float(obstacle_data[4 * i + 3])
        xb = x + laser_to_base_x_m
        if xb < min_forward_x_m:
            continue
        if lateral_abs_max_m is not None and abs(y) > lateral_abs_max_m:
            continue
        if forward_cone_rad is not None:
            if xb <= 0.0:
                continue
            angle = math.atan2(y, xb)
            if abs(angle) > forward_cone_rad:
                continue
        d = math.hypot(xb, y) - r
        if d < best:
            best = d
    return max(0.0, best) if best != float("inf") else float("inf")

def obstacles_remain_for_avoid(
    obstacle_data: list,
    *,
    pass_rear_x_m: float,
    lateral_abs_max_m: float,
) -> bool:
    """
    True if any gate obstacle is not fully behind the vehicle (laser frame).
    Rear edge x-r must be <= pass_rear_x_m (e.g. -0.35) to count as cleared.
    """
    if len(obstacle_data) < 4:
        return False
    n = len(obstacle_data) // 4
    for i in range(n):
        x = float(obstacle_data[4 * i + 1])
        y = float(obstacle_data[4 * i + 2])
        r = float(obstacle_data[4 * i + 3])
        if abs(y) > lateral_abs_max_m:
            continue
        if (x - r) > pass_rear_x_m:
            return True
    return False

def csv_path_blocked_by_obstacles(
    obstacle_data: list,
    *,
    track_pts: List[Tuple[float, float]],
    vehicle_xy: Tuple[float, float],
    laser_to_map,
    lookahead_m: float,
    clear_radius_m: float,
) -> bool:
    """
    True if any obstacle (map) is within clear_radius of the CSV path
    for the next lookahead_m along the track from the vehicle.
    Used to delay GLOBAL return until the racing line ahead is clear.
    """
    if len(obstacle_data) < 4 or len(track_pts) < 2 or laser_to_map is None:
        return False
    if lookahead_m <= 0.0 or clear_radius_m <= 0.0:
        return False

    vx, vy = vehicle_xy
    n = len(track_pts)
    best_i = 0
    best_d2 = float("inf")
    for i, (px, py) in enumerate(track_pts):
        d2 = (px - vx) ** 2 + (py - vy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    path_xy: List[Tuple[float, float]] = []
    acc = 0.0
    i = best_i
    path_xy.append(track_pts[i])
    while acc < lookahead_m:
        j = (i + 1) % n
        ax, ay = track_pts[i]
        bx, by = track_pts[j]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            i = j
            continue
        acc += seg
        path_xy.append((bx, by))
        i = j
        if len(path_xy) > n + 2:
            break

    # keep clear_radius only
    n_obs = len(obstacle_data) // 4
    for oi in range(n_obs):
        lx = float(obstacle_data[4 * oi + 1])
        ly = float(obstacle_data[4 * oi + 2])
        rr = float(obstacle_data[4 * oi + 3])
        mapped = laser_to_map(lx, ly)
        if mapped is None:
            continue
        mx, my = mapped
        thresh2 = (clear_radius_m + rr) ** 2
        for px, py in path_xy:
            if (mx - px) ** 2 + (my - py) ** 2 <= thresh2:
                return True
    return False
