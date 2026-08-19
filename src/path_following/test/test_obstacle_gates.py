#!/usr/bin/env python3
"""플래너 장애물 게이트 — 곡선 구간에서 실장애를 버리지 않는지 검증.

    python3 -m pytest src/path_following/test/test_obstacle_gates.py -q

배경: `obstacle_lateral_abs_max_m` 는 레이저 프레임 |y| 상한이라 차 진행축
기준 직선 튜브다. 곡선이나 헤딩 오차가 있으면 레이스라인 정중앙 장애물도
튜브 밖으로 나간다 — 반경 10 m 코너에서 3 m 앞이면 |y|=0.45, 직선에서도
헤딩 5° 면 5 m 앞에서 0.44 다. 회피를 시작해야 할 거리에서 장애물이
사라지는 원인이었다. 레이스라인 코리도는 맵 좌표로 재므로 곡선에서도
정확하니, 코리도가 도는 동안에는 튜브를 넓히고 판단을 코리도에 맡긴다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.obstacle_filter import (  # noqa: E402
    filter_obstacles_laser_frame,
)

# 레이스라인 = 아래쪽 변 y=0 (x 0..20) 을 포함하는 닫힌 사각 루프.
TRACK = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]

NARROW = 0.42  # 예전 동작 (코리도 못 쓸 때의 폴백)
WIDE = 1.50    # 코리도가 돌 때의 sanity bound
CORRIDOR = 0.40

GATES = dict(
    forward_min_m=0.30,
    forward_max_m=12.0,
    lateral_abs_max_m=NARROW,
    corridor_max_lat_m=CORRIDOR,
    track_pts=TRACK,
    lateral_abs_max_corridor_m=WIDE,
)


def heading_offset_map(yaw_rad: float):
    """차가 레이스라인 대비 yaw 만큼 틀어져 있을 때의 laser→map."""

    def f(lx: float, ly: float):
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return (c * lx - s * ly, s * lx + c * ly)

    return f


def on_raceline_at(distance_m: float, yaw_rad: float) -> tuple[float, float]:
    """레이스라인 위 `distance_m` 앞 지점을, 틀어진 차의 레이저 좌표로."""
    return (distance_m * math.cos(yaw_rad), -distance_m * math.sin(yaw_rad))


def run(obstacles: list, *, corridor=True, laser_to_map=None) -> list:
    return filter_obstacles_laser_frame(
        obstacles,
        corridor_enable=corridor,
        laser_to_map=laser_to_map if corridor else None,
        require_corridor_tf=corridor,
        **GATES,
    )


# ------------------------------------------------- 곡선 / 헤딩 오차 (핵심)


def test_obstacle_on_raceline_survives_heading_offset():
    """헤딩 5° 에서 5 m 앞 레이스라인 정중앙 장애물 — 예전엔 잘렸다."""
    yaw = math.radians(5.0)
    x, y = on_raceline_at(5.0, yaw)
    assert abs(y) > NARROW, "전제: 예전 튜브 밖이어야 의미 있는 테스트다"

    out = run([0.0, x, y, 0.25], laser_to_map=heading_offset_map(yaw))
    assert len(out) == 4, "레이스라인 위 장애물이 헤딩 오차로 버려졌다"


def test_regression_narrow_tube_would_drop_it():
    """회귀 방지: 좁은 튜브만 쓰면 같은 장애물이 사라진다."""
    yaw = math.radians(5.0)
    x, y = on_raceline_at(5.0, yaw)
    out = filter_obstacles_laser_frame(
        [0.0, x, y, 0.25],
        corridor_enable=True,
        laser_to_map=heading_offset_map(yaw),
        require_corridor_tf=True,
        **{**GATES, "lateral_abs_max_corridor_m": NARROW},
    )
    assert out == []


def test_curved_approach_at_typical_avoid_distance():
    """반경 10 m 코너에서 3~5 m 앞 장애물이 모두 살아야 한다."""
    for d in (3.0, 4.0, 5.0):
        yaw = math.asin(min(1.0, d / (2 * 10.0)))  # 현-각 근사
        x, y = on_raceline_at(d, yaw)
        out = run([0.0, x, y, 0.25], laser_to_map=heading_offset_map(yaw))
        assert len(out) == 4, f"{d}m 앞 코너 장애물이 버려졌다 (|y|={abs(y):.2f})"


# ----------------------------------------------------- 트랙 밖은 계속 기각


def test_off_track_obstacle_still_rejected():
    """넓힌 튜브가 벽·트랙 밖 물체까지 통과시키면 안 된다."""
    out = run([0.0, 5.0, 1.20, 0.10], laser_to_map=lambda lx, ly: (lx, ly))
    assert out == []


def test_sanity_bound_still_applies():
    """코리도가 돌아도 |y| 가 sanity bound 를 넘으면 기각."""
    far = WIDE + 0.5
    out = run([0.0, 5.0, far, 0.10], laser_to_map=lambda lx, ly: (lx, ly))
    assert out == []


def test_narrow_tube_used_when_corridor_off():
    """코리도를 못 쓰면 예전의 보수적 튜브로 되돌아간다."""
    inside = run([0.0, 5.0, 0.30, 0.10], corridor=False)
    outside = run([0.0, 5.0, 0.60, 0.10], corridor=False)
    assert len(inside) == 4
    assert outside == []


# --------------------------------------------- 장애물 크기를 반영한 코리도


def test_wide_obstacle_reaching_into_corridor_is_kept():
    """최근접점은 코리도 밖이지만 몸통이 레이스라인을 물고 있는 경우.

    좌표는 클러스터 최근접점이라 물체의 한쪽 끝이다. 반경을 빼지 않으면
    50 cm 상자가 레이스라인에 걸쳐 있어도 통과시켜 버린다.
    """
    ident = lambda lx, ly: (lx, ly)  # noqa: E731
    lat = CORRIDOR + 0.20
    assert run([0.0, 5.0, lat, 0.25], laser_to_map=ident), "반경 반영이 안 됐다"
    assert run([0.0, 5.0, lat, 0.01], laser_to_map=ident) == []


def test_forward_window_unchanged():
    ident = lambda lx, ly: (lx, ly)  # noqa: E731
    assert run([0.0, 0.10, 0.0, 0.1], laser_to_map=ident) == []
    assert run([0.0, 15.0, 0.0, 0.1], laser_to_map=ident) == []
    assert len(run([0.0, 5.0, 0.0, 0.1], laser_to_map=ident)) == 4


def test_missing_tf_yields_empty():
    out = filter_obstacles_laser_frame(
        [0.0, 5.0, 0.0, 0.1],
        corridor_enable=True,
        laser_to_map=None,
        require_corridor_tf=True,
        **GATES,
    )
    assert out == []
