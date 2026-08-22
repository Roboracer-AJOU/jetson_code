#!/usr/bin/env python3
"""좁은 데 들어와 있는 차가 거기서 빠져나갈 경로를 거부당하면 안 된다.

    python3 -m pytest src/path_following/test/test_clearance_floor.py -q

실측(20260822): 재합류가 일곱 번 연속 죽었다. 전부 **차 바로 앞**(5~55 cm)에서
막혔고, 이탈은 0.22~0.35 m 로 작았으며 헤딩도 21° 이내였다. 경로가 나빠서가
아니라 **그 자리 자체가 이미 막힌 것으로 판정** 되고 있었다.

`debug/raceline_clearance.py` 로 재 보니 이유가 분명했다. 레이스라인은 벽에서
최소 0.316 m / 중앙값 0.622 m 떨어져 있는데 팽창반경이 0.254 m 다. 즉

    이탈 0.20 m → 트랙의 25 % 가 팽창대 안
    이탈 0.35 m → 트랙의 47 % 가 팽창대 안

회피로 30 cm 비킨 차는 절반의 확률로 "막힌" 자리에 서 있고, 그러면 그
자리에서 출발하는 모든 경로가 1번째 점에서 기각된다. 거부해도 차는 그
자리를 벗어나지 못한다 — 오히려 재합류 경로가 거기서 빠져나가는 길이라
거부가 정확히 반대로 작동한다.

팽창반경은 물리적 벽이 아니라 여유 예산이다. 차 반폭은 0.15 m 다.
"""
from __future__ import annotations

import sys
from pathlib import Path as FsPath
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.avoidance_safety import (  # noqa: E402
    InflatedMap,
    first_blocked_index,
)
from path_following.local_planner_node import LocalPlannerNode  # noqa: E402

INFLATION = 0.254  # 실차 path_check_inflation_m


class _Corridor:
    """y=0 과 y=W 에 벽이 있는 폭 W 의 복도. x 방향으로 뚫려 있다."""

    def __init__(self, width_m=1.24, res=0.02, length_m=6.0):
        n_y = int(round((width_m + 2 * res) / res))
        n_x = int(round(length_m / res))
        data = np.zeros((n_y, n_x), dtype=np.int8)
        data[0, :] = 100
        data[-1, :] = 100
        self.info = SimpleNamespace(
            resolution=res,
            width=n_x,
            height=n_y,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=-res)),
        )
        self.data = data.ravel().tolist()


class _Node:
    _clearance_floor_at = LocalPlannerNode._clearance_floor_at

    def __init__(self, width_m=1.24):
        self._inflated_map = InflatedMap(_Corridor(width_m), INFLATION)
        self.path_check_inflation_m = INFLATION


def _straight(y, x0=1.0, n=40, step=0.055):
    return [(x0 + i * step, y) for i in range(n)]


def test_on_the_raceline_nothing_is_relaxed():
    """가운데(여유 0.62 m)에서는 기준이 팽창반경 그대로여야 한다."""
    n = _Node()
    assert n._inflated_map.clearance_at(2.0, 0.62) > INFLATION
    assert abs(n._clearance_floor_at((2.0, 0.62)) - INFLATION) < 1e-9


def test_in_a_tight_spot_the_floor_drops_to_where_the_car_already_is():
    n = _Node()
    here = n._inflated_map.clearance_at(2.0, 0.22)
    assert here < INFLATION, "이 자리가 팽창대 안이어야 시험이 성립한다"
    assert abs(n._clearance_floor_at((2.0, 0.22)) - here) < 1e-6


def test_the_floor_never_goes_below_the_car_half_width():
    """마진이 얇은 것과 물리적으로 못 들어가는 것은 다르다."""
    n = _Node()
    floor = n._clearance_floor_at((2.0, 0.06))  # 벽에서 6 cm — 반폭보다 좁다
    assert abs(floor - vg.HALF_WIDTH_M) < 1e-9


def test_a_path_leaving_a_tight_spot_is_no_longer_refused():
    """실측에서 죽던 그림 — 차가 좁은 데 있고 경로는 거기서 나가는 중."""
    n = _Node()
    im = n._inflated_map
    # 벽에서 0.22 m 지점에서 출발해 가운데로 빠져나가는 경로
    pts = [(1.0 + i * 0.055, 0.22 + i * 0.02) for i in range(40)]
    floor = n._clearance_floor_at(pts[0])

    strict = first_blocked_index(pts, im, None, start_index=1)
    relaxed = first_blocked_index(pts, im, None, start_index=1, min_clearance_m=floor)

    assert strict < len(pts), "예전 기준으로는 막혀야 이 시험이 의미가 있다"
    assert relaxed >= len(pts), "빠져나가는 경로를 여전히 거부한다"


def test_a_path_going_deeper_into_the_wall_is_still_refused():
    """풀어 준 건 '지금만큼' 까지다. 더 파고드는 경로는 그대로 막아야 한다."""
    n = _Node()
    im = n._inflated_map
    pts = [(1.0 + i * 0.055, 0.22 - i * 0.01) for i in range(40)]
    floor = n._clearance_floor_at(pts[0])
    assert first_blocked_index(pts, im, None, start_index=1, min_clearance_m=floor) < len(
        pts
    )


def test_a_wall_is_still_a_wall_from_the_middle_of_the_track():
    """가운데서 벽으로 꽂는 경로는 완화와 무관하게 막힌다."""
    n = _Node()
    im = n._inflated_map
    pts = [(1.0 + i * 0.055, 0.62 - i * 0.03) for i in range(40)]
    floor = n._clearance_floor_at(pts[0])
    assert abs(floor - INFLATION) < 1e-9
    assert first_blocked_index(pts, im, None, start_index=1, min_clearance_m=floor) < len(
        pts
    )


def test_the_default_still_uses_the_inflation_radius():
    """min_clearance_m 을 안 주면 예전 그대로 — 다른 호출부가 안 바뀐다."""
    n = _Node()
    im = n._inflated_map
    pts = _straight(0.22)
    assert first_blocked_index(pts, im, None, start_index=1) == first_blocked_index(
        pts, im, None, start_index=1, min_clearance_m=INFLATION
    )
