#!/usr/bin/env python3
"""계획 기동은 잘린 경로를 받으면 안 된다.

    python3 -m pytest src/path_following/test/test_planned_path_not_truncated.py -q

FGM 경로는 조준점까지 그은 직선이라 앞부분만 살려도 의미가 있다. 그래서
`_truncate_path_at_collision` 은 막힌 지점 앞에서 자르고 "남은 길이가
`path_check_min_length_m` 이상이면 쓸 만하다" 고 답한다.

기동에는 그 규칙을 쓰면 안 된다. 진입-유지-복귀가 한 덩어리라, 벽에 걸려
잘려 나가는 건 언제나 뒷부분(유지·복귀)이다. 그걸 받으면 차는 진입만 타고
최대 오프셋에 도달하는데 **그 지점이 바로 잘려 나간 벽**이다. 실차에서
회피하던 방향 벽에 박은 게 이것이다.
"""
from __future__ import annotations

import sys
from pathlib import Path as FsPath
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.avoidance_safety import InflatedMap  # noqa: E402
from path_following.local_planner_node import LocalPlannerNode  # noqa: E402


class _Grid:
    """x ≥ 3.0 m 가 통째로 벽인 5 m × 5 m 격자."""

    def __init__(self, res=0.05, n=100, wall_x=3.0):
        data = np.zeros((n, n), dtype=np.int8)
        data[:, int(wall_x / res) :] = 100
        self.info = SimpleNamespace(
            resolution=res,
            width=n,
            height=n,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        )
        self.data = data.ravel().tolist()


class _Checker:
    """`_path_fully_clear` / `_truncate_path_at_collision` 만 떼어낸 스텁."""

    _path_fully_clear = LocalPlannerNode._path_fully_clear
    _truncate_path_at_collision = LocalPlannerNode._truncate_path_at_collision
    _wall_stop_distance_m = LocalPlannerNode._wall_stop_distance_m
    _clearance_floor_at = LocalPlannerNode._clearance_floor_at

    def __init__(self, speed_mps: float = 0.0):
        self._inflated_map = InflatedMap(_Grid(), inflation_m=0.15)
        self.path_check_inflation_m = 0.15
        self.path_check_enable = True
        self.path_check_backoff_m = 0.0
        self.path_check_min_length_m = 0.6
        self._last_path_cut = 0
        self._map_warned = True
        self._ego_speed_mps = speed_mps
        self.wall_stop_check_enable = True
        self.wall_stop_reaction_sec = 0.15
        self.avoid_speed_params = SimpleNamespace(a_brake=3.0)

    def _obstacle_disks_map(self, _tf):
        return []

    def get_logger(self):
        return SimpleNamespace(warn=lambda *a, **k: None)


def test_a_short_wallward_path_is_fine_when_crawling():
    """저속에서는 짧은 경로도 받는다 — 서면 그만이다."""
    c = _Checker(speed_mps=0.5)
    assert c._wall_stop_distance_m() < 0.2


def test_the_same_path_is_refused_at_racing_speed():
    """레이스라인이 벽에 붙은 구간에서 FGM 이 바깥 갭을 고르는 그 상황."""
    assert _Checker(speed_mps=6.0)._wall_stop_distance_m() > 6.0


def test_the_requirement_grows_with_speed():
    seq = [_Checker(speed_mps=v)._wall_stop_distance_m() for v in (0, 2, 3, 4, 6, 7)]
    assert seq == sorted(seq)


def test_it_is_the_distance_we_actually_need_to_stop():
    """되짚어 계산: 반응거리 + v²/2a."""
    v, a, t = 5.0, 3.0, 0.15
    got = _Checker(speed_mps=v)._wall_stop_distance_m()
    assert got == pytest.approx(v * t + v * v / (2.0 * a))


def test_turning_the_check_off_restores_the_old_rule():
    c = _Checker(speed_mps=7.0)
    c.wall_stop_check_enable = False
    assert c._wall_stop_distance_m() == 0.0


def _path(x0, x1, step=0.1, y=2.5):
    p = SimpleNamespace(poses=[])
    x = x0
    while x <= x1 + 1e-9:
        p.poses.append(
            SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
            )
        )
        x += step
    return p


def test_a_path_clear_of_the_wall_passes():
    n = _Checker()
    assert n._path_fully_clear(_path(0.5, 2.5), None)


def test_a_path_into_the_wall_is_rejected_outright():
    """뒷부분만 벽이어도 전체를 거부한다."""
    n = _Checker()
    assert not n._path_fully_clear(_path(0.5, 4.0), None)


def test_the_old_rule_would_have_accepted_that_same_path():
    """회귀 방지 — 자르기 규칙은 이 경로를 받아 준다. 그게 사고였다."""
    n = _Checker()
    _, usable = n._truncate_path_at_collision(_path(0.5, 4.0), None)
    assert usable, "전제가 바뀌었다: 자르기 규칙이 더 이상 안 받아 준다"
    assert not n._path_fully_clear(_path(0.5, 4.0), None)


def test_a_path_blocked_immediately_is_rejected_by_both():
    n = _Checker()
    p = _path(2.9, 4.0)
    assert not n._path_fully_clear(p, None)
    _, usable = n._truncate_path_at_collision(_path(2.9, 4.0), None)
    assert not usable


def test_the_current_position_is_not_judged():
    """0번 점은 검사하지 않는다 — 이미 거기 서 있는데 할 수 있는 게 없다."""
    n = _Checker()
    p = _path(3.5, 3.5)  # 점 하나뿐이라 검사 자체를 건너뛴다
    assert n._path_fully_clear(p, None)


def test_checking_is_skipped_when_disabled():
    n = _Checker()
    n.path_check_enable = False
    assert n._path_fully_clear(_path(0.5, 4.0), None)


@pytest.mark.parametrize("end", [3.2, 3.5, 4.0, 4.9])
def test_any_amount_of_wall_overlap_is_rejected(end: float):
    n = _Checker()
    assert not n._path_fully_clear(_path(0.5, end), None)
