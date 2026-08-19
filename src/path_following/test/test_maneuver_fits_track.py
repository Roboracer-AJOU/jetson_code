#!/usr/bin/env python3
"""기동이 트랙 안에 들어갈 때까지 속도를 낮춰 다시 뽑는지 검증.

    python3 -m pytest src/path_following/test/test_maneuver_fits_track.py -q

진입·복귀 길이는 `sqrt(D2·|Δd|·v²/a)` 라 v 에 선형이다. 6 m/s 에서 0.58 m 를
비키려면 진입만 6.3 m 고 기동 전체가 16 m 인데, 우리 트랙은 37 m 다. 반
바퀴짜리 기동이 코너를 몇 개씩 지나가니 벽에 안 걸리는 게 이상하다.

실측(37 m 트랙 전 지점에 정면 장애물, `scripts/check_offset_budget.py`):

    6 m/s 고정      → FGM 폴백 51%
    속도 탐색 붙임  → FGM 폴백 14%, 기동으로 76% 처리 (감속 시 중앙값 3.5 m/s)

FGM 폴백이 곧 벽으로 가는 길이었으므로 이 차이가 크다. 감속을 싫어하지만,
감속이 필요한 자리는 **애초에 기동이 트랙에 안 들어가는** 자리다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path as FsPath

import numpy as np
import pytest

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.local_planner_node import (  # noqa: E402
    LocalPlannerNode,
    _forbid_order,
    _min_opt,
)
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
)

CFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.25,
    max_offset_m=0.70,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=2.0,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    enter_max_m=6.7,
    exit_min_m=1.5,
    exit_max_m=6.7,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.3735,
    wheelbase_m=vg.WHEELBASE_M,
)

TRACK_L = 37.4
N = 748


class _Node:
    """직선 기준선 + s 별 좌/우 벽 예산을 가진 가짜 플래너."""

    _plan_fitting_the_track = LocalPlannerNode._plan_fitting_the_track
    _maneuver_fits_walls = LocalPlannerNode._maneuver_fits_walls
    _WALL_FIT_STEP_M = LocalPlannerNode._WALL_FIT_STEP_M

    def __init__(self, v=6.0, left=0.70, right=0.70):
        self.maneuver_cfg = CFG
        self._ego_speed_mps = v
        self.avoid_offset_plan_v_floor_mps = 2.0
        self.avoid_offset_plan_v_step_mps = 0.5
        self._total_l = TRACK_L
        self._n = N
        self._budget_left = np.full(N, float(left))
        self._budget_right = np.full(N, float(right))

    def narrow(self, s_from, s_to, left=None, right=None):
        """[s_from, s_to] 구간의 예산을 좁힌다."""
        i0 = int(s_from / TRACK_L * N)
        i1 = min(N - 1, int(s_to / TRACK_L * N))
        if left is not None:
            self._budget_left[i0 : i1 + 1] = left
        if right is not None:
            self._budget_right[i0 : i1 + 1] = right

    def plan(self, obstacles, **kw):
        kw.setdefault("forbid_side", 0)
        kw.setdefault("max_left", CFG.max_offset_m)
        kw.setdefault("max_right", CFG.max_offset_m)
        return self._plan_fitting_the_track(
            obstacles, 0.0, 0.0, 0.0, **kw
        )


BLOCKER = [ObstacleSD(s=9.0, d=0.0, r=0.18)]


# ── 넓은 트랙에서는 감속하지 않는다 ─────────────────────────────────────


@pytest.mark.parametrize("v", [4.0, 5.0, 6.0, 7.0])
def test_a_wide_track_costs_no_speed(v: float):
    n = _Node(v=v)
    m, plan_v = n.plan(BLOCKER)
    assert m is not None
    assert plan_v is None, f"{v} m/s 에서 이유 없이 감속했다 ({plan_v})"


def test_the_full_speed_plan_is_the_first_one_tried():
    """되면 바로 받는다 — 더 느린 대안을 찾아보지 않는다."""
    n = _Node(v=6.0)
    m, _ = n.plan(BLOCKER)
    fast, _ = _Node(v=6.0).plan(BLOCKER)
    assert m.enter_len_m == pytest.approx(fast.enter_len_m)


# ── 좁아지면 속도를 낮춘다 ──────────────────────────────────────────────


def test_a_tight_stretch_forces_a_slower_plan():
    """기동이 지나갈 구간이 좁으면 감속해서 기동을 짧게 만든다."""
    n = _Node(v=6.0)
    n.narrow(0.0, 20.0, left=0.62, right=0.62)
    # 6 m/s 짜리 16 m 기동은 안 들어가지만 느리면 들어간다
    m, plan_v = n.plan(BLOCKER)
    assert m is not None, "감속으로도 못 풀었다"
    if plan_v is not None:
        assert plan_v < 6.0
        assert m.total_length_m < 16.0


def test_slowing_down_actually_shortens_the_maneuver():
    """길이는 v 에 **선형** 이다 — L = sqrt(D2·|Δd|·v²/a) ∝ v.

    횡가속이 v²/L² 이라 v² 로 착각하기 쉽지만, 예산을 고정하고 길이를 역산하면
    선형이다. 절반 속도 → 절반 길이.
    """
    fast, _ = _Node(v=6.0).plan(BLOCKER)
    slow, _ = _Node(v=3.0).plan(BLOCKER)
    assert slow.total_length_m < fast.total_length_m
    assert slow.enter_len_m == pytest.approx(0.5 * fast.enter_len_m, rel=0.02)


def test_it_never_plans_below_the_floor():
    n = _Node(v=6.0)
    n.narrow(0.0, TRACK_L, left=0.0, right=0.0)  # 어디로도 못 나간다
    m, plan_v = n.plan(BLOCKER)
    assert m is None and plan_v is None


def test_gives_up_rather_than_crawling():
    """아주 좁으면 계획을 포기한다 — 기어가면서 비키는 것보다 제동이 낫다."""
    n = _Node(v=6.0)
    n.narrow(0.0, TRACK_L, left=0.40, right=0.40)  # 필요량 0.58 미만
    m, _ = n.plan(BLOCKER)
    assert m is None


# ── 방향은 속도보다 먼저 바꾼다 ─────────────────────────────────────────


def test_switching_sides_is_preferred_over_slowing_down():
    """한쪽만 벽이면 반대로 가면 된다 — 속도를 깎을 이유가 없다."""
    n = _Node(v=6.0)
    n.narrow(0.0, 25.0, left=0.10)  # 왼쪽만 막음
    m, plan_v = n.plan(BLOCKER)
    assert m is not None
    assert m.side == -1
    assert plan_v is None, "방향만 바꾸면 되는데 감속했다"


def test_forbid_order_respects_an_explicit_block():
    assert _forbid_order(+1) == (+1,)
    assert _forbid_order(-1) == (-1,)


def test_forbid_order_tries_both_sides_when_free():
    assert _forbid_order(0) == (0, +1, -1)


# ── 벽 적합성 검사 ──────────────────────────────────────────────────────


def test_a_plan_inside_the_budget_fits():
    n = _Node(v=6.0)
    m, _ = n.plan(BLOCKER)
    assert n._maneuver_fits_walls(m, 0.0)


def test_the_ramp_is_checked_not_just_the_hold():
    """유지 구간은 넓고 진입 램프만 좁은 경우를 잡아야 한다.

    `_wall_budget_over` 는 유지 구간만 보므로 이걸 놓친다. 그래서 계획이
    나온 뒤 d(s) 를 점마다 다시 본다.
    """
    n = _Node(v=6.0)
    m, _ = n.plan(BLOCKER)
    assert m is not None
    # 진입이 끝나는 근처만 좁힌다 (유지 구간보다 앞)
    side_budget = (
        n._budget_left if m.side > 0 else n._budget_right
    )
    i = int((m.enter_end_ds / TRACK_L) * N)
    side_budget[max(0, i - 2) : i + 2] = 0.05
    assert not n._maneuver_fits_walls(m, 0.0)


def test_no_budget_grid_means_no_opinion():
    """맵이 아직 안 왔으면 통과시킨다 — 경로 충돌검사가 뒤에서 받는다."""
    n = _Node(v=6.0)
    m, _ = n.plan(BLOCKER)
    n._budget_left = None
    assert n._maneuver_fits_walls(m, 0.0)


def test_the_check_follows_the_side_of_the_offset():
    """왼쪽으로 가는 계획은 왼쪽 예산만 봐야 한다."""
    n = _Node(v=6.0)
    n.narrow(0.0, TRACK_L, right=0.0)  # 오른쪽 전부 막음
    m, _ = n.plan(BLOCKER)
    assert m is not None and m.side == +1
    assert n._maneuver_fits_walls(m, 0.0)


# ── 속도 상한 합성 ──────────────────────────────────────────────────────


def test_min_opt_combines_two_optional_caps():
    assert _min_opt(None, None) is None
    assert _min_opt(3.0, None) == 3.0
    assert _min_opt(None, 3.0) == 3.0
    assert _min_opt(4.0, 3.0) == 3.0
    assert _min_opt(2.0, 3.0) == 2.0


def test_a_track_limited_plan_reports_its_speed_cap():
    """트랙 때문에 느리게 계획했으면 그 속도가 상한으로 나가야 한다."""
    n = _Node(v=7.0)
    n.narrow(0.0, 20.0, left=0.60, right=0.60)
    m, plan_v = n.plan(BLOCKER)
    if m is not None and plan_v is not None:
        cap = _min_opt(m.speed_cap_mps, plan_v)
        assert cap is not None and cap <= plan_v + 1e-9
        assert cap < 7.0
