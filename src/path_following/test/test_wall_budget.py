#!/usr/bin/env python3
"""회피가 벽 쪽으로 계획되지 않는지 검증.

    python3 -m pytest src/path_following/test/test_wall_budget.py -q

실차에서 회피하던 방향 벽에 박았다. 원인은 계획기가 트랙 경계를 전혀 안 본
것이다. `avoid_offset_max_m` 은 0.70 m 로 박혀 있었는데, 실측 레이스라인→벽
여유는 중앙값 0.70 m / 최소 0.30 m 라 **구간의 76% 에서 낼 수 없는 값**이었다.

원래는 경로 충돌검사가 그걸 받아 주기로 했지만 두 군데서 샜다.

  1. 검사는 경로를 **자르고** "남은 길이가 충분하면 쓸 만하다" 고 답했다.
     기동에서 잘려 나가는 건 복귀 구간이다. 차는 진입만 타고 최대 오프셋에
     도달하는데, 그 지점이 바로 잘려 나간 벽이다.
  2. 방향 선택이 "덜 움직이는 쪽" 이었다. 레이스라인은 코너 안쪽에 붙으므로
     덜 움직이는 쪽이 곧 벽인 경우가 많다.

실측 좌 중앙값 0.88 m / 우 0.78 m 인데 **둘 중 좋은 쪽**은 최소가 0.58 m 다.
방향만 제대로 고르면 이 트랙 어디서든 비킬 수 있다는 뜻이라, 예산을 좌우
따로 재서 넘긴다.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.avoidance_safety import InflatedMap  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
    choose_pass_offset,
    group_blocking,
    plan_maneuver,
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

# 정면 장애물. 비키려면 0.18 + 0.15 + 0.25 = 0.58 m 필요하다.
BLOCKER = [ObstacleSD(s=9.0, d=0.0, r=0.18)]
NEED = 0.18 + vg.HALF_WIDTH_M + CFG.lateral_margin_m


def _plan(**kw):
    return plan_maneuver(
        BLOCKER, CFG, d_ego=0.0, d_ego_prime=0.0, v=6.0, **kw
    )


# ── 방향 선택이 벽을 본다 ────────────────────────────────────────────────


def test_picks_the_open_side_when_one_side_is_a_wall():
    """왼쪽이 벽이면 오른쪽으로 나가야 한다."""
    m = _plan(max_left=0.20, max_right=0.80)
    assert m is not None
    assert m.side == -1, "벽 쪽으로 계획했다"
    assert m.d_pass < 0.0


def test_picks_the_open_side_the_other_way_too():
    m = _plan(max_left=0.80, max_right=0.20)
    assert m is not None
    assert m.side == +1
    assert m.d_pass > 0.0


def test_refuses_when_neither_side_has_room():
    """양쪽 다 좁으면 계획을 안 낸다 — 감속·TRAILING 이 받아야 한다."""
    assert _plan(max_left=0.30, max_right=0.30) is None


def test_a_budget_just_over_the_requirement_is_accepted():
    """예산이 필요량을 넘으면 딱 필요한 만큼만 나간다 — 남는다고 더 안 간다."""
    m = _plan(max_left=NEED + 0.01, max_right=0.10)
    assert m is not None and m.side == +1
    assert abs(m.d_pass) == pytest.approx(NEED, abs=1e-6)


def test_a_hair_under_the_requirement_is_refused():
    assert _plan(max_left=NEED - 0.01, max_right=NEED - 0.01) is None


def test_budget_never_exceeds_the_configured_cap():
    """예산이 아무리 넓어도 max_offset_m 을 넘지 않는다."""
    m = _plan(max_left=5.0, max_right=5.0)
    assert m is not None
    assert abs(m.d_pass) <= CFG.max_offset_m + 1e-9


# ── 회귀: 예산을 안 주면 예전처럼 벽을 무시한다 ──────────────────────────


def test_without_a_budget_the_planner_is_wall_blind():
    """이게 실차에서 벽에 박은 동작이다. 노드는 반드시 예산을 넘겨야 한다."""
    m = _plan()
    assert m is not None
    assert abs(m.d_pass) >= NEED - 1e-9


def test_cheaper_side_no_longer_wins_over_an_open_side():
    """이미 왼쪽에 나가 있어도, 왼쪽이 벽이면 오른쪽으로 간다.

    예전 선택 규칙(`abs(d_pass - d_ego)` 최소)이면 왼쪽이 이긴다.
    """
    m = plan_maneuver(
        BLOCKER,
        CFG,
        d_ego=+0.30,
        d_ego_prime=0.0,
        v=6.0,
        max_left=0.35,
        max_right=0.80,
    )
    assert m is not None
    assert m.side == -1


def test_cheaper_side_still_wins_when_both_are_open():
    """벽이 없으면 예전대로 덜 움직이는 쪽이다 — 지나가다 뒤집히면 안 된다."""
    m = plan_maneuver(
        BLOCKER,
        CFG,
        d_ego=+0.30,
        d_ego_prime=0.0,
        v=6.0,
        max_left=0.80,
        max_right=0.80,
    )
    assert m is not None
    assert m.side == +1


# ── choose_pass_offset 단위 ─────────────────────────────────────────────


def test_choose_pass_offset_respects_each_side_separately():
    g = group_blocking(BLOCKER, CFG)
    assert g is not None
    assert choose_pass_offset(g, CFG, 0.0, max_left=0.10, max_right=0.10) is None
    got = choose_pass_offset(g, CFG, 0.0, max_left=0.10, max_right=0.80)
    assert got is not None and got[1] == -1


def test_forbid_side_still_works_with_budgets():
    """충돌검사에 걸린 방향은 예산이 있어도 다시 안 고른다."""
    g = group_blocking(BLOCKER, CFG)
    got = choose_pass_offset(
        g, CFG, 0.0, forbid_side=-1, max_left=0.80, max_right=0.80
    )
    assert got is not None and got[1] == +1


# ── InflatedMap.blocked_many 가 blocked 와 일치하는가 ───────────────────


class _Grid:
    """가운데 세로벽 하나 있는 5 m × 5 m 격자."""

    def __init__(self):
        res, n = 0.05, 100
        data = np.zeros((n, n), dtype=np.int8)
        data[:, 50] = 100
        self.info = type(
            "I",
            (),
            {
                "resolution": res,
                "width": n,
                "height": n,
                "origin": type(
                    "O",
                    (),
                    {"position": type("P", (), {"x": 0.0, "y": 0.0})()},
                )(),
            },
        )()
        self.data = data.ravel().tolist()


def test_blocked_many_matches_blocked_point_by_point():
    im = InflatedMap(_Grid(), inflation_m=0.20)
    xs = np.linspace(-0.5, 5.5, 61)
    ys = np.full_like(xs, 2.5)
    got = im.blocked_many(xs, ys)
    want = [im.blocked(float(x), float(y)) for x, y in zip(xs, ys)]
    assert list(got) == want


def test_outside_the_map_counts_as_blocked():
    im = InflatedMap(_Grid(), inflation_m=0.20)
    got = im.blocked_many([-1.0, 99.0], [2.5, 2.5])
    assert bool(got[0]) and bool(got[1])
