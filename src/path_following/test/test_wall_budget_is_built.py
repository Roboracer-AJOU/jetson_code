#!/usr/bin/env python3
"""벽 예산이 **실제로 만들어지는지** 검증.

    python3 -m pytest src/path_following/test/test_wall_budget_is_built.py -q

`test_wall_budget.py` 는 `plan_maneuver` 에 `max_left`/`max_right` 를 직접
넘겨서 "예산을 주면 지키는가" 만 본다. 그래서 그 예산을 **만드는** 쪽이
통째로 죽어 있어도 287 개가 전부 통과했다.

실제로 그런 일이 있었다. `_build_wall_budget` 이 `self.avoid_offset_max_m` 을
읽었는데 그 속성은 어디에서도 대입되지 않는다 (값은 `maneuver_cfg.max_offset_m`
에 있다). `/map` 이 올 때마다 콜백이 AttributeError 로 죽었고, 예산 계산 직전
줄에서 터지므로 `_budget_left` 는 계속 None 이었다. 예산이 None 이면 쓰는 쪽이
전부 조용히 빠져나가 상한 0.70 m 를 어디서나 낼 수 있다고 믿는 예전 동작으로
돌아간다 — 벽 예산이 막으려던 바로 그 동작이다.

그래서 여기서는 가짜 노드에 `avoid_offset_max_m` 을 **일부러 안 준다**. 상한을
maneuver_cfg 말고 다른 데서 읽으려 들면 AttributeError 로 걸린다.
"""
from __future__ import annotations

import sys
from pathlib import Path as FsPath

import numpy as np
import pytest

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.local_planner_node import LocalPlannerNode  # noqa: E402
from path_following.offset_maneuver import ManeuverConfig  # noqa: E402

CAP = 0.70

CFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.25,
    max_offset_m=CAP,
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

TRACK_L = 40.0
N = 400
STEP = LocalPlannerNode._WALL_BUDGET_STEP_M


class _Corridor:
    """+y 로 뻗은 직선 복도. 기준선이 +y 라 좌측(+d)은 -x, 우측은 +x 다."""

    def __init__(self, left: float, right: float, gap: tuple | None = None):
        self.left, self.right, self.gap = left, right, gap

    def blocked_many(self, xs, ys):
        hit = (xs < -self.left) | (xs > self.right)
        if self.gap is not None:  # 좌측 중간에 낀 벽 (그 너머는 다시 빈 공간)
            lo, hi = self.gap
            hit |= (xs <= -lo) & (xs >= -hi)
        return hit


class _Log:
    def info(self, msg: str) -> None:
        pass


class _Node:
    """맵과 직선 기준선만 가진 가짜 플래너.

    `avoid_offset_max_m` 은 일부러 없다 — 그게 이 파일의 요점이다.
    """

    _build_wall_budget = LocalPlannerNode._build_wall_budget
    _wall_budget_over = LocalPlannerNode._wall_budget_over
    _index_at_s = LocalPlannerNode._index_at_s
    _delta_s = LocalPlannerNode._delta_s
    _WALL_BUDGET_STEP_M = LocalPlannerNode._WALL_BUDGET_STEP_M

    def __init__(self, corridor: _Corridor | None, cap: float = CAP):
        self.maneuver_cfg = (
            CFG if cap == CAP else ManeuverConfig(**{**CFG.__dict__, "max_offset_m": cap})
        )
        self._inflated_map = corridor
        self._n = N
        self._total_l = TRACK_L
        self._xs_np = np.zeros(N)
        self._ys_np = np.linspace(0.0, TRACK_L, N)
        self._budget_left = None
        self._budget_right = None

    def get_logger(self):
        return _Log()


def _budgets(left: float, right: float, **kw):
    n = _Node(_Corridor(left, right, **kw))
    n._build_wall_budget()
    return n


# ── 애초에 만들어지기는 하는가 ──────────────────────────────────────────


def test_the_budget_actually_gets_built():
    """이게 죽어 있어서 벽에 박았다. 예산이 None 이면 안 된다."""
    n = _budgets(0.50, 0.50)
    assert n._budget_left is not None and n._budget_right is not None
    assert n._budget_left.shape == (N,)
    assert n._budget_left.max() > 0.0


def test_the_cap_comes_from_the_maneuver_config():
    """상한은 `maneuver_cfg.max_offset_m` 하나에서만 온다.

    다른 이름으로 읽으면 이 노드엔 그 속성이 없으므로 AttributeError 가 난다.
    상한을 바꿨을 때 예산이 따라오는지까지 봐야 진짜로 그 값을 쓴 것이다.
    """
    wide = _Corridor(5.0, 5.0)  # 벽은 상한보다 훨씬 멀다
    for cap in (0.30, 0.70):
        n = _Node(wide, cap=cap)
        n._build_wall_budget()
        assert float(np.median(n._budget_left)) == pytest.approx(cap, abs=STEP)


# ── 벽까지의 거리를 제대로 재는가 ───────────────────────────────────────


def test_a_near_wall_caps_the_budget():
    """벽이 상한보다 가까우면 벽 앞에서 멈춘다 (한 스텝 이내)."""
    n = _budgets(0.30, 0.30)
    got = float(np.median(n._budget_left))
    assert 0.30 - STEP - 1e-9 <= got <= 0.30 + 1e-9


def test_a_far_wall_leaves_the_configured_cap():
    """벽이 멀면 상한에서 잘린다 — 예산이 무한정 커지면 안 된다."""
    n = _budgets(3.0, 3.0)
    assert float(np.median(n._budget_left)) == pytest.approx(CAP, abs=STEP)
    assert float(np.median(n._budget_right)) == pytest.approx(CAP, abs=STEP)


def test_left_and_right_are_measured_separately():
    """레이스라인은 코너 안쪽에 붙는다 — 한쪽만 벽인 게 정상이다."""
    n = _budgets(0.25, 3.0)
    assert float(np.median(n._budget_left)) <= 0.25 + 1e-9
    assert float(np.median(n._budget_right)) == pytest.approx(CAP, abs=STEP)


def test_free_space_beyond_a_wall_is_not_budget():
    """벽 너머의 빈 공간을 세면 벽을 뚫고 지나가는 계획이 나온다."""
    n = _budgets(3.0, 3.0, gap=(0.20, 0.35))
    assert float(np.median(n._budget_left)) <= 0.20 + 1e-9


def test_no_map_means_no_budget():
    """맵이 아직 없으면 예산도 없다 — 0 으로 채우면 회피가 통째로 막힌다."""
    n = _Node(None)
    n._build_wall_budget()
    assert n._budget_left is None and n._budget_right is None


# ── 구간 조회 ───────────────────────────────────────────────────────────


def test_a_stretch_reports_its_tightest_point():
    """기동은 구간 내내 오프셋을 물고 있다 — 한 점이라도 못 내면 못 쓴다."""
    n = _budgets(3.0, 3.0)
    i0, i1 = int(10.0 / TRACK_L * N), int(11.0 / TRACK_L * N)
    n._budget_left[i0 : i1 + 1] = 0.20
    assert n._wall_budget_over(9.0, 12.0)[0] == pytest.approx(0.20)
    assert n._wall_budget_over(20.0, 25.0)[0] == pytest.approx(CAP, abs=STEP)


def test_without_a_grid_it_falls_back_to_the_cap():
    """예산이 없으면 상한을 그대로 돌려준다. 이 경로도 상한을 읽는다."""
    n = _Node(None)
    assert n._wall_budget_over(0.0, 5.0) == (CAP, CAP)
