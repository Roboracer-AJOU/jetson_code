#!/usr/bin/env python3
"""계획한 회피 기동을 **끝까지 붙들고 가는지** 검증.

    python3 -m pytest src/path_following/test/test_offset_maneuver_commit.py -q

계획 자체가 완만해도, 매 주기 자차 위치로 다시 그리면 소용이 없다. 차는 진입
곡선에서 제일 급한 앞부분만 계속 새로 타게 되고 목표 오프셋에는 영영 닿지
못한 채 조향만 물고 있는다. 실제로 이전 구현이 그랬다 — 여기서 그 회귀를
막는다. 노드를 띄우지 않고 계산 메서드만 스텁 self 에 바인딩해 돌린다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path as FsPath
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.local_planner_node import LocalPlannerNode  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
)

CFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.12,
    max_offset_m=0.65,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=2.0,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    enter_max_m=9.0,
    exit_min_m=1.5,
    exit_max_m=12.0,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.60 * 0.3735,
    wheelbase_m=vg.WHEELBASE_M,
)


class _Node:
    """기준선이 +x 축 직선인 가짜 플래너.

    직선이라 s=x, d=y 이고 궤도 yaw 는 0 이다. 커밋/재계획 판정은 곡률과
    무관하므로 이 단순화가 결론을 바꾸지 않는다.
    """

    _plan_or_keep_maneuver = LocalPlannerNode._plan_or_keep_maneuver
    _advance_maneuver_ds = LocalPlannerNode._advance_maneuver_ds
    _MANEUVER_MAX_STEP_M = LocalPlannerNode._MANEUVER_MAX_STEP_M
    _maneuver_still_clears = LocalPlannerNode._maneuver_still_clears
    _maneuver_obstacles_sd = LocalPlannerNode._maneuver_obstacles_sd
    _clear_maneuver = LocalPlannerNode._clear_maneuver
    _build_offset_path = LocalPlannerNode._build_offset_path
    _log_maneuver = LocalPlannerNode._log_maneuver
    _hold_window = LocalPlannerNode._hold_window
    _plan_fitting_the_track = LocalPlannerNode._plan_fitting_the_track
    _maneuver_fits_walls = LocalPlannerNode._maneuver_fits_walls
    _WALL_FIT_STEP_M = LocalPlannerNode._WALL_FIT_STEP_M

    def __init__(self, obstacles_map=(), v=6.0, wall_left=None, wall_right=None):
        self.maneuver_cfg = CFG
        # 좌/우 오프셋 예산. None 이면 트랙이 무한히 넓다고 본다.
        self.wall_left = wall_left
        self.wall_right = wall_right
        self.avoid_offset_max_m = CFG.max_offset_m
        self.avoid_offset_plan_v_floor_mps = 2.0
        self.avoid_offset_plan_v_step_mps = 0.5
        # 벽 예산 격자가 없으면 `_maneuver_fits_walls` 는 무조건 통과한다.
        self._budget_left = None
        self._budget_right = None
        self._total_l = 0.0
        self.avoid_offset_replan_lateral_m = 0.20
        self.avoid_offset_replan_obstacle_m = 0.35
        self.avoid_offset_step_m = 0.10
        self.avoid_frenet_exit_len_m = 1.5
        self.trailing_min_leader_speed_mps = 0.5
        self.map_frame = "map"
        self._ego_speed_mps = v
        self._maneuver = None
        self._maneuver_s0 = None
        self._maneuver_last_s = None
        self._maneuver_ds_cache = None
        self._maneuver_speed_cap = None
        self._last_maneuver_log_ns = 0
        self._dynamic_sd = []
        #: 맵 고정 장애물 (s_abs, d, r)
        self.obstacles_map = list(obstacles_map)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.plan_calls = 0

    # ---- 자차 상태 ----
    @property
    def _s_ego(self):
        return self.x

    @property
    def _static_sd(self):
        return self.obstacles_map

    def pose(self):
        p = SimpleNamespace()
        p.pose = SimpleNamespace()
        p.pose.position = SimpleNamespace(x=self.x, y=self.y, z=0.0)
        p.pose.orientation = SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(self.yaw / 2), w=math.cos(self.yaw / 2)
        )
        return p

    def _wall_budget_over(self, _s_from, _s_to):
        cap = self.avoid_offset_max_m
        return (
            cap if self.wall_left is None else self.wall_left,
            cap if self.wall_right is None else self.wall_right,
        )

    # ---- 직선 기준선 기하 ----
    def _delta_s(self, a, b):
        return float(a) - float(b)

    def _frenet_xy(self, mx, my):
        return float(mx), float(my)

    def _project_to_frenet(self, x, y, yaw):
        return x, y, math.tan(yaw), 0.0, 0.0, yaw

    def _xy_yaw_at_s(self, s):
        return float(s), 0.0, 0.0

    def _append_pose(self, path, x, y):
        path.poses.append(SimpleNamespace(x=float(x), y=float(y)))

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0, to_msg=lambda: None)
        )

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)


def _fake_path_cls(monkeypatch=None):
    return SimpleNamespace(poses=[], header=SimpleNamespace())


@pytest.fixture(autouse=True)
def _stub_path(monkeypatch):
    """_build_offset_path 가 쓰는 ROS Path 를 가벼운 스텁으로 바꾼다."""
    import path_following.local_planner_node as mod

    class _P:
        def __init__(self):
            self.poses = []
            self.header = SimpleNamespace(frame_id="", stamp=None)

    monkeypatch.setattr(mod, "Path", _P)
    yield


def _drive(node, distance, step=0.05, replan_each_step=True):
    """계획을 따라 distance 만큼 전진시킨다. 계획 호출 횟수를 센다.

    노드의 한 주기와 같은 순서로 돈다: 진행도 전진 → 계획 유지/재계획 →
    경로 추종. 순서가 어긋나면 캐시가 한 주기 밀려 결론이 달라진다.
    """
    travelled = 0.0
    while travelled < distance:
        ds = node._advance_maneuver_ds(node.pose())
        if replan_each_step:
            before = node._maneuver
            node._plan_or_keep_maneuver(node.pose())
            if node._maneuver is not before:
                node.plan_calls += 1
                ds = node._advance_maneuver_ds(node.pose())
        if node._maneuver is not None and ds is not None:
            node.y = node._maneuver.d_at(ds)
        node.x += step
        travelled += step


# ----------------------------------------------------------------------
def test_plan_is_committed_not_redrawn_every_cycle():
    """완주할 때까지 계획은 한 번만 세워져야 한다."""
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    n.plan_calls = 1
    total = n._maneuver.total_length_m
    _drive(n, total - 0.2)
    assert n.plan_calls == 1, f"{n.plan_calls} 번 다시 그렸다 — 커밋이 안 된다"


def test_the_car_actually_reaches_the_planned_offset():
    """리앵커 버그가 있으면 오프셋에 영영 도달하지 못한다."""
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    d_pass = n._maneuver.d_pass
    _drive(n, n._maneuver.enter_end_ds + 0.3)
    assert abs(n.y - d_pass) < 0.02, f"오프셋 {d_pass:.2f} 목표인데 {n.y:.2f} 에 있다"


def test_the_car_ends_back_on_the_line():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    _drive(n, n._maneuver.total_length_m + 0.5)
    assert abs(n.y) < 0.02, f"복귀 후 CTE {n.y:.3f} m"


def test_clearance_is_maintained_while_passing():
    s_obs, r = 10.0, 0.25
    n = _Node(obstacles_map=[(s_obs, 0.0, r)])
    n._plan_or_keep_maneuver(n.pose())
    worst = 1e9
    while n.x < s_obs + 1.0:
        if s_obs - r <= n.x <= s_obs + r:
            worst = min(worst, abs(n.y) - r - vg.HALF_WIDTH_M)
        _drive(n, 0.05, step=0.05)
    assert worst >= CFG.lateral_margin_m - 1e-6, f"최소 여유 {worst:.3f} m"


def test_replans_when_a_new_obstacle_invalidates_the_plan():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    first = n._maneuver
    assert first.side == +1
    # 계획한 통과 위치에 장애물이 새로 나타났다.
    n.obstacles_map.append((10.0, first.d_pass, 0.25))
    n._plan_or_keep_maneuver(n.pose())
    assert n._maneuver is not first, "막힌 계획을 그대로 들고 간다"


def test_replans_when_the_car_drifts_off_the_plan():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    first = n._maneuver
    n.x += 1.0
    n.y = first.d_at(1.0) + 0.5  # 계획에서 크게 벗어남
    n._advance_maneuver_ds(n.pose())
    n._plan_or_keep_maneuver(n.pose())
    assert n._maneuver is not first


def test_small_tracking_error_does_not_trigger_a_replan():
    """추종 오차 몇 cm 로 다시 그리면 매 주기 재계획과 똑같아진다."""
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    first = n._maneuver
    n.x += 1.0
    n.y = first.d_at(1.0) + 0.05
    n._advance_maneuver_ds(n.pose())
    n._plan_or_keep_maneuver(n.pose())
    assert n._maneuver is first


def test_plan_is_dropped_once_finished():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    _drive(n, n._maneuver.total_length_m + 0.5, replan_each_step=False)
    n.obstacles_map = []
    n._advance_maneuver_ds(n.pose())
    assert n._plan_or_keep_maneuver(n.pose()) is False
    assert n._maneuver is None


def test_moving_leader_is_not_treated_as_something_to_swerve_around():
    n = _Node()
    n._dynamic_sd = [(10.0, 0.0, 0.25, 4.0, 1.0)]  # 같은 방향 4 m/s
    assert n._plan_or_keep_maneuver(n.pose()) is False
    n._dynamic_sd = [(10.0, 0.0, 0.25, 0.05, 1.0)]  # 사실상 정지
    assert n._plan_or_keep_maneuver(n.pose()) is True


class _LoopNode(_Node):
    """s 가 37.4 m 에서 한 바퀴 도는 실제 트랙 길이의 폐곡선."""

    TOTAL = 37.4

    def _delta_s(self, a, b):
        d = (float(a) - float(b)) % self.TOTAL
        return d - self.TOTAL if d >= 0.5 * self.TOTAL else d

    def _frenet_xy(self, mx, my):
        return float(mx) % self.TOTAL, float(my)

    @property
    def _s_ego(self):
        return self.x % self.TOTAL


def test_progress_survives_the_lap_wraparound():
    """기동이 트랙 반바퀴를 넘겨도 진행도가 뒤집히면 안 된다.

    `_delta_s(s_now, s0)` 를 그대로 쓰면 [-L/2, +L/2) 로 접혀서, 37 m 트랙의
    18.7 m 지점에서 진행도가 +19 대신 -18 로 나온다. 그 상태로는 "기동이
    끝났나" 를 영영 판정하지 못해 AVOID 에서 못 빠져나온다.
    """
    n = _LoopNode(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    last = -1.0
    for _ in range(700):  # 35 m
        n.x += 0.05
        ds = n._advance_maneuver_ds(n.pose())
        assert ds > last, f"진행도가 되감겼다: {last:.2f} → {ds:.2f}"
        last = ds
    assert last == pytest.approx(35.0, abs=0.2)


def test_maneuver_fits_well_inside_one_lap():
    """한 기동이 반바퀴를 먹으면 랩 대부분을 라인 밖에서 보내게 된다."""
    cap = 0.15 * _LoopNode.TOTAL
    cfg = ManeuverConfig(
        **{**CFG.__dict__, "enter_max_m": cap, "exit_max_m": cap}
    )
    from path_following.offset_maneuver import plan_maneuver

    for v in (4.0, 6.0, 7.0):
        m = plan_maneuver(
            [ObstacleSD(12.0, 0.0, 0.25)], cfg, d_ego=0.0, d_ego_prime=0.0, v=v
        )
        assert m is not None
        # 총 길이는 장애물까지의 거리를 포함하니 짧을 수가 없다. 문제는
        # **라인을 벗어나 있는 거리** 이고, 그건 리드 구간만큼 줄어든다.
        off_line = m.total_length_m - m.lead_len_m
        assert off_line < 0.4 * _LoopNode.TOTAL, (
            f"v={v} 에서 라인 밖 {off_line:.1f} m — 랩의 절반을 벗어난다"
        )


# ----------------------------------------------------------------------
# 발행 경로
# ----------------------------------------------------------------------
def test_published_path_starts_at_the_car_not_behind_it():
    """지나온 구간을 발행하면 Stanley 최근접점이 뒤로 잡혀 조향이 뒤집힌다."""
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    _drive(n, 3.0)
    n._advance_maneuver_ds(n.pose())
    path = n._build_offset_path(n.pose())
    assert path is not None and len(path.poses) >= 2
    assert path.poses[0].x >= n.x - 1e-6, "경로가 차 뒤에서 시작한다"


def test_published_path_reaches_past_the_return():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    path = n._build_offset_path(n.pose())
    assert path.poses[-1].x > n._maneuver.total_length_m
    assert abs(path.poses[-1].y) < 1e-6, "경로 끝은 라인 위여야 한다"


def test_published_path_matches_the_plan():
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    _drive(n, 2.0)
    n._advance_maneuver_ds(n.pose())
    path = n._build_offset_path(n.pose())
    for p in path.poses[:60]:
        ds = p.x - n._maneuver_s0
        assert p.y == pytest.approx(n._maneuver.d_at(ds), abs=1e-9)


def test_path_curvature_stays_within_the_planned_steering():
    """발행 경로를 실제로 미분해서 조향 요구를 잰다."""
    n = _Node(obstacles_map=[(10.0, 0.0, 0.25)])
    n._plan_or_keep_maneuver(n.pose())
    path = n._build_offset_path(n.pose())
    ys = [p.y for p in path.poses]
    h = n.avoid_offset_step_m
    worst = max(
        abs(ys[i - 1] - 2 * ys[i] + ys[i + 1]) / (h * h)
        for i in range(1, len(ys) - 1)
    )
    steer = math.degrees(math.atan(vg.WHEELBASE_M * worst))
    assert steer < 3.0, f"6 m/s 회피에 조향 {steer:.2f}° 를 요구한다"
