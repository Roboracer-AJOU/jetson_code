#!/usr/bin/env python3
"""AEB 로 멈춘 뒤 **멈춘 방향으로** 빠져나가는지 검증.

    python3 -m pytest src/path_following/test/test_escape_heading_lock.py -q

FGM 은 "지금 제일 열린 각도" 만 본다. 장애물 정면에 멈추면 정면 섹터가 통째로
버블에 막히니 옆이 이기고, 조준각이 FOV 끝(±80°)까지 간다. 탈출 속도 0.8 m/s
로 2 초면 1.6 m 인데 최대 조향의 회전반경이 0.85 m 라 그 사이 헤딩이 100° 넘게
돈다 — 실차에서 옆으로 돌다 역주행 방향이 되거나 벽에 붙었다.

그래서 멈춘 순간의 맵 헤딩을 기억하고, 그걸 차체 기준 각도로 바꿔 FGM 에
[기준각, 콘] 으로 넘긴다. 차가 돌아간 만큼 기준각이 반대로 움직이므로 조준이
스스로 되돌아온다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.fgm_node import FGMNode  # noqa: E402
from path_following.local_planner_node import LocalPlannerNode  # noqa: E402

CONE = math.radians(55.0)


# ── 플래너: 멈춘 헤딩을 기억해 기준각을 낸다 ────────────────────────────


def _pose(yaw_deg: float):
    """맵 프레임 자세. `_quat_to_yaw` 가 읽는 필드만 채운다."""
    h = math.radians(yaw_deg) * 0.5
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))
    return SimpleNamespace(pose=SimpleNamespace(orientation=q))


class _Pub:
    def __init__(self):
        self.sent: list[list[float]] = []

    def publish(self, msg):
        self.sent.append([float(v) for v in msg.data])

    @property
    def last(self) -> list[float]:
        return self.sent[-1]


class _Log:
    def warn(self, _m):
        pass

    def info(self, _m):
        pass


class _Planner:
    _update_escape_heading = LocalPlannerNode._update_escape_heading

    def __init__(self, escaping: bool = True, lock: bool = True):
        self._escaping = escaping
        self.aeb_escape_heading_lock = lock
        self.aeb_escape_heading_cone_rad = CONE
        self._aeb_escape_yaw = None
        self.pub_fgm_prefer = _Pub()

    def _aeb_escape_active(self) -> bool:
        return self._escaping

    def get_logger(self):
        return _Log()


def test_nothing_is_sent_while_driving_normally():
    """평소엔 빈 배열 — FGM 은 원래대로 정면을 선호한다."""
    p = _Planner(escaping=False)
    p._update_escape_heading(_pose(30.0))
    assert p.pub_fgm_prefer.last == []
    assert p._aeb_escape_yaw is None


def test_stopping_latches_the_heading():
    p = _Planner()
    p._update_escape_heading(_pose(30.0))
    assert p._aeb_escape_yaw == pytest.approx(math.radians(30.0))
    angle, cone = p.pub_fgm_prefer.last
    # 막 멈춘 순간이라 기억한 방향이 곧 정면이다
    assert angle == pytest.approx(0.0, abs=1e-6)
    assert cone == pytest.approx(CONE)


def test_turning_away_pulls_the_reference_back():
    """왼쪽으로 20° 돌았으면 기준각이 오른쪽 20° 로 간다 — 되돌아오라는 뜻."""
    p = _Planner()
    p._update_escape_heading(_pose(30.0))
    p._update_escape_heading(_pose(50.0))
    assert p.pub_fgm_prefer.last[0] == pytest.approx(math.radians(-20.0), abs=1e-6)


def test_the_latch_does_not_drift():
    """돌아가는 동안 기준이 따라가면 제한이 아무 의미가 없다."""
    p = _Planner()
    for yaw in (30.0, 45.0, 70.0, 95.0):
        p._update_escape_heading(_pose(yaw))
    assert p._aeb_escape_yaw == pytest.approx(math.radians(30.0))
    assert p.pub_fgm_prefer.last[0] == pytest.approx(math.radians(-65.0), abs=1e-6)


def test_wraps_the_short_way_around():
    """±180° 근처에서 반대로 돌라고 하면 안 된다."""
    p = _Planner()
    p._update_escape_heading(_pose(170.0))
    p._update_escape_heading(_pose(-170.0))
    assert p.pub_fgm_prefer.last[0] == pytest.approx(math.radians(-20.0), abs=1e-6)


def test_a_new_stop_latches_a_new_heading():
    p = _Planner()
    p._update_escape_heading(_pose(30.0))
    p._escaping = False
    p._update_escape_heading(_pose(80.0))
    assert p._aeb_escape_yaw is None
    p._escaping = True
    p._update_escape_heading(_pose(80.0))
    assert p._aeb_escape_yaw == pytest.approx(math.radians(80.0))


def test_no_pose_means_no_reference():
    """자세를 모르면 기준을 못 세운다 — 예전 동작으로 두고 래치도 안 건다."""
    p = _Planner()
    p._update_escape_heading(None)
    assert p.pub_fgm_prefer.last == []
    assert p._aeb_escape_yaw is None


def test_the_lock_can_be_turned_off():
    p = _Planner(lock=False)
    p._update_escape_heading(_pose(30.0))
    assert p.pub_fgm_prefer.last == []


# ── FGM: 콘 밖으로는 조준하지 않는다 ────────────────────────────────────


clamp = FGMNode._clamp_to_cone


def test_a_gap_inside_the_cone_is_untouched():
    lo, hi = math.radians(-10.0), math.radians(20.0)
    assert clamp(lo, hi, 0.0, CONE) == (lo, hi)


def test_a_gap_hanging_out_of_the_cone_gets_clipped():
    lo, hi = math.radians(30.0), math.radians(80.0)
    got_lo, got_hi = clamp(lo, hi, 0.0, CONE)
    assert got_lo == pytest.approx(lo)
    assert got_hi == pytest.approx(CONE)


@pytest.mark.parametrize("lo_deg,hi_deg,sign", [(70.0, 80.0, +1), (-80.0, -70.0, -1)])
def test_a_gap_fully_outside_collapses_to_the_cone_edge(lo_deg, hi_deg, sign):
    """갭이 통째로 콘 밖이면 **콘 쪽 끝** 을 쓴다. 갭 끝이 아니다.

    장애물 바로 앞에 멈추면 버블이 정면을 통째로 덮어 갭이 FOV 끝에만 남는다.
    이게 AEB 정지의 기본 상황이라, 여기서 갭 끝을 따라가면 콘이 아무것도
    제한하지 못하고 옆으로 도는 예전 동작 그대로가 된다.
    """
    lo, hi = clamp(math.radians(lo_deg), math.radians(hi_deg), 0.0, CONE)
    assert lo == hi == pytest.approx(sign * CONE)


def test_the_cone_follows_the_reference():
    """기준이 -20° 면 허용 범위도 통째로 그만큼 돈다."""
    ref = math.radians(-20.0)
    lo, hi = clamp(math.radians(-80.0), math.radians(80.0), ref, CONE)
    assert lo == pytest.approx(ref - CONE)
    assert hi == pytest.approx(ref + CONE)


# ── FGM: 조준각이 기준을 따라간다 ───────────────────────────────────────


class _Picker:
    def __init__(self, clear_fn, bias: float = 1.0):
        self._clear_fn = clear_fn
        self.corridor_angle_samples = 21
        self.corridor_straight_bias = bias

    def _corridor_clear_distance(self, _g, _w, angle):
        return self._clear_fn(angle)

    pick = FGMNode._pick_target_angle


# 장애물 코앞에 멈춘 상황: 버블이 정면을 통째로 덮어 갭이 FOV 끝에만 남는다.
EDGE_GAP = (math.radians(62.0), math.radians(80.0))


def _open_to_the_left(angle: float) -> float:
    return min(5.0, max(0.3, 6.0 * angle))


def test_without_the_cone_the_aim_sits_out_at_the_gap():
    """전제 확인: 갭이 옆에만 있으면 조준도 거기로 간다 (벽으로 돈 동작).

    조준각은 검증된 갭 [lo, hi] 안으로 갇히므로, 정면 선호 벌점이 아무리
    세도 갭이 62° 부터면 62° 밑으로는 못 내려온다.
    """
    lo, hi = EDGE_GAP
    angle, _ = _Picker(_open_to_the_left).pick(None, None, lo, hi, lo, 2.5)
    assert math.degrees(angle) >= 62.0


def test_the_cone_pulls_the_aim_back_toward_the_stopped_heading():
    lo, hi = clamp(*EDGE_GAP, 0.0, CONE)
    angle, _ = _Picker(_open_to_the_left).pick(
        None, None, lo, hi, lo, 2.5, bias_ref=0.0
    )
    assert math.degrees(angle) == pytest.approx(55.0)


def test_as_the_car_turns_the_cone_keeps_pulling_it_back():
    """탈출 중 회전이 스스로 멎는지 — 이게 핵심이다.

    차가 왼쪽으로 돌면 (1) 기준각이 오른쪽으로 같은 만큼 가고 (2) 세상에
    고정된 갭도 차체 기준으로는 같은 만큼 내려온다. 그래서 허용 상한
    `기준+콘` 이 회전량만큼 계속 낮아지고 조준각이 따라 내려온다.
    """
    gap_world_lo, gap_world_hi = EDGE_GAP
    aims = []
    for turned_deg in (0.0, 20.0, 40.0, 60.0):
        t = math.radians(turned_deg)
        ref = -t  # 기억한 헤딩을 지금 차체 기준으로 본 각도
        lo, hi = clamp(gap_world_lo - t, gap_world_hi - t, ref, CONE)
        angle, _ = _Picker(_open_to_the_left).pick(
            None, None, lo, hi, max(lo, min(hi, ref)), 2.5, bias_ref=ref
        )
        aims.append(math.degrees(angle))
    assert aims == sorted(aims, reverse=True), f"조준각이 안 줄어든다: {aims}"
    assert aims[-1] <= 0.0, f"60° 돌았는데 아직 왼쪽을 겨냥한다: {aims[-1]:.0f}°"


def test_the_penalty_is_measured_from_the_reference_not_from_straight():
    """기준이 -30° 면 -30° 조준에 벌점이 없어야 한다.

    벌점을 정면 기준으로 재면, 되돌아가라고 준 기준각 자체가 손해를 봐서
    차가 돌아간 자리에 그대로 머문다.
    """
    ref = math.radians(-30.0)
    lo, hi = math.radians(-60.0), math.radians(60.0)
    angle, _ = _Picker(lambda _a: 4.0).pick(None, None, lo, hi, ref, 2.5, bias_ref=ref)
    assert angle == pytest.approx(ref)


def test_default_reference_is_straight_ahead():
    """평소 동작은 그대로 — bias_ref 를 안 주면 예전과 같다."""
    lo, hi = math.radians(-60.0), math.radians(60.0)
    angle, _ = _Picker(lambda _a: 4.0).pick(None, None, lo, hi, 0.0, 2.5)
    assert angle == pytest.approx(0.0)


# ── FGM: 갭 선택도 기준을 따른다 ────────────────────────────────────────


class _Selector:
    """빔 인덱스 i 를 각도 i·STEP 으로 두고 갭 선택만 돌린다.

    히스테리시스는 인덱스가 아니라 **각도** 로 직전 갭을 기억한다 — FOV 가
    속도에 따라 변하면 work 배열 인덱스가 다른 각도를 가리키기 때문이다.
    """

    STEP = 0.01  # rad/bin

    def __init__(self, last: int | None = None):
        self.min_gap_bins = 4
        self.hyst_ratio = 0.78
        self._last_gap_center_angle = None if last is None else last * self.STEP

    select = FGMNode._select_gap

    def pick(self, gaps, max_len, **kw):
        n = max(int(g[-1]) for g in gaps) + 2
        angles = np.arange(n, dtype=float) * self.STEP
        return self.select(gaps, max_len, work_angles=angles, **kw)


def _gap(a: int, b: int) -> np.ndarray:
    return np.arange(a, b)


def test_lock_ignores_the_gap_it_was_following():
    """탈출 중엔 한 번 문 옆 갭에 계속 끌려가면 안 된다."""
    near = _gap(48, 60)
    far = _gap(100, 140)
    assert _Selector(last=120).pick([near, far], 40, aim_idx=50, lock=True) is near


def test_without_lock_the_hysteresis_still_wins():
    """평소 주행에서는 갭 튐 방지가 그대로 살아 있어야 한다."""
    near = _gap(48, 60)
    far = _gap(100, 140)
    assert _Selector(last=120).pick([near, far], 40, aim_idx=50) is far
