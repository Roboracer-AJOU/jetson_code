"""FGM 이 "필요한 만큼만" 트는지 — 과회피 회귀 테스트.

실차에서 r=0.20 콘 하나에 라인에서 1.2~1.5 m 씩 벗어났다 (필요량 0.45 m 의
2.5~3.3 배). 회피 중 경로 추종오차는 네 번 모두 +0.33 m 로 흔들림이 없었으니
차가 넘친 게 아니라 **경로가 크게 그려진** 것이었다. 원인은 두 곳이었다.

1. `_pick_target_angle` 의 `want` 에 목표점 거리(최대 5 m)가 들어갔다.
   보상 `min(clear, want)` 이 5 m 까지 안 꺾이니, 45° 의 벌점 0.79 m 를
   여유 증가분이 항상 이겼다.
2. `_select_gap` 이 각도상 제일 넓은 갭을 골랐다. 트랙에서 그건 보통
   레이스라인이 아니라 건너편이다.
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


# ----------------------------------------------------------- want 포화점


class _Want:
    def __init__(self, v: float, **kw):
        self._ego_speed = v
        self.corridor_want_time_s = kw.get("t", 0.5)
        self.corridor_want_min_m = kw.get("lo", 1.5)
        self.corridor_want_max_m = kw.get("hi", 3.0)

    want = FGMNode._corridor_want_m


def test_want_is_bounded_well_below_target_distance():
    """어떤 속도에서도 목표점 거리(최대 5 m)까지 올라가면 안 된다."""
    for v in (0.0, 1.0, 3.0, 6.0, 9.0, 20.0):
        w = _Want(v).want()
        assert 1.5 <= w <= 3.0, f"v={v} 에서 want={w}"


def test_want_grows_with_speed_between_bounds():
    assert _Want(2.0).want() < _Want(5.0).want()


# ------------------------------------------------- 각도 선정 (핵심 회귀)


class _Picker:
    """`_pick_target_angle` 만 떼어낸 가짜 노드.

    코리도 여유는 실제 스캔 대신 `clear_fn(angle)` 로 준다 — 이 테스트가
    보려는 건 기하가 아니라 **점수식이 어느 각도를 고르는가** 이다.
    """

    def __init__(self, clear_fn, samples: int = 11, bias: float = 1.0):
        self._clear_fn = clear_fn
        self.corridor_angle_samples = samples
        self.corridor_straight_bias = bias

    def _corridor_clear_distance(self, _g, _w, angle):
        return self._clear_fn(angle)

    pick = FGMNode._pick_target_angle


def _curved_corridor(angle: float) -> float:
    """굽은 통로: 정면은 곧 막히고 옆으로 틀수록 멀리 뚫린다."""
    return min(5.0, 1.8 + 4.0 * abs(angle))


LO, HI = math.radians(-80.0), math.radians(80.0)


def test_old_want_swung_the_car_wide():
    """회귀 재현: want 에 목표점 거리를 넣으면 크게 튼다."""
    n = _Picker(_curved_corridor)
    angle, _ = n.pick(None, None, LO, HI, 0.0, 5.0)
    # 좌우 대칭 통로라 어느 쪽으로 트는지는 임의다. 크기만 본다.
    assert abs(math.degrees(angle)) > 25.0, "예전 과회피가 재현되지 않는다"


def test_bounded_want_keeps_the_car_near_straight():
    n = _Picker(_curved_corridor)
    angle, _ = n.pick(None, None, LO, HI, 0.0, 2.5)
    assert abs(math.degrees(angle)) <= 20.0, (
        f"여유가 충분한데도 {math.degrees(angle):.0f}° 나 텄다"
    )


def test_still_turns_when_straight_is_actually_blocked():
    """정면이 진짜 막히면 여전히 튼다 — 소극적으로 만든 게 아니다."""

    def blocked_ahead(angle: float) -> float:
        return 0.3 if abs(angle) < math.radians(20.0) else 4.0

    n = _Picker(blocked_ahead)
    angle, clear = n.pick(None, None, LO, HI, 0.0, 2.5)
    assert abs(math.degrees(angle)) >= 20.0, "막힌 정면을 그대로 겨냥했다"
    assert clear > 1.0


def test_returns_preferred_immediately_when_clear_enough():
    n = _Picker(lambda _a: 4.0)
    angle, _ = n.pick(None, None, LO, HI, 0.0, 2.5)
    assert angle == 0.0


def test_never_leaves_the_verified_gap():
    lo, hi = math.radians(10.0), math.radians(30.0)
    n = _Picker(_curved_corridor)
    angle, _ = n.pick(None, None, lo, hi, lo, 2.5)
    assert lo - 1e-9 <= angle <= hi + 1e-9


# ------------------------------------------------------------ 갭 선택


class _Selector:
    """빔 인덱스 i 를 각도 i·STEP 으로 두고 갭 선택만 돌린다.

    히스테리시스는 인덱스가 아니라 **각도** 로 직전 갭을 기억한다 — FOV 가
    속도에 따라 변하면 work 배열 인덱스가 다른 각도를 가리키기 때문이다.
    """

    STEP = 0.01  # rad/bin

    def __init__(self, min_bins: int = 4, last: int | None = None):
        self.min_gap_bins = min_bins
        self.hyst_ratio = 0.78
        self._last_gap_center_angle = None if last is None else last * self.STEP

    select = FGMNode._select_gap

    def pick(self, gaps, max_len, **kw):
        n = max((int(g[-1]) for g in gaps), default=0) + 2
        angles = np.arange(n, dtype=float) * self.STEP
        return self.select(gaps, max_len, work_angles=angles, **kw)


def _gap(a: int, b: int) -> np.ndarray:
    return np.arange(a, b)


def test_regression_the_history_survives_a_changing_fov():
    """FOV 가 바뀌면 work 인덱스 공간이 통째로 밀린다.

    갭 탐색은 FOV 안 빔만 모아 work 배열을 만든다. 속도에 따라 FOV 가
    좁아지면 같은 갭이 다른 인덱스에 앉는데, 직전 갭을 **인덱스**로 기억하면
    그 순간 엉뚱한 쪽이 "가깝다" 고 나온다 — 좌우로 방황하는 원인이었다.

    같은 각도의 갭 두 개를, FOV 가 좁아져 인덱스가 20 밀린 다음 프레임에서
    다시 고른다. 각도로 기억하면 따라가던 쪽을 그대로 문다.
    """
    step = _Selector.STEP
    # 이전 프레임: 좁은 FOV. 따라가던 갭이 work 인덱스 30 에 있었다.
    old_idx, shift = 30, 100
    # 이번 프레임: 감속으로 FOV 가 열려 앞쪽에 빔 100 개가 더 붙었다.
    # 같은 각도의 그 갭은 이제 인덱스 130 이다.
    right = _gap(shift + old_idx - 20, shift + old_idx + 21)  # 중심 130
    left = _gap(40, 81)  # 중심 60 — 옛 인덱스 30 에는 이쪽이 "가깝다"
    angles = (np.arange(300, dtype=float) - shift) * step

    sel = _Selector()
    sel._last_gap_center_angle = old_idx * step  # 각도는 안 변한다

    chosen = sel.select([left, right], 41, aim_idx=130, work_angles=angles)
    assert chosen is right, "각도로 기억하면 따라가던 쪽을 그대로 문다"

    # 옛 방식(인덱스 비교)이었다면 왼쪽으로 튀었다는 것을 같이 박아 둔다.
    def old_pick(gaps, last_idx):
        return min(gaps, key=lambda g: abs(int(g[len(g) // 2]) - last_idx))

    assert old_pick([left, right], old_idx) is left


def test_picks_gap_nearest_straight_not_the_widest():
    """회귀: 콘 하나 옆의 좁은 통로 대신 트랙 건너편을 잡던 문제."""
    near = _gap(48, 56)     # 정면(50) 을 품는 좁은 갭
    far = _gap(100, 160)    # 훨씬 넓지만 멀리 있는 갭
    chosen = _Selector().pick([near, far], max_len=60, aim_idx=50)
    assert chosen is near, "제일 넓은 갭을 골랐다 (예전 동작)"


def test_prefers_wider_gap_when_equally_far_from_straight():
    left = _gap(20, 28)
    right = _gap(72, 92)   # 정면(50) 에서 같은 거리지만 더 넓다
    chosen = _Selector().pick([left, right], max_len=20, aim_idx=50)
    assert chosen is right


def test_skips_gaps_too_narrow_to_pass():
    sliver = _gap(49, 51)   # min_gap_bins 미만 — 차가 못 들어간다
    usable = _gap(70, 90)
    chosen = _Selector(min_bins=4).pick([sliver, usable], 20, aim_idx=50)
    assert chosen is usable


def test_hysteresis_still_wins_when_history_exists():
    """이력이 있으면 정면보다 '아까 따라가던 갭' 이 우선 — 갭 튐 방지."""
    near = _gap(48, 60)
    far = _gap(100, 140)
    chosen = _Selector(last=120).pick([near, far], 40, aim_idx=50)
    assert chosen is far


def test_falls_back_to_widest_without_straight_index():
    near = _gap(48, 56)
    far = _gap(100, 160)
    assert _Selector().pick([near, far], 60, aim_idx=None) is far


def test_empty_gap_list():
    assert _Selector().pick([], 0, aim_idx=50) is None


@pytest.mark.parametrize("idx,expect_near", [(50, True), (130, False)])
def test_selection_follows_where_straight_is(idx, expect_near):
    near, far = _gap(48, 56), _gap(100, 160)
    chosen = _Selector().pick([near, far], 60, aim_idx=idx)
    assert (chosen is near) is expect_near
