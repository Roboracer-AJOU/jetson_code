"""고속에서 FGM 이 낼 수 없는 각을 고르지 않게.

회피는 전부 FGM 이 한다. 그런데 FGM 은 갭만 보고 각을 고르므로 저속에서
정답인 45~60° 가 고속에서도 그대로 나온다. 그 각은

  1. 그대로 요구 조향이 되는데 타이어가 못 낸다 — 차가 그 방향으로 밀린다.
  2. `_avoid_target_speed` 의 maneuver 항이 "그 조향을 낼 수 있는 속도" 로
     답하면서 속도까지 깎는다. 실측 0.1 m/s — 회피하려다 장애물 앞에서 선다.

그래서 `fov_narrow_speed`(4 m/s) 위에서는 FOV 를 낼 수 있는 각까지 좁히고,
그래도 남는 경우를 위해 플래너 쪽에 속도 하한을 둔다.

    python3 -m pytest src/path_following/test/test_fgm_speed_gate.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.fgm_node import FGMNode  # noqa: E402


class _Fgm:
    """`_fov_for_speed` 만 떼어 온 가짜 FGM 노드."""

    _fov_for_speed = FGMNode._fov_for_speed

    def __init__(self, v: float, *, enable: bool = True, full: bool = False):
        self._ego_speed = v
        self._use_full_scan_fov = full
        self.fov_speed_narrow = enable
        self.fov_angle = math.radians(80.0)
        self.fov_narrow_speed = 4.0
        self.fov_narrow_blend = 1.0
        self.fov_narrow_a_lat = 4.5
        self.fov_half_min = math.radians(12.0)
        self.target_lead_time_s = 0.70
        self.target_min_m = 1.0


def _fov(v: float, **kw) -> float:
    return math.degrees(_Fgm(v, **kw)._fov_for_speed())


# --------------------------------------------------------- 저속은 그대로 둔다


def test_low_speed_keeps_the_wide_view():
    """저속 FGM 은 넓은 각이 있어야 막힌 곳에서 빠져나온다."""
    for v in (0.0, 1.0, 2.5, 4.0):
        assert _fov(v) == 80.0, v


def test_the_switch_turns_it_all_off():
    for v in (5.0, 7.0):
        assert _fov(v, enable=False) == 80.0


def test_full_scan_mode_is_untouched():
    assert _fov(7.0, full=True) == 80.0


# ------------------------------------------------------------- 고속은 좁힌다


def test_racing_speed_narrows_the_view():
    assert _fov(5.0) < 25.0
    assert _fov(6.0) < 20.0
    assert _fov(7.0) < 18.0


def test_the_view_keeps_shrinking_with_speed():
    seq = [_fov(v) for v in (4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0)]
    assert seq == sorted(seq, reverse=True)


def test_the_narrow_edge_is_an_angle_the_car_can_hold():
    """되짚어 계산: 순수추종 κ=2·sinψ/L 이 횡가속 예산 안이어야 한다."""
    for v in (5.0, 6.0, 7.0):
        n = _Fgm(v)
        psi = n._fov_for_speed()
        lead = max(n.target_min_m, v * n.target_lead_time_s)
        assert v * v * 2.0 * math.sin(psi) / lead <= n.fov_narrow_a_lat + 1e-6


def test_it_never_closes_past_the_floor():
    for v in (8.0, 10.0, 20.0):
        assert _fov(v) >= 12.0 - 1e-9


def test_it_never_opens_wider_than_configured():
    for v in (0.0, 3.0, 4.5, 9.0):
        assert _fov(v) <= 80.0 + 1e-9


# --------------------------------------------------------------- 이어져 있다


def test_the_narrowing_comes_on_without_a_step():
    """문턱에서 FOV 가 튀면 조준각도 같이 튄다."""
    xs = [3.6 + 0.05 * k for k in range(40)]
    ys = [_fov(v) for v in xs]
    assert max(abs(b - a) for a, b in zip(ys, ys[1:])) < 4.0
    assert ys[0] == 80.0
    assert ys[-1] < 25.0


def test_reverse_speed_uses_magnitude():
    assert _fov(-7.0) == _fov(7.0)
