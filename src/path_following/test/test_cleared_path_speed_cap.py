"""벽에서 잘린 회피 경로는 버리는 게 아니라 속도로 갚는다.

실측(20260822, `debug/lap3.csv` + 런치 로그)에서 회피가 무너진 경로다.

    원인=벽 (벽 17, 장애물 48)  확보 2.10 m < 요구 2.14 m  (v=3.2)
    원인=벽 (벽 10, 장애물 42)  확보 1.05 m < 요구 1.09 m  (v=2.2)
    원인=벽 (벽 21, 장애물 55)  확보 2.70 m < 요구 4.18 m  (v=4.6)

세 건 다 **장애물은 훨씬 뒤에서야 걸린다** — 장애물을 제대로 피하는
경로였는데 벽에서 잘렸고, 그 중 둘은 요구치에 4 cm 모자라 버려졌다.
버린 뒤 남는 건 CSV 이고, 회피에 들어간 이유가 라인 위 장애물이므로
CSV 는 정의상 그 장애물을 향한다. 실제로 4.6 m/s 에서 0.46 초 직진하다
AEB 가 받았다.
"""

from __future__ import annotations

import math

import pytest

from path_following.local_planner_node import LocalPlannerNode


class _Stub:
    """속도 상한 계산에 필요한 것만 들고 있는 껍데기."""

    class _P:
        a_brake = 3.0

    def __init__(self, cleared: float, enable: bool = True):
        self._cleared_len_m = cleared
        self.wall_stop_check_enable = enable
        self.wall_stop_reaction_sec = 0.15
        self.avoid_speed_params = self._P()
        self._ego_speed_mps = 0.0

    _cleared_path_speed_limit = LocalPlannerNode._cleared_path_speed_limit
    _wall_stop_distance_m = LocalPlannerNode._wall_stop_distance_m


@pytest.mark.parametrize("cleared,v_when_rejected", [(2.10, 3.2), (1.05, 2.2), (2.70, 4.6)])
def test_the_measured_rejections_become_speed_caps(cleared, v_when_rejected):
    """버려졌던 세 경로가 이제는 통과하고, 대신 속도가 잡힌다."""
    p = _Stub(cleared)
    cap = p._cleared_path_speed_limit()

    # 상한은 그때 속도보다 낮다 — 감속은 여전히 일어난다
    assert cap < v_when_rejected

    # 그리고 그 상한에서는 확보 길이 안에 정확히 선다
    p._ego_speed_mps = cap
    assert p._wall_stop_distance_m() == pytest.approx(cleared, rel=1e-6)


def test_the_cap_is_the_inverse_of_the_stopping_distance():
    """`_wall_stop_distance_m` 와 서로 역함수여야 한다.

    둘이 어긋나면 "받아들인 경로인데 그 속도로는 못 선다" 가 생긴다.
    """
    p = _Stub(float("inf"))
    for v in (0.5, 1.0, 2.0, 3.5, 5.0, 7.0):
        p._ego_speed_mps = v
        p._cleared_len_m = p._wall_stop_distance_m()
        assert p._cleared_path_speed_limit() == pytest.approx(v, rel=1e-9)


def test_an_uncut_path_has_no_cap():
    """안 잘린 경로는 상한이 없다 — 회피가 끝나면 CSV 속도로 돌아간다."""
    assert math.isinf(_Stub(float("inf"))._cleared_path_speed_limit())


def test_the_cap_is_off_when_the_check_is_off():
    assert math.isinf(_Stub(1.0, enable=False)._cleared_path_speed_limit())


def test_a_shorter_clearance_never_allows_more_speed():
    caps = [_Stub(L)._cleared_path_speed_limit() for L in (0.6, 1.0, 2.0, 4.0, 8.0)]
    assert caps == sorted(caps)


def test_the_gate_releases_when_the_planner_stops_publishing():
    """override 를 내리면 상한이 풀려야 한다.

    안 풀면 회피가 끝난 뒤에도 옛 상한이 남아 CSV 속도를 계속 막는다.
    """

    class _Pub:
        def publish(self, _msg):
            pass

    class _Gate:
        _cleared_len_m = 1.0
        _override_active = True
        pub_override_gate = _Pub()

        def _set_path_planned(self, v):
            self.planned = v

        _publish_override_gate = LocalPlannerNode._publish_override_gate

    g = _Gate()
    g._publish_override_gate(False)
    assert math.isinf(g._cleared_len_m)
