"""장애물 앞에 **도착했을 때 이미** 순항속도여야 한다.

인지 시점부터 감속을 시작하므로, 인지거리(`avoid_on` 게이트)가 감속거리보다
길어야 성립한다. 짧으면 장애물 앞에서 급감속이 나고, 그건 AEB 에서만 나야 한다.

    감속거리 = (v² - v_target²) / (2·a_brake)

    python3 -m pytest src/path_following/test/test_avoid_preempt_decel.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.local_planner_node import CFG  # noqa: E402

A_BRAKE = CFG["avoid_a_brake_mps2"]
HIGH = CFG["avoid_cruise_speed_high_mps"]
LOW = CFG["avoid_cruise_speed_low_mps"]
TH = CFG["avoid_cruise_high_speed_th"]


def _on_gate(v: float) -> float:
    """`_effective_avoid_gates` 의 avoid_on 계산을 그대로 옮겨 온 것."""
    scale = CFG["avoid_timing_margin"] * (max(v, 0.5) / CFG["avoid_timing_ref_mps"])
    on = max(
        CFG["avoid_on_min_m"], min(CFG["avoid_on_max_m"], CFG["avoid_on_m"] * scale)
    )
    w = (v - CFG["avoid_on_late_max_speed"]) / CFG["avoid_on_late_blend_mps"]
    w = min(1.0, max(0.0, w))
    return on * (CFG["avoid_on_late_scale"] + w * (1.0 - CFG["avoid_on_late_scale"]))


def _target(v: float) -> float:
    """`_avoid_cruise_target` 이 진입 속도로 고르는 값."""
    return HIGH if v > TH else LOW


def _brake_dist(v: float) -> float:
    tgt = _target(v)
    if v <= tgt:
        return 0.0
    return (v * v - tgt * tgt) / (2.0 * A_BRAKE)


def test_there_is_room_to_slow_down_at_every_speed():
    for tenth in range(10, 81):
        v = tenth / 10.0
        assert _on_gate(v) >= _brake_dist(v), f"{v} m/s 에서 인지거리가 모자라다"


def test_the_worst_case_still_has_margin():
    """가장 빡빡한 지점에도 여유가 있어야 한다 — 실차는 계산대로 안 선다."""
    worst = min(_on_gate(t / 10.0) - _brake_dist(t / 10.0) for t in range(10, 81))
    assert worst > 1.0, f"여유 {worst:.2f} m"


def test_top_speed_is_covered():
    v = 8.0
    assert _on_gate(v) >= _brake_dist(v)


def test_slow_traffic_needs_no_room():
    assert _brake_dist(LOW) == 0.0, "이미 저속 목표다"
    assert _brake_dist(1.5) == 0.0


def test_the_step_at_the_threshold_is_covered():
    """문턱 바로 아래는 목표가 한 단 낮아 감속거리가 더 필요하다."""
    just_under = TH - 0.1
    assert _target(just_under) == LOW
    assert _on_gate(just_under) >= _brake_dist(just_under)


def test_the_ramp_is_gentle():
    """급감속은 AEB 뿐이다. 회피 감속은 a_brake 로 묶인다."""
    assert A_BRAKE <= 3.5


def test_it_needs_less_room_than_a_full_stop():
    """3 m/s 까지만 줄이면 되니 정지보다 짧다 — 이게 감속을 줄이는 지점이다."""
    v = 7.0
    to_stop = v * v / (2.0 * A_BRAKE)
    # 1 - (3/7)² ≈ 0.82
    assert _brake_dist(v) < to_stop * 0.85
