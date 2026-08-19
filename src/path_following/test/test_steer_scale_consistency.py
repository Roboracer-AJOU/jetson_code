#!/usr/bin/env python3
"""조향 체인의 **실효 배율**을 고정한다.

    python3 -m pytest src/path_following/test/test_steer_scale_consistency.py -q

조향은 두 노드를 거치고, 각 노드가 배율을 하나씩 갖고 있다.

    Stanley   게인 재환산 ×0.428 (steer_scale_calibrated 가 켜졌을 때)
    control   ÷ max_steering_angle_rad
    ESP       S=±1 → 서보 ±50°

여기서 놓치기 쉬운 점: **앞의 두 배율은 서로 상쇄된다.** 그래서

    (보정ON  + 0.3735)  와  (보정OFF + 0.8727)

은 단위 표기만 다를 뿐 물리 출력이 완전히 같다. "단위가 맞는 조합" 이 둘이라
단위 일치만 검사하면 아무것도 못 잡는다.

반대로 (보정ON + 0.8727) 은 0.428 이다. 요구한 전륜각의 43% 만 나간다.
저속에서는 피드백이 메워서 티가 안 나고, 고속에서 요구 조향각이 커지면
풀락에 걸려 라인을 벗어난다. 한쪽 값만 옮기면 이 조합에 빠진다.

그래서 검사할 것은 분모가 아니라 곱해진 결과다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.control_node import CFG as CONTROL_CFG  # noqa: E402
from path_following.stanley_waypoint_follow_node import CFG as STANLEY_CFG  # noqa: E402

# ESP normToAngle: S=±1 → 서보 40°/140°, 즉 중앙에서 ±50°.
ESP_SERVO_FULL_RAD = math.radians(50.0)

# 요구한 전륜각이 그대로 나가는 값.
TUNED_DELIVERY = 1.0

# 분모만 서보각으로 잘못 두었을 때 나오는 값. 회귀 감시용.
STARVED_DELIVERY = 0.428


def gain_rebase() -> float:
    """Stanley 가 게인에 곱하는 값."""
    if not STANLEY_CFG["steer_scale_calibrated"]:
        return 1.0
    return float(STANLEY_CFG["max_steering_angle_real_rad"]) / float(
        STANLEY_CFG["max_steering_angle"]
    )


def delivery() -> float:
    """체인 전체 배율. 운동학 정확 = 1.0 기준."""
    return (
        gain_rebase() / float(CONTROL_CFG["max_steering_angle_rad"])
    ) * float(STANLEY_CFG["max_steering_angle"])


def test_effective_delivery_matches_the_tuned_value():
    assert delivery() == pytest.approx(TUNED_DELIVERY, abs=0.02), (
        f"조향 실효 배율 {delivery():.3f} — 튜닝값 {TUNED_DELIVERY} 에서 벗어났다. "
        "Stanley 게인과 control_node 분모를 같이 옮겼는지 확인할 것"
    )


def test_either_consistent_combination_gives_the_same_output():
    """단위가 맞는 조합은 둘이고, 둘의 물리 출력은 같다."""
    servo_full = float(STANLEY_CFG["max_steering_angle"])
    real_full = float(STANLEY_CFG["max_steering_angle_real_rad"])
    calibrated = (real_full / servo_full) / real_full * servo_full
    uncalibrated = 1.0 / servo_full * servo_full
    assert calibrated == pytest.approx(uncalibrated), "두 조합은 같은 출력이어야 한다"
    assert calibrated == pytest.approx(TUNED_DELIVERY, abs=1e-6)


def test_mixing_the_two_combinations_starves_the_steering():
    """보정ON 인데 분모를 서보각으로 두면 조향이 43% 로 깎인다."""
    servo_full = float(STANLEY_CFG["max_steering_angle"])
    real_full = float(STANLEY_CFG["max_steering_angle_real_rad"])
    mixed = (real_full / servo_full) / servo_full * servo_full
    assert mixed == pytest.approx(STARVED_DELIVERY, abs=0.002)
    # 이 상태로는 풀락을 명령해도 전륜이 절반도 안 꺾인다.
    wheel = math.degrees(mixed * real_full)
    assert wheel < math.degrees(real_full) * 0.5


def test_full_lock_command_maps_to_a_reachable_servo_angle():
    """Stanley 풀락이 서보 가동범위를 넘지 않아야 한다."""
    out_full = (
        float(STANLEY_CFG["max_steering_angle_real_rad"])
        if STANLEY_CFG["steer_scale_calibrated"]
        else float(STANLEY_CFG["max_steering_angle"])
    )
    s_cmd = out_full / float(CONTROL_CFG["max_steering_angle_rad"])
    assert 0.0 < s_cmd <= 1.0, f"풀락에서 S={s_cmd:.3f} — 서보 범위를 넘는다"


def test_wheel_angle_at_full_lock_is_physically_sane():
    """풀락이 실제로 만드는 전륜각이 차량 한계 안이어야 한다."""
    servo_full = float(STANLEY_CFG["max_steering_angle"])
    real_full = float(STANLEY_CFG["max_steering_angle_real_rad"])
    servo_to_wheel = real_full / servo_full
    out_full = real_full if STANLEY_CFG["steer_scale_calibrated"] else servo_full
    s_cmd = min(1.0, out_full / float(CONTROL_CFG["max_steering_angle_rad"]))
    wheel = math.degrees(s_cmd * ESP_SERVO_FULL_RAD * servo_to_wheel)
    assert 0.0 < wheel <= math.degrees(real_full) + 1e-6, (
        f"풀락 전륜각 {wheel:.1f}° 가 실측 한계 {math.degrees(real_full):.1f}° 를 넘는다"
    )
