#!/usr/bin/env python3
"""횡가속 피드백 상한이 회피/복귀에만 걸리는지 검증.

    python3 -m pytest src/path_following/test/test_feedback_lateral_cap_scope.py -q

설계 의도:

  * CSV_TRACKING (깨끗한 트랙)
      곡선은 곡률 FF 가 처리하고, 피드백(heading+CTE)은 남은 오차를 정확히
      지우는 역할이다. 여기에 횡가속 상한을 걸면 안 된다. 상한은 v² 에
      반비례해서 6 m/s 에 2.1°, 7 m/s 에 1.5° 까지 조이는데, 그러면 라인을
      아예 못 따라간다.

  * LOCAL_PATH (회피 + 복귀)
      장애물 옆을 지나느라 경로에서 벌어진 상태다. 헤딩을 되돌리려고 조향을
      크게 넣으면 벽으로 간다. 조향은 최소로 쓰고 미리 피하거나 속도를 줄이는
      쪽이 맞다. 그래서 여기서만 접지력 예산으로 묶는다.

REJOIN 도 planner override gate 를 켜므로 Stanley 에서 LOCAL_PATH 로 들어온다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.stanley_waypoint_follow_node import CFG  # noqa: E402


class _Steer:
    """`_stanley_control` 의 상한 부분만 떼어낸 스텁."""

    def __init__(self) -> None:
        self.wheelbase = float(CFG["wheelbase"])
        self.feedback_lateral_accel_mps2 = float(CFG["feedback_lateral_accel_mps2"])
        self.max_lateral_accel_mps2 = float(CFG["max_lateral_accel_mps2"])
        self.max_steering = float(CFG["max_steering_angle_real_rad"])

    def _steering_for_lateral_accel(self, a_max: float, speed: float):
        v = abs(speed)
        if a_max <= 0.0 or v < 0.5:
            return None
        return math.atan(self.wheelbase * a_max / (v * v))

    def feedback_out(self, fb_raw: float, speed: float, mode: str) -> float:
        """실제 노드와 같은 순서로 피드백 항에 상한을 건다."""
        fb_limit = (
            self._steering_for_lateral_accel(self.feedback_lateral_accel_mps2, speed)
            if mode == "LOCAL_PATH" and self.feedback_lateral_accel_mps2 > 0.0
            else None
        )
        if fb_limit is not None and abs(fb_raw) > fb_limit:
            return math.copysign(fb_limit, fb_raw)
        return fb_raw


# 상한이 실제로 물리는 속도대. 1 m/s 는 상한이 풀락보다 커서 무의미하다.
_FAST = (2.0, 3.0, 4.0, 5.0, 6.0, 7.0)


@pytest.mark.parametrize("v", _FAST)
def test_clean_track_feedback_is_not_capped(v: float):
    """CSV 추종에서는 피드백이 요청한 만큼 그대로 나가야 한다."""
    n = _Steer()
    want = n.max_steering  # 풀락을 요청해도 이 모드에선 안 잘린다
    assert n.feedback_out(want, v, "CSV_TRACKING") == pytest.approx(want)


@pytest.mark.parametrize("v", _FAST)
def test_avoidance_feedback_is_capped(v: float):
    """회피/복귀에서는 접지력 예산으로 잘려야 한다.

    풀락을 요청한다. 2 m/s 예산이 18.3° 라 21.4° 풀락은 전 구간에서 물린다.
    """
    n = _Steer()
    want = n.max_steering
    got = n.feedback_out(want, v, "LOCAL_PATH")
    budget = math.atan(n.wheelbase * n.feedback_lateral_accel_mps2 / (v * v))
    assert got == pytest.approx(budget)
    assert got < want, f"{v} m/s 에서 상한이 안 걸렸다"


def test_cap_would_have_crippled_clean_track_tracking():
    """회귀 방지: CSV 에 걸었을 때 얼마나 조였는지 숫자로 남긴다."""
    n = _Steer()
    capped = n.feedback_out(n.max_steering, 6.0, "LOCAL_PATH")
    assert math.degrees(capped) < 2.5, "6 m/s 상한은 2.1° 수준이어야 한다"


def test_low_speed_is_never_capped_in_either_mode():
    """0.5 m/s 미만은 상한 계산 자체를 건너뛴다."""
    n = _Steer()
    want = math.radians(15.0)
    for mode in ("CSV_TRACKING", "LOCAL_PATH"):
        assert n.feedback_out(want, 0.3, mode) == pytest.approx(want)


def test_rejoin_budget_stays_below_cornering_budget():
    """복귀 기동은 코너링보다 언제나 완만해야 한다."""
    n = _Steer()
    assert n.feedback_lateral_accel_mps2 < n.max_lateral_accel_mps2
