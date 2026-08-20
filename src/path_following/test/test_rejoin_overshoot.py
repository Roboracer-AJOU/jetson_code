#!/usr/bin/env python3
"""회피 후 라인 복귀에서 라인을 넘어 벽으로 가는 두 경로를 막는다.

    python3 -m pytest src/path_following/test/test_rejoin_overshoot.py -q

증상은 둘이었다.

  1. 복귀가 코너에 겹치면 라인을 가로질러 바깥 벽으로 간다.
  2. 벽에 가까운 라인으로 복귀할 때 관성으로 넘어가 박는다.

원인도 둘이고, 서로 다른 층에 있다.

**(1) 계획층 — 기동이 기준선을 직선으로 가정했다.**
`plan_maneuver` 의 예산 검사는 기동 자신의 |d''| 만 봤다. 코너에서는 기준선이
이미 v²κ 를 쓰고 있어서 실제 부하는 v²(|κ|+|d''|) 다. R=6 m 를 6 m/s 로 돌면
코너만 6.0 m/s² 라 접지력(5~6)에 남는 게 없는데, d'' 만 보는 검사는 통과했다.
감속도 계획 실패도 안 났다.

**(2) 제어층 — Stanley 헤딩항(=복귀 감쇠)을 고속에서 깎고 있었다.**
δ = θ_e + atan(k·e/v) 에서 θ_e 가 있어야 오차 동역학이 1차(ė = −k·e)가 되어
라인을 안 넘는다. `oppose_only_blend` 가 켜진 뒤로 억제가 걸리는 경우는 정확히
"헤딩항이 복귀를 되받는 중" 뿐이라, 남은 억제는 감쇠만 골라 깎고 있었다.

아래 폐루프 검사는 시뮬레이터가 아니라 **실제 `_stanley_control`** 을 호출한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.offset_maneuver import (  # noqa: E402
    ManeuverConfig,
    ObstacleSD,
    plan_maneuver,
)
from path_following.stanley_waypoint_follow_node import (  # noqa: E402
    CFG as SCFG,
    StanleyWaypointFollowNode,
)

MCFG = ManeuverConfig(
    half_width_m=vg.HALF_WIDTH_M,
    lateral_margin_m=0.25,
    max_offset_m=0.70,
    a_lat_enter_mps2=3.0,
    a_lat_exit_mps2=1.8,
    a_lat_hard_mps2=4.5,
    enter_min_m=1.0,
    enter_max_m=9.0,
    exit_min_m=1.5,
    exit_max_m=12.0,
    hold_front_m=vg.FRONT_M + 0.20,
    hold_rear_m=vg.LENGTH_M + 0.30,
    merge_gap_m=3.0,
    v_plan_min_mps=1.5,
    max_steer_rad=0.3735 * 0.60,
    wheelbase_m=vg.WHEELBASE_M,
)

OBST = [ObstacleSD(s=14.0, d=0.0, r=0.25)]


def _plan(v: float, radius_m: float | None, corner_at_m: float = 16.0):
    kappa = None
    if radius_m is not None:
        kappa = lambda ds: 0.0 if ds < corner_at_m else 1.0 / radius_m  # noqa: E731
    return plan_maneuver(
        OBST, MCFG, d_ego=0.0, d_ego_prime=0.0, v=v, kappa_ref=kappa
    )


# ------------------------------------------------- (1) 계획층: 코너 인식


def test_a_straight_line_plan_is_untouched():
    """직선에서는 코너 인식이 아무것도 바꾸면 안 된다.

    회피를 느리게 만드는 건 목적이 아니다. 코너와 겹칠 때만 개입해야 한다.
    """
    for v in (4.0, 5.0, 6.0, 7.0):
        naive = _plan(v, None)
        aware = plan_maneuver(
            OBST,
            MCFG,
            d_ego=0.0,
            d_ego_prime=0.0,
            v=v,
            kappa_ref=lambda ds: 0.0,
        )
        assert naive is not None and aware is not None
        assert abs(naive.exit_len_m - aware.exit_len_m) < 1e-9
        assert abs(naive.enter_len_m - aware.enter_len_m) < 1e-9
        assert naive.speed_cap_mps is None and aware.speed_cap_mps is None


def test_regression_a_corner_used_to_be_invisible_to_the_budget():
    """코너를 모르던 시절엔 예산이 남아돈다고 답했다.

    이게 벽에 박은 이유다. R=6 m 를 6 m/s 로 도는 것만으로 6.0 m/s² 인데
    계획은 3.0 이라고 보고했다.
    """
    blind = _plan(6.0, None)
    aware = _plan(6.0, 6.0)
    assert blind is not None and aware is not None

    assert blind.peak_lateral_accel_mps2 < MCFG.a_lat_hard_mps2
    assert blind.speed_cap_mps is None, "예전엔 감속을 요구하지 않았다"

    assert aware.peak_lateral_accel_mps2 > 6.0, "코너 몫이 들어와야 한다"
    assert aware.speed_cap_mps is not None, "이제는 감속으로 답해야 한다"


def test_the_corner_share_is_actually_the_corner_share():
    """보고하는 횡가속이 v²(|κ|+|d''|) 와 맞는지."""
    v, R = 6.0, 8.0
    aware = _plan(v, R)
    assert aware is not None
    corner_only = v * v / R
    # 코너 몫보다는 크고, 코너 몫 + 직선 계획의 몫보다는 작아야 한다.
    straight = _plan(v, None)
    assert straight is not None
    assert corner_only < aware.peak_lateral_accel_mps2
    assert (
        aware.peak_lateral_accel_mps2
        <= corner_only + straight.peak_lateral_accel_mps2 + 1e-6
    )


def test_a_gentle_corner_still_needs_no_slowdown():
    """모든 코너에 감속을 걸면 레이싱이 안 된다. 여유가 있으면 통과해야 한다."""
    aware = _plan(5.0, 10.0)  # 코너만 2.5 m/s²
    assert aware is not None
    assert aware.speed_cap_mps is None


def test_the_return_stretches_before_it_asks_to_slow_down():
    """먼저 완만하게 만들어 보고, 그래도 안 되면 감속이다."""
    straight = _plan(6.0, None)
    corner = _plan(6.0, 8.0)
    assert straight is not None and corner is not None
    assert corner.exit_len_m > straight.exit_len_m


def test_a_corner_speed_cap_is_a_speed_the_corner_allows():
    """돌려주는 상한이 실제로 예산 안에 드는 속도여야 한다."""
    for v, R in ((6.0, 8.0), (7.0, 6.0), (7.0, 10.0)):
        m = _plan(v, R)
        assert m is not None and m.speed_cap_mps is not None
        capped = m.speed_cap_mps
        kappa_total = m.peak_lateral_accel_mps2 / (max(v, 1e-9) ** 2)
        assert capped * capped * kappa_total <= MCFG.a_lat_hard_mps2 + 1e-6
        assert capped < v


# ------------------------------------------------- (2) 제어층: 복귀 감쇠

_G = 0.3735 / math.radians(50.0)


class _Ctl:
    """Stanley 노드의 조향 계산만 떼어 온 스텁 — 계산은 실제 코드가 한다."""

    def __init__(self, **over):
        N = StanleyWaypointFollowNode
        self.max_steering = 0.3735
        self._steer_gain_rebase = _G
        self.steer_scale_calibrated = True
        self.wheelbase = float(SCFG["wheelbase"])
        self.timer_period = 0.03
        for key in (
            "stanley_k",
            "stanley_heading_gain",
            "local_path_stanley_k",
            "local_path_heading_gain",
        ):
            setattr(self, key, float(SCFG[key]) * _G)
        for key in (
            "stanley_softening",
            "stanley_heading_cte_blend_m",
            "stanley_heading_min_weight",
            "stanley_heading_weight_speed_lo",
            "stanley_heading_weight_speed_hi",
            "local_path_cte_speed_cap_mps",
            "local_path_lookahead_m",
            "max_lateral_accel_mps2",
            "feedback_lateral_accel_mps2",
            "ff_gain",
            "ff_sign",
            "ff_lookahead_m",
            "steering_smooth_alpha",
            "local_path_steering_smooth_alpha",
        ):
            setattr(self, key, float(SCFG[key]))
        self.ff_kappa_clip = float(SCFG.get("ff_kappa_clip", 2.5))
        self.steering_rate_limit_radps = float(SCFG["steering_rate_limit_radps"]) * _G
        self.local_path_steering_rate_limit_radps = (
            float(SCFG["local_path_steering_rate_limit_radps"]) * _G
        )
        self.stanley_heading_oppose_only_blend = bool(
            SCFG["stanley_heading_oppose_only_blend"]
        )
        self.enable_steer_ff = bool(SCFG["enable_steer_ff"])
        self.ff_gain_schedule_enable = bool(SCFG["ff_gain_schedule_enable"])
        self.ff_gain_speed_bp = list(SCFG["ff_gain_speed_bp"])
        self.ff_gain_bp = list(SCFG["ff_gain_bp"])
        self._local_path_planned = True
        self._accel_hold_u = 0.0
        self._last_accel_extra = 0.0
        self._last_steering_cmd = 0.0
        self._esp_lag_enable = False
        for key, val in over.items():
            setattr(self, key, val)
        for name, fn in vars(N).items():
            if name in ("__init__", "_warn_if_wrong_way"):
                continue
            if callable(fn) and not name.startswith("__"):
                setattr(self, name, fn.__get__(self))
        self._init_ff_gain_schedule()

    def _warn_if_wrong_way(self, *a, **k):
        return None

    def get_parameter(self, name):
        holder = type("P", (), {})()
        holder.value = getattr(self, name, SCFG.get(name))
        return holder

    def get_logger(self):
        return type("L", (), {"__getattr__": lambda s, _: (lambda *a, **k: None)})()


_PATH = [(i * 0.1, 0.0) for i in range(-20, 600)]


def _line_crossing(
    v: float, d0: float, yaw0_deg: float, ctl: _Ctl, grip: float = 6.0
) -> float:
    """d0 만큼 벌어진 채 라인 쪽으로 각을 물고 도착 — 반대편으로 넘어간 최대 [m].

    접지력을 넘는 조향은 곡률로 안 바뀐다. 그게 관성이 이기는 순간이라
    여기서 잘라야 결과가 실제와 맞는다.
    """
    x, y, yaw = 0.0, d0, math.radians(yaw0_deg)
    steer = 0.0
    dt = 0.002
    every = int(round(ctl.timer_period / dt))
    worst = 0.0
    for k in range(int(5.0 / dt)):
        if k % every == 0:
            raw = ctl._stanley_control(_PATH, x, y, yaw, v, "CSV_TRACKING")[0]
            steer = ctl._rate_limit_steering(ctl._smooth_steering(raw))
        kappa = math.tan(steer) / ctl.wheelbase
        if v * v * abs(kappa) > grip:
            kappa = math.copysign(grip / (v * v), kappa)
        yaw += v * kappa * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        worst = max(worst, -math.copysign(1.0, d0) * y)
    return max(0.0, worst)


def test_the_damping_weight_climbs_with_speed():
    ctl = _Ctl()
    base = ctl.stanley_heading_min_weight
    assert ctl._heading_min_weight_at(1.0) == base
    assert ctl._heading_min_weight_at(4.0) == base
    assert ctl._heading_min_weight_at(6.0) == 1.0
    assert ctl._heading_min_weight_at(9.0) == 1.0
    seq = [ctl._heading_min_weight_at(v) for v in (2.0, 4.0, 5.0, 6.0, 7.0)]
    assert seq == sorted(seq), "속도가 올라가는데 감쇠가 줄면 안 된다"


def test_low_speed_behaviour_is_untouched():
    """20260816 실측으로 맞춘 저속 튜닝을 건드리지 않았는지.

    스케줄을 끈 것과 **수치가 같아야** 한다.
    """
    off = _Ctl(stanley_heading_weight_speed_hi=0.0)
    on = _Ctl()
    for v in (2.0, 3.0, 4.0):
        assert _line_crossing(v, 0.5, -25.0, on) == _line_crossing(v, 0.5, -25.0, off)


def test_a_high_speed_rejoin_no_longer_crosses_the_line():
    """벽 옆 라인이면 넘어가는 순간이 사고다. 실제 도착각 범위에서 0 이어야 한다.

    REJOIN 경로는 합류각을 속도로 묶는다(rejoin_max_heading_deg 18°). 그
    범위에서는 어떤 속도에서도 안 넘어야 한다.
    """
    ctl = _Ctl()
    for deg in (10.0, 15.0, 18.0, 20.0):
        for v in (5.0, 6.0, 7.0):
            assert _line_crossing(v, 0.5, -deg, ctl) < 1e-3, f"{deg}° @ {v}m/s"


def test_regression_the_old_damping_did_cross_the_line():
    """감쇠를 깎던 예전 설정이 실제로 넘어갔는지.

    안 넘어간다면 원인이 다른 데 있다는 뜻이고, 이 변경의 근거가 사라진다.
    """
    old = _Ctl(stanley_heading_weight_speed_hi=0.0)  # 스케줄 없음 = 예전 동작
    assert _line_crossing(7.0, 0.5, -20.0, old) > 0.05
    new = _Ctl()
    assert _line_crossing(7.0, 0.5, -20.0, new) < 1e-3


def test_damping_helps_even_past_the_planned_merge_angle():
    """계획을 벗어난 각으로 도착해도 예전보다는 덜 넘어야 한다.

    추종 오차나 FGM 폴백으로 25° 넘게 들어오는 경우가 있다. 완전히 막지는
    못한다 — 그건 감쇠가 아니라 도착각으로 풀 문제다 — 하지만 나빠지면 안 된다.
    """
    old = _Ctl(stanley_heading_weight_speed_hi=0.0)
    new = _Ctl()
    for deg in (25.0, 30.0):
        assert _line_crossing(7.0, 0.5, -deg, new) < _line_crossing(
            7.0, 0.5, -deg, old
        )
