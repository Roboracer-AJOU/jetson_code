#!/usr/bin/env python3
"""라인과 크게 비스듬할 때 재합류가 거짓말을 하지 않는지 검증.

    python3 -m pytest src/path_following/test/test_rejoin_heading_validity.py -q

배경 (실차): 회피 뒤 복귀하다 두 바퀴 연속으로 벽에 박았다. 박은 순간
차의 헤딩이 레이스라인과 거의 **수직** 이었다.

원인은 Frenet 투영에 있었다. `d0p = tan(yaw_err)` 를 ±1.0 으로 자르고 있었고,
그건 곧 "45° 보다 더 틀어진 상태는 존재하지 않는다" 는 뜻이다. 75° 로 벽을
향한 차도 플래너에는 45° 로 들어갔다. quintic 은 그 45° 를 시작 기울기로
얌전한 경로를 냈고, 경로 자체는 충돌검사를 통과했다 — 차가 그걸 따라갈 수
없다는 사실만 아무도 몰랐다.

d(s) 는 s 의 함수라 수직에서는 정의가 무너진다 (ds/dt → 0). 한계를 넘으면
경로를 만들 게 아니라 **정렬** 을 먼저 해야 한다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path as _Path
from types import SimpleNamespace

sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))

from path_following.local_planner_node import LocalPlannerNode  # noqa: E402

LIMIT_DEG = 55.0


class _Clock:
    def __init__(self):
        self.t = 10_000_000_000  # 경고 스로틀(1초)에 걸리지 않게 0 이 아닌 시각에서 시작

    def now(self):
        from builtin_interfaces.msg import Time

        return SimpleNamespace(nanoseconds=self.t, to_msg=lambda: Time())


class _Log:
    def __init__(self):
        self.msgs = []

    def warn(self, m):
        self.msgs.append(m)

    info = debug = warn


class _Straight:
    """y=0 을 따라가는 직선 기준선. 투영은 x 좌표 그대로."""

    _n = 4
    _xs = [0.0, 1.0, 2.0, 3.0]
    _ys = [0.0, 0.0, 0.0, 0.0]
    _seg_start = [0.0, 1.0, 2.0, 3.0]
    _seg_len = [1.0, 1.0, 1.0, 1.0]
    _total_l = 4.0
    map_frame = "map"

    rejoin_min_length_m = 0.50
    rejoin_max_length_m = 10.0
    rejoin_time_sec = 0.8
    rejoin_max_active_ns = 5_000_000_000

    def __init__(self, v: float = 3.0, blocked: bool = False):
        self._rejoin_yaw_err_limit = math.radians(LIMIT_DEG)
        self._alignment_release_rad = 0.6 * self._rejoin_yaw_err_limit
        self._ego_speed_mps = v
        self._blocked = blocked
        self._last_path_cut = 1
        self._last_align_warn_ns = 0
        self._last_rejoin_warn_ns = 0
        self._rejoin_is_alignment = False
        self._rejoin_path_msg = None
        self._clock = _Clock()
        self._log = _Log()

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._log

    def _closest_on_loop(self, xp, yp):
        return xp, 0.0, 0, 0.0

    def _xy_yaw_at_s(self, s):
        return float(s), 0.0, 0.0

    def _lookup_laser_to_map_transform(self):
        return None

    def _truncate_path_at_collision(self, path, _tf):
        return (path, False) if self._blocked else (path, True)

    _project_to_frenet = LocalPlannerNode._project_to_frenet
    _append_pose = LocalPlannerNode._append_pose
    _build_alignment_path = LocalPlannerNode._build_alignment_path
    _warn_rejoin_aligning = LocalPlannerNode._warn_rejoin_aligning
    _warn_rejoin_given_up = LocalPlannerNode._warn_rejoin_given_up
    _alignment_done = LocalPlannerNode._alignment_done


def _pose(x, y, yaw_deg):
    half = math.radians(yaw_deg) * 0.5
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        )
    )


def test_a_steep_heading_is_reported_as_it_is():
    """예전엔 75° 가 45° 로 들어갔다. 그 20° 는 그대로 추종 오차가 된다."""
    n = _Straight()
    _, d0, d0p, _, _, yaw_err = n._project_to_frenet(0.5, 0.8, math.radians(75.0))

    assert abs(d0 - 0.8) < 1e-9
    assert abs(math.degrees(yaw_err) - 75.0) < 1e-6, "실제 헤딩오차가 안 나온다"
    # 기울기는 여전히 잘리지만(tan 발산 방지) 45° 가 아니라 유효 한계까지다.
    assert math.degrees(math.atan(d0p)) > 45.0 + 1e-6
    assert abs(math.degrees(math.atan(d0p)) - LIMIT_DEG) < 1e-6


def test_within_the_limit_nothing_is_clamped():
    n = _Straight()
    for deg in (0.0, 10.0, 30.0, 44.0, 50.0):
        _, _, d0p, _, _, _ = n._project_to_frenet(0.5, 0.3, math.radians(deg))
        assert abs(d0p - math.tan(math.radians(deg))) < 1e-9, f"{deg}° 에서 잘렸다"


def test_the_sign_survives_the_clamp():
    n = _Straight()
    _, _, d0p, _, _, yaw_err = n._project_to_frenet(0.5, 0.8, math.radians(-75.0))
    assert d0p < 0.0 and yaw_err < 0.0


def test_a_near_perpendicular_car_gets_an_alignment_path_not_a_rejoin():
    """수직에 가까우면 옆으로 붙으라는 요구를 뺀다. 방향부터 맞춘다."""
    n = _Straight(v=3.0)
    out = n._build_alignment_path(0.0, 0.8, math.radians(80.0))

    assert out is not None and len(out.poses) >= 2
    assert n._rejoin_is_alignment is True
    # 이탈이 줄지 않는 게 핵심이다 — 줄이려 들면 그게 벽으로 가는 요구가 된다.
    for ps in out.poses:
        assert abs(ps.pose.position.y - 0.8) < 1e-9
    # 진행방향으로는 실제로 뻗어 있어야 Stanley 가 볼 게 있다.
    assert out.poses[-1].pose.position.x > out.poses[0].pose.position.x + 1.0
    assert any("정렬" in m for m in n._log.msgs)


def test_a_blocked_alignment_path_is_refused_and_says_why():
    n = _Straight(blocked=True)
    assert n._build_alignment_path(0.0, 0.8, math.radians(80.0)) is None
    assert n._rejoin_is_alignment is False
    assert any("REJOIN 포기" in m for m in n._log.msgs)


def test_the_alignment_path_is_dropped_once_the_car_points_the_right_way():
    """정렬 경로는 이탈을 안 줄인다 — 방향이 맞으면 놓아 줘야 복귀가 시작된다."""
    n = _Straight()
    n._rejoin_path_msg = n._build_alignment_path(0.0, 0.8, math.radians(80.0))
    assert n._rejoin_is_alignment is True

    assert n._alignment_done(_pose(0.5, 0.8, 70.0)) is False, "아직 한계 위인데 놓았다"
    assert n._rejoin_path_msg is not None
    # 진입각(55°) 바로 밑에서 놓으면 quintic 이 tan(54°) 를 받아 또 포기한다.
    assert n._alignment_done(_pose(0.5, 0.8, 50.0)) is False, "이력 없이 바로 놓았다"
    assert n._rejoin_path_msg is not None

    assert n._alignment_done(_pose(0.5, 0.8, 20.0)) is True
    assert n._rejoin_path_msg is None, "정렬이 끝났는데 경로를 붙들고 있다"
    assert n._rejoin_is_alignment is False


def test_a_normal_rejoin_path_is_never_dropped_by_the_alignment_check():
    """'한 번 그리면 끝까지' 규칙은 정상 복귀에서 그대로여야 한다."""
    n = _Straight()
    n._rejoin_is_alignment = False
    n._rejoin_path_msg = object()
    assert n._alignment_done(_pose(0.5, 0.8, 5.0)) is False
    assert n._rejoin_path_msg is not None
