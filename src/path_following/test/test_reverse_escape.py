#!/usr/bin/env python3
"""장애물 **코앞** 에 섰을 때 뒤로 빠지는지 검증.

    python3 -m pytest src/path_following/test/test_reverse_escape.py -q

FGM 은 조향으로만 빠져나간다. 그런데 막힌 반각은

    asin((장애물반경 + 버블 + 차반폭) / 거리)

라 거리가 그 반경(약 0.55 m) 안이면 90° 다 — 최대 조향을 줘도 열린 방향이
없다. 실차에서 이때 전진 탈출 창 3 초를 제자리에서 다 쓰고, 창이 닫히면
standoff 가 다시 물어 영영 그 자리였다.

0.4 m 만 물러나면 같은 식이 50° 로 떨어져 전진 탈출이 그때부터 가능해진다.
그래서 "아무 진전 없이 시간 초과" 를 최대 조향 실패로 보고 곧게 물러난다.

후진은 제동과 달리 차를 **움직이는** 명령이라 실패 규칙이 반대다. 신호가
끊기면 걸지 않고 푼다. 그 비대칭이 여기 절반을 차지한다.
"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following import vehicle_geometry as vg  # noqa: E402
from path_following.control_node import VehicleControlNode  # noqa: E402
from path_following.emergency_brake_node import EmergencyBrakeNode  # noqa: E402

TICK = 0.02


class _Log:
    def __init__(self):
        self.lines: list[str] = []

    def _rec(self, msg):
        self.lines.append(str(msg))

    info = warn = error = _rec


# ── 판단: 언제 물러나는가 ───────────────────────────────────────────────


class _Aeb:
    """후진 상태 기계만 갖춘 가짜 AEB 노드."""

    def __init__(self, rear: float = 1.5, **kw):
        self.reverse_enable = kw.get("reverse_enable", True)
        self.reverse_travel = kw.get("reverse_travel", 0.40)
        self.reverse_max_sec = kw.get("reverse_max_sec", 2.0)
        self.reverse_min_clearance = kw.get("reverse_min_clearance", 0.60)
        self.reverse_abort_clearance = kw.get("reverse_abort_clearance", 0.25)
        self.escape_enable = True
        self.escape_min_travel = 0.35
        self.escape_max_sec = 3.0
        self.escape_speed_end = 1.0
        self.escape_hard_stop = 0.12
        self._escape_until = 0.0
        self._escape_travel = 0.0
        self._escape_count = 0
        self._reverse_until = 0.0
        self._reverse_travel = 0.0
        self._reverse_count = 0
        self._speed = 0.0
        self.rear = rear
        self._log = _Log()

    def get_logger(self):
        return self._log

    def _rear_clearance(self) -> float:
        return self.rear

    @property
    def log(self) -> str:
        return "".join(self._log.lines)

    # 실제 코드가 서로를 부르므로 원래 이름으로도 달아 둔다
    _maybe_start_reverse = EmergencyBrakeNode._maybe_start_reverse
    _open_escape_window = EmergencyBrakeNode._open_escape_window
    start = EmergencyBrakeNode._maybe_start_reverse
    update = EmergencyBrakeNode._update_reverse
    open_window = EmergencyBrakeNode._open_escape_window
    update_window = EmergencyBrakeNode._update_escape_window


def test_a_stalled_forward_escape_triggers_the_reverse():
    """제자리에서 3 초를 다 쓰면 최대 조향으로도 못 나간 것이다."""
    a = _Aeb()
    a.open_window(0.0)
    now = 0.0
    for _ in range(200):
        now += TICK
        if not a.update_window(now, TICK, closest=0.25):
            break
    assert a._reverse_until > 0.0, "시간 초과했는데 후진을 안 건다"
    assert "후진 탈출 시작" in a.log


def test_creeping_forward_is_not_a_reason_to_back_up():
    """조금이라도 나아가고 있으면 느릴 뿐이다. 뒤로 갈 일이 아니다."""
    a = _Aeb()
    a.open_window(0.0)
    a._speed = 0.1  # 3 초 동안 0.3 m — min_travel 0.35 에는 못 미친다
    now = 0.0
    for _ in range(200):
        now += TICK
        if not a.update_window(now, TICK, closest=0.25):
            break
    assert a._reverse_until == 0.0, f"기어가는 중인데 후진: {a.log}"


def test_success_and_hard_stop_never_reach_the_reverse():
    for kw, closest in ((dict(), 0.25), (dict(), 0.05)):
        a = _Aeb(**kw)
        a.open_window(0.0)
        a._speed = 0.5
        now = 0.0
        for _ in range(200):
            now += TICK
            if not a.update_window(now, TICK, closest=closest):
                break
        assert a._reverse_until == 0.0


def test_a_blocked_rear_means_we_just_sit_there():
    """앞뒤 다 막혔으면 미는 것보다 서 있는 게 낫다."""
    a = _Aeb(rear=0.30)  # min_clearance 0.60 미만
    a.start(0.0)
    assert a._reverse_until == 0.0
    assert "전진도 후진도 막혔다" in a.log


def test_it_can_be_turned_off():
    a = _Aeb(reverse_enable=False)
    a.start(0.0)
    assert a._reverse_until == 0.0


def test_starting_twice_does_not_extend_the_window():
    a = _Aeb()
    a.start(0.0)
    first = a._reverse_until
    a.start(1.0)
    assert a._reverse_until == first
    assert a._reverse_count == 1


# ── 진행: 언제 멈추는가 ─────────────────────────────────────────────────


def _back_up(a: _Aeb, speed: float = 0.4, ticks: int = 400, rear_of=None):
    """후진을 끝까지 돌리고 (경과시간, 틱수) 를 낸다."""
    a._speed = speed
    now, n = 0.0, 0
    for _ in range(ticks):
        now += TICK
        n += 1
        if rear_of is not None:
            a.rear = rear_of(a._reverse_travel)
        if not a.update(now, TICK):
            break
    return now, n


def test_it_stops_after_backing_up_far_enough():
    a = _Aeb(reverse_travel=0.40)
    a.start(0.0)
    _, n = _back_up(a, speed=0.4)
    assert n == 50, "0.4 m/s × 1.0 s = 0.40 m"
    assert a._reverse_until == 0.0
    assert a._reverse_travel == 0.0, "종료 시 누적 거리는 초기화된다"
    assert "물러남" in a.log


def test_backing_up_hands_control_back_with_a_fresh_forward_window():
    """물러난 만큼 앞이 열렸다. 그 자리에서 다시 물리면 무의미하다."""
    a = _Aeb()
    a.start(0.0)
    _back_up(a)
    assert a._escape_until > 0.0, "전진 탈출 창이 다시 안 열렸다"
    assert a._escape_count == 1


def test_it_stops_on_time_even_if_the_wheels_are_spinning():
    """속도계가 0 이라 거리가 안 쌓여도 시간이 끊는다."""
    a = _Aeb(reverse_max_sec=2.0)
    a.start(0.0)
    now, _ = _back_up(a, speed=0.0)
    assert now == pytest.approx(2.0, abs=TICK)
    assert a._reverse_until == 0.0
    assert "시간 초과" in a.log


def test_a_wall_appearing_behind_aborts_immediately():
    """뒤가 좁아지면 목표 거리를 못 채웠어도 즉시 선다."""
    a = _Aeb(reverse_abort_clearance=0.25)
    a.start(0.0)
    # 0.2 m 물러난 시점에 뒤가 0.2 m 로 좁아진다
    _, n = _back_up(a, speed=0.4, rear_of=lambda d: 1.5 if d < 0.2 else 0.2)
    assert n < 50, "목표 0.40 m 를 다 채워 버렸다"
    assert "뒤가 막혔다" in a.log
    assert a._escape_until == 0.0, "중단인데 전진 창을 열면 안 된다"


def test_an_idle_machine_reports_not_reversing():
    a = _Aeb()
    assert a.update(0.0, TICK) is False


# ── 후방 여유 측정 ──────────────────────────────────────────────────────


class _Scan:
    """라이다 원점 기준 360° 스캔."""

    def __init__(self, ranges, angle_min=-np.pi, inc=None):
        self.ranges = list(ranges)
        self.angle_min = angle_min
        self.angle_increment = (
            inc if inc is not None else 2.0 * np.pi / len(self.ranges)
        )


class _Rear:
    def __init__(self, scan):
        self._scan = scan
        self._beam_angles = None
        self.min_range = 0.05
        self.max_range = 6.0
        self.reverse_half_width = 0.25

    _beam_angle_array = EmergencyBrakeNode._beam_angle_array
    clearance = EmergencyBrakeNode._rear_clearance


def _uniform(n: int, r: float) -> _Scan:
    return _Scan([r] * n)


def _wall_behind(dist: float, n: int = 720) -> _Scan:
    """x = -dist 에 놓인 평면 벽. 뒤를 향하지 않는 빔은 멀리 둔다."""
    ranges = []
    for i in range(n):
        angle = -np.pi + i * (2.0 * np.pi / n)
        c = np.cos(angle)
        ranges.append(dist / -c if c < -0.2 else 6.0)
    return _Scan(ranges, inc=2.0 * np.pi / n)


def test_the_rear_clearance_is_measured_from_the_bumper():
    """라이다가 축보다 0.31 m 앞이라 뒤끝까지가 0.41 m 다.

    스캔 거리를 그대로 쓰면 있지도 않은 여유 0.41 m 를 믿고 벽에 박는다.
    """
    got = _Rear(_wall_behind(1.0)).clearance()
    assert got == pytest.approx(1.0 - vg.LASER_TO_REAR_M, abs=0.01)
    assert vg.LASER_TO_REAR_M == pytest.approx(0.41)


def test_things_beside_the_car_do_not_count_as_behind_it():
    """옆 벽까지 세면 좁은 복도에서 후진이 영영 안 열린다."""
    n = 360
    ranges = [6.0] * n
    for i in range(n):
        angle = -np.pi + i * (2.0 * np.pi / n)
        # 정확히 옆(±90°) 0.3 m — 후방 코리도 밖이다
        if abs(abs(angle) - np.pi / 2) < np.radians(3.0):
            ranges[i] = 0.3
    assert _Rear(_Scan(ranges)).clearance() > 1.0


def test_the_front_is_ignored():
    """앞이 코앞이라서 후진하는 것이다. 앞을 세면 시작조차 못 한다."""
    n = 360
    ranges = [3.0] * n
    for i in range(n):
        angle = -np.pi + i * (2.0 * np.pi / n)
        if abs(angle) < np.radians(20.0):
            ranges[i] = 0.2
    assert _Rear(_Scan(ranges)).clearance() == pytest.approx(
        3.0 - vg.LASER_TO_REAR_M, abs=0.05
    )


def test_a_wall_right_behind_reads_zero_not_negative():
    """음수 여유를 내면 비교가 뒤집혀 '넉넉하다' 로 읽힌다."""
    assert _Rear(_uniform(360, 0.2)).clearance() == 0.0


def test_junk_beams_are_dropped():
    n = 360
    ranges = [float("inf")] * n
    ranges[0] = float("nan")  # -180° = 정후방
    ranges[1] = 0.01          # min_range 미만
    assert _Rear(_Scan(ranges)).clearance() == float("inf")


def test_no_scan_means_no_measurement():
    """못 재면 inf — 대신 시작 조건이 아니라 중단 조건에서 걸러야 한다."""
    assert _Rear(None).clearance() == float("inf")


# ── 집행: control_node ──────────────────────────────────────────────────


class _Ctl:
    def __init__(self, **kw):
        self._escape_reverse_duty = kw.get("duty", 0.12)
        self._escape_reverse_stale = kw.get("stale", 0.3)
        self._escape_reverse_max_sec = kw.get("max_sec", 2.5)
        self._escape_reverse_max_speed = kw.get("max_speed", 0.6)
        self._escape_reverse_cmd = False
        self._escape_reverse_recv_time = 0.0
        self._escape_reverse_since = 0.0
        self._escape_reverse_spent = False
        self._auto_duty_output_sign = kw.get("sign", 1.0)
        self._measured_speed_mps = 0.0
        self._log = _Log()

    def get_logger(self):
        return self._log

    cb = VehicleControlNode._escape_reverse_callback
    requested = VehicleControlNode._escape_reverse_requested
    duty = VehicleControlNode._escape_reverse_output_duty


def _send(c: _Ctl, value: bool, at: float, monkeypatch) -> None:
    monkeypatch.setattr("time.time", lambda: at)
    c.cb(SimpleNamespace(data=value))


def test_no_request_means_no_reverse():
    assert _Ctl().requested(1.0) is False


def test_a_fresh_request_reverses(monkeypatch):
    c = _Ctl()
    _send(c, True, 10.0, monkeypatch)
    assert c.requested(10.0) is True


def test_a_stale_request_is_dropped(monkeypatch):
    """제동은 끊기면 걸지만(fail-safe) 후진은 끊기면 푼다.

    이 비대칭이 핵심이다. 신호가 끊겼는데 계속 뒤로 가면 그게 사고다.
    """
    c = _Ctl(stale=0.3)
    _send(c, True, 10.0, monkeypatch)
    assert c.requested(10.2) is True
    assert c.requested(10.4) is False


def test_a_request_stuck_on_gets_cut_off(monkeypatch):
    """요청 노드가 이상해져도 끝없이 뒤로 가지는 않는다."""
    c = _Ctl(max_sec=2.5)
    now = 10.0
    for _ in range(300):  # 6 초
        _send(c, True, now, monkeypatch)
        if not c.requested(now):
            break
        now += TICK
    assert now - 10.0 == pytest.approx(2.5, abs=0.05)
    assert "시간 상한" in "".join(c._log.lines)


def test_the_budget_only_comes_back_after_the_request_drops(monkeypatch):
    c = _Ctl(max_sec=1.0)
    now = 10.0
    for _ in range(200):
        _send(c, True, now, monkeypatch)
        if not c.requested(now):
            break
        now += TICK
    assert c.requested(now) is False, "상한을 넘겼는데 계속 나간다"

    _send(c, False, now, monkeypatch)
    assert c.requested(now) is False
    _send(c, True, now, monkeypatch)
    assert c.requested(now) is True, "요청이 내려갔다 올라오면 다시 걸려야 한다"


def test_the_duty_pushes_the_car_backwards():
    """제동 역토크와 같은 부호 규칙을 쓴다."""
    for sign in (1.0, -1.0):
        c = _Ctl(duty=0.12, sign=sign)
        assert c.duty() == pytest.approx(-0.12 * sign)


def test_the_duty_cuts_out_if_it_rolls_too_fast():
    """뒤는 보면서 가는 게 아니다. 기어가는 속도를 넘기면 안 된다."""
    c = _Ctl(max_speed=0.6)
    c._measured_speed_mps = -0.7
    assert c.duty() == 0.0
    c._measured_speed_mps = -0.3
    assert c.duty() != 0.0


# ── 의도치 않은 후진 (제동 역토크 폭주) ─────────────────────────────────


class _Brake:
    def __init__(self, release: float = 0.15, duty: float = 0.15, sign: float = 1.0):
        self._emergency_brake_release_speed = release
        self._emergency_brake_duty = duty
        self._auto_duty_output_sign = sign
        self._measured_speed_mps = 0.0

    out = VehicleControlNode._emergency_brake_output_duty


def test_the_brake_only_pushes_against_forward_motion():
    b = _Brake()
    b._measured_speed_mps = 2.0
    assert b.out() == pytest.approx(-0.15)


def test_the_brake_lets_go_once_stopped():
    b = _Brake()
    for v in (0.15, 0.05, 0.0):
        b._measured_speed_mps = v
        assert b.out() == 0.0, f"v={v} 에서 아직 민다"


def test_regression_the_brake_does_not_drive_the_car_backwards():
    """실차에서 가끔 뒤로 가던 것 — 해제 판정에 abs() 를 쓴 탓이었다.

    ERPM 은 부호가 있어서 뒤로 구르면 속도가 음수다. abs() 로 재면 |−0.5|
    가 임계 0.15 를 넘어 역토크가 되살아나고, 그게 차를 더 뒤로 민다.
    뒤로 갈수록 더 세게 미는 폭주다. 20 Hz 에서 감속이 해제 구간(±0.15)을
    한 틱에 건너뛰면 바로 이 상태가 된다.
    """
    b = _Brake(release=0.15)
    for v in (-0.2, -0.5, -1.5):
        b._measured_speed_mps = v
        assert b.out() == 0.0, f"뒤로 {abs(v):.1f} m/s 인데 더 민다"
