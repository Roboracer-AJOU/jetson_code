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
        self.reverse_stuck_sec = kw.get("reverse_stuck_sec", 1.0)
        self.reverse_stuck_obstacle = kw.get("reverse_stuck_obstacle", 0.79)
        self.reverse_cooldown_sec = kw.get("reverse_cooldown_sec", 3.0)
        self.stuck_speed = 0.05
        self._reverse_ready_at = 0.0
        self._idle_since = 0.0
        self.escape_enable = True
        self.escape_min_travel = 0.35
        self.escape_max_sec = 3.0
        self.escape_speed_end = 1.0
        # 실제 설정값. 이 값이 곧 실차에서 문제가 된 지점이라 낮춰 잡으면
        # 재현이 안 된다.
        self.escape_hard_stop = round(vg.LASER_TO_FRONT_M + 0.05, 3)  # 0.24
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
    is_stuck = EmergencyBrakeNode._stuck_against_something


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


# ── 그냥 서 있는 것도 실패다 (탈출 창과 무관) ───────────────────────────


def _sit_still(a: _Aeb, closest: float, secs: float, speed: float = 0.0):
    """정지 상태로 시간을 흘리고, 후진이 걸린 시각을 낸다 (없으면 None)."""
    a._speed = speed
    now = 0.0
    for _ in range(int(secs / TICK)):
        now += TICK
        if a._reverse_until <= 0.0 and a.is_stuck(now, closest):
            a.start(now)
            if a._reverse_until > 0.0:
                return now
    return None


def test_regression_the_hardstop_loop_no_longer_traps_the_car():
    """실차에서 탈출 창 #105 까지 헛돌던 상황.

    `closest`(0.24) 가 `escape_hard_stop`(0.24) 과 같은 값이라 창이 열린 다음
    틱에 hard_stop 침범으로 닫힌다. 시간 초과 분기에 영영 도달하지 못하니
    거기 걸어 둔 후진 판정도 영영 안 걸렸다. 차는 조향만 떨며 서 있었다.

    그래서 창이 아니라 결과를 본다 — 서 있으면 못 나가는 것이다.
    """
    a = _Aeb()
    # 로그의 0.24 는 반올림 표시고 실제 값은 임계 언저리에서 흔들린다.
    # 임계 바로 아래로 한 번만 내려가도 창이 그 틱에 닫힌다.
    at_threshold = a.escape_hard_stop - 0.005

    a.open_window(0.0)
    # 창이 hard_stop 으로 즉시 닫힌다 (stalled 아님 → 예전 경로로는 안 걸림)
    assert a.update_window(TICK, TICK, closest=at_threshold) is False
    assert "hard_stop" in a.log
    assert a._reverse_until == 0.0, "예전 경로로는 안 걸리는 게 맞다"

    assert _sit_still(a, closest=at_threshold, secs=3.0) is not None, "여전히 갇힌다"
    assert "후진 탈출 시작" in a.log


def test_it_waits_the_full_second_before_giving_up():
    a = _Aeb(reverse_stuck_sec=1.0)
    at = _sit_still(a, closest=0.30, secs=3.0)
    assert at == pytest.approx(1.0, abs=2 * TICK)


def test_stopping_with_nothing_ahead_is_just_parked():
    """출발 전 정차나 신호 대기까지 후진하면 안 된다."""
    a = _Aeb()
    assert _sit_still(a, closest=3.0, secs=5.0) is None


def test_crawling_forward_resets_the_clock():
    """조금이라도 나아가고 있으면 갇힌 게 아니다."""
    a = _Aeb(reverse_stuck_sec=1.0)
    now = 0.0
    for i in range(200):
        now += TICK
        # 0.5 초마다 한 틱씩 움직인다
        a._speed = 0.5 if i % 25 == 0 else 0.0
        if a.is_stuck(now, 0.30):
            a.start(now)
    assert a._reverse_until == 0.0, f"기어가는 중인데 후진: {a.log}"


def test_it_does_not_back_up_over_and_over():
    """물러난 직후 또 걸리면 뒤가 빌 때까지 뒷걸음질한다."""
    a = _Aeb(reverse_cooldown_sec=3.0)
    a.start(0.0)
    _back_up(a)                      # 정상 종료 → 쿨다운 시작
    assert a._reverse_ready_at > 0.0
    a._speed = 0.0
    a._idle_since = 0.0
    # 쿨다운 안에서는 아무리 서 있어도 안 걸린다
    now = a._reverse_ready_at - 0.5
    assert a.is_stuck(now, 0.30) is False or (a.start(now) or a._reverse_until == 0.0)


def test_a_blocked_rear_does_not_spam_the_log():
    """앞뒤 다 막힌 상태는 매 틱 참이다. 20 Hz 로 찍으면 로그가 죽는다."""
    a = _Aeb(rear=0.10)
    now = 0.0
    for _ in range(200):  # 4 초
        now += TICK
        a.start(now)
    assert a.log.count("전진도 후진도 막혔다") <= 3


# ── 후방 여유 측정 ──────────────────────────────────────────────────────


STEP = EmergencyBrakeNode._REVERSE_PROBE_STEP_M


class _Map:
    """벽까지 거리를 직접 주는 가짜 맵.

    실차 맵은 distance transform 격자지만, 여기서 보고 싶은 건 그 값을
    어떻게 읽어 뒤로 얼마나 갈 수 있다고 판단하는가다.
    """

    def __init__(self, fn):
        self._fn = fn

    def clearance_at(self, x: float, y: float) -> float:
        return self._fn(x, y)


class _Rear:
    """차가 map 원점에 +x 를 보고 서 있는 상태."""

    def __init__(self, gmap, tf=(1.0, 0.0, 0.0, 0.0), **kw):
        self._map = gmap
        self._tf = tf
        self.reverse_map_margin = kw.get("margin", 0.10)
        self.reverse_travel = kw.get("travel", 0.40)
        self.reverse_min_clearance = kw.get("min_clearance", 0.60)

    _REVERSE_PROBE_STEP_M = STEP

    def _lookup_laser_to_map(self):
        return self._tf

    clearance = EmergencyBrakeNode._rear_clearance


def _wall_at(x_wall: float):
    """x = x_wall 에 놓인 벽 하나. 그 앞쪽은 벽까지 거리가 곧 여유다."""
    return _Map(lambda x, y: abs(x - x_wall))


LIMIT = 1.0  # travel 0.40 + min_clearance 0.60


def test_it_reports_how_far_the_bumper_can_travel():
    """뒤끝이 x=-0.41 이고 벽이 x=-1.41 이면 1.0 m 갈 수 있다.

    다만 중심선 기준이라 차 반폭+여유(0.25) 만큼 일찍 막힌 것으로 본다.
    보수적인 쪽이라 그대로 둔다.
    """
    need = vg.HALF_WIDTH_M + 0.10
    got = _Rear(_wall_at(-(vg.LASER_TO_REAR_M + 1.0))).clearance()
    want = 1.0 - need  # 0.75
    # 0.05 간격으로 훑으므로 한 칸까지는 늦게 잡힐 수 있다
    assert want <= got <= want + STEP + 1e-9


def test_a_wall_right_behind_gives_nothing():
    got = _Rear(_wall_at(-(vg.LASER_TO_REAR_M + 0.05))).clearance()
    assert got == 0.0


def test_it_stops_looking_once_it_has_seen_enough():
    """더 멀리 재 봐야 쓰지도 않는다 — 상한에서 자른다."""
    assert _Rear(_Map(lambda x, y: 50.0)).clearance() == pytest.approx(LIMIT)


def test_the_offset_to_the_rear_bumper_is_applied():
    """라이다가 축보다 0.31 m 앞이라 뒤끝까지가 0.41 m 다.

    이 오프셋을 빼먹으면 있지도 않은 0.41 m 를 믿고 벽으로 민다.
    """
    assert vg.LASER_TO_REAR_M == pytest.approx(0.41)
    seen: list[float] = []
    _Rear(_Map(lambda x, y: seen.append(x) or 50.0)).clearance()
    assert seen[0] == pytest.approx(-vg.LASER_TO_REAR_M)


def test_it_follows_the_car_heading():
    """차가 돌아 서 있으면 '뒤' 도 같이 돈다. 맵 축이 아니다."""
    # +y 를 보고 서 있다 (yaw=90°) → 뒤는 -y 방향
    tf = (0.0, 1.0, 0.0, 0.0)
    seen: list[tuple] = []
    _Rear(_Map(lambda x, y: seen.append((x, y)) or 50.0), tf=tf).clearance()
    x0, y0 = seen[0]
    assert x0 == pytest.approx(0.0)
    assert y0 == pytest.approx(-vg.LASER_TO_REAR_M)


def test_a_narrow_corridor_still_lets_it_back_up():
    """양옆 벽이 있어도 차폭이 들어가면 물러날 수 있어야 한다."""
    # 중심선에서 좌우 0.30 m 에 벽 (통로 폭 0.60) — 차폭 0.30 은 들어간다
    assert _Rear(_Map(lambda x, y: 0.30)).clearance() == pytest.approx(LIMIT)


def test_a_corridor_too_narrow_is_refused():
    # 좌우 0.20 — 반폭 0.15 + 여유 0.10 = 0.25 에 못 미친다
    assert _Rear(_Map(lambda x, y: 0.20)).clearance() == 0.0


# ── 못 재면 물러나지 않는다 (전진 검사와 반대) ──────────────────────────


def test_regression_no_map_means_do_not_reverse():
    """예전엔 못 재면 inf(비었다)를 냈다. 그게 실차에서 이렇게 나왔다:

        후진 탈출 시작 #23 — 뒤 여유 inf m
        후진 탈출 종료 — 뒤가 막혔다 (0.00m)   ← 0.02 s 뒤

    라이다가 뒤를 못 보니 후방 섹터에 빔이 없어 inf 가 나오고, FOV 가장자리
    잡음이 하나 들어오면 0.00 이 나온다. 20 ms 만에 끝나니 차는 찔끔 움직이고
    말았다.

    뒤를 볼 수단이 맵뿐이라, 맵이 없으면 낙관하면 안 된다 — 눈 감고 후진하는
    셈이다.
    """
    assert _Rear(None).clearance() == 0.0


def test_no_tf_means_do_not_reverse():
    r = _Rear(_Map(lambda x, y: 50.0))
    r._tf = None
    assert r.clearance() == 0.0


def test_outside_the_map_is_blocked():
    """맵 밖은 `clearance_at` 이 0 을 준다. 그대로 막힌 것으로 읽어야 한다."""
    assert _Rear(_Map(lambda x, y: 0.0)).clearance() == 0.0


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
