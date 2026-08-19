#!/usr/bin/env python3
"""AEB 탈출 창 단위 테스트.

노드를 띄우지 않고 상태 머신 메서드만 스텁 self 에 바인딩해 돌린다.

    python3 -m pytest src/path_following/test/test_aeb_escape.py -q
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_following.emergency_brake_node import EmergencyBrakeNode  # noqa: E402

TICK = 0.02  # timer_period_sec


class _Log:
    def __init__(self):
        self.lines: list[str] = []

    def _rec(self, msg):
        self.lines.append(str(msg))

    info = warn = error = _rec


class _Stub:
    """_open_escape_window / _update_escape_window 가 건드리는 것만 갖춘 가짜 노드."""

    def __init__(self, **kw):
        self.escape_enable = kw.get("escape_enable", True)
        self.escape_min_travel = kw.get("escape_min_travel", 0.35)
        self.escape_max_sec = kw.get("escape_max_sec", 3.0)
        self.escape_speed_end = kw.get("escape_speed_end", 1.0)
        self.escape_hard_stop = kw.get("escape_hard_stop", 0.12)
        self._escape_until = 0.0
        self._escape_travel = 0.0
        self._escape_count = 0
        self._speed = 0.0
        self._log = _Log()
        # 시간 초과 분기가 후진 탈출을 부른다. 뒤가 막힌 것으로 두면 이
        # 파일의 관심사(전진 탈출 창)만 남는다 — 후진은 별도 파일에서 본다.
        self.reverse_enable = kw.get("reverse_enable", False)
        self.reverse_min_clearance = 0.60
        self._reverse_until = 0.0

    def get_logger(self):
        return self._log

    def _rear_clearance(self) -> float:
        return 0.0

    open_window = EmergencyBrakeNode._open_escape_window
    update = EmergencyBrakeNode._update_escape_window
    _maybe_start_reverse = EmergencyBrakeNode._maybe_start_reverse


def test_regression_retrigger_loop_without_escape():
    """탈출 창이 없으면 벌어지는 일 — 회귀 방지용으로 남긴다.

    standoff(0.30) 안쪽 0.25 m 에 서 있고 차는 안 움직인다. 창이 꺼져 있으면
    해제 다음 틱에 too_close 가 그대로 참이라 즉시 재발동한다.
    """
    s = _Stub(escape_enable=False)
    s.open_window(0.0)
    assert s._escape_until == 0.0, "꺼져 있으면 창이 열리면 안 된다"
    assert s.update(TICK, TICK, closest=0.25) is False, "억제 없음 → 즉시 재발동"


def test_escape_window_suppresses_retrigger_while_stopped():
    """정지 상태로 서 있어도 창이 유지돼야 탈출을 시작할 수 있다."""
    s = _Stub()
    s.open_window(0.0)
    now = 0.0
    for _ in range(50):  # 1 초
        now += TICK
        assert s.update(now, TICK, closest=0.25) is True


def test_escape_closes_after_moving_far_enough():
    s = _Stub(escape_min_travel=0.35)
    s.open_window(0.0)
    s._speed = 0.5
    now, ticks = 0.0, 0
    for _ in range(150):
        now += TICK
        ticks += 1
        if not s.update(now, TICK, closest=0.25):
            break
    # 0.5 m/s × 0.7 s = 0.35 m → 35 틱
    assert ticks == 35
    assert s._escape_until == 0.0
    assert s._escape_travel == 0.0, "종료 시 누적 이동거리는 초기화된다"
    assert "탈출 성공" in "".join(s._log.lines)


def test_escape_closes_on_timeout():
    s = _Stub(escape_max_sec=1.0)
    s.open_window(0.0)
    now = 0.0
    for _ in range(60):
        now += TICK
        if not s.update(now, TICK, closest=0.25):
            break
    assert s._escape_until == 0.0
    assert "시간 초과" in "".join(s._log.lines)
    assert now >= 1.0


def test_hard_stop_still_fires_inside_window():
    """창 안이라도 진짜 코앞이면 억제하지 않는다. 이게 없으면 벽으로 들어간다."""
    s = _Stub(escape_hard_stop=0.12)
    s.open_window(0.0)
    assert s.update(TICK, TICK, closest=0.25) is True
    assert s.update(2 * TICK, TICK, closest=0.10) is False
    assert s._escape_until == 0.0
    assert "hard_stop" in "".join(s._log.lines)


def test_escape_closes_when_back_up_to_speed():
    s = _Stub(escape_speed_end=1.0, escape_min_travel=99.0)
    s.open_window(0.0)
    s._speed = 1.4
    assert s.update(TICK, TICK, closest=0.5) is False
    assert "정상 주행 복귀" in "".join(s._log.lines)


def test_window_is_not_reopened_by_update():
    """닫힌 창은 update 만으로 다시 열리지 않는다."""
    s = _Stub()
    assert s.update(TICK, TICK, closest=0.25) is False
    assert s._escape_until == 0.0
