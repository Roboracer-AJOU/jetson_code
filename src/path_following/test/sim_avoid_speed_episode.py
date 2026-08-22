"""회피 한 번을 처음부터 끝까지 돌려 보고 속도가 어떻게 가는지 찍는다.

읽어서 "맞게 짜여 있다" 고 말하는 대신, 실제 메서드를 그대로 물려서
접근 → 인지 → 회피 → 복귀 → 글로벌 을 20 Hz 로 돌린다.

    python3 src/path_following/test/sim_avoid_speed_episode.py
"""

from __future__ import annotations

import sys
from pathlib import Path as FsPath

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1]))

from path_following.local_planner_node import CFG, LocalPlannerNode  # noqa: E402

HZ = 20.0
DT = 1.0 / HZ


class _Clock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self.ns


class _Sim:
    """`_planner_speed_scale` 의 속도 정책부만 실제 코드로 돌리는 최소 노드."""

    # ---- 검사 대상: 전부 실제 구현이다 ----
    _speed_scaled_dist = LocalPlannerNode._speed_scaled_dist
    _avoid_on_late_factor = LocalPlannerNode._avoid_on_late_factor
    _effective_avoid_gates = LocalPlannerNode._effective_avoid_gates
    _avoid_speed_capped = LocalPlannerNode._avoid_speed_capped
    _avoid_cruise_target = LocalPlannerNode._avoid_cruise_target
    _slew_limit_speed = LocalPlannerNode._slew_limit_speed

    def __init__(self):
        for k in (
            "avoid_on_m",
            "avoid_on_min_m",
            "avoid_on_max_m",
            "avoid_off_m",
            "avoid_off_min_m",
            "avoid_off_max_m",
            "fgm_enable_m",
            "fgm_enable_min_m",
            "fgm_enable_max_m",
            "avoid_timing_margin",
            "avoid_timing_ref_mps",
            "avoid_on_late_scale",
            "avoid_on_late_max_speed",
            "avoid_on_late_blend_mps",
            "avoid_cruise_speed_high_mps",
            "avoid_cruise_speed_low_mps",
            "avoid_cruise_high_speed_th",
            "avoid_a_accel_mps2",
        ):
            setattr(self, k, CFG[k])
        self.avoid_speed_params = type(
            "P", (), {"a_brake": CFG["avoid_a_brake_mps2"]}
        )()
        self._clock = _Clock()
        self._slew_prev_v = None
        self._slew_prev_ns = 0
        self._avoid_cruise_latched = None
        self._avoid_cruise_prev = None
        self._avoid_cruise_release_ns = 0
        self.avoid_cruise_regrab_ns = int(CFG["avoid_cruise_regrab_sec"] * 1e9)
        self._ego_speed_mps = 0.0
        self.mode = "GLOBAL"
        self._obstacle_on = False

    def get_clock(self):
        return self._clock

    def cruise_scale(self, v_csv: float) -> tuple[float, float]:
        """`_planner_speed_scale` 의 순항속도 적용 + slew + 배율 환산."""
        v_target = v_csv  # 다른 한계는 안 걸리는 상황을 가정
        cruise = self._avoid_cruise_target()
        if cruise > 0.0:
            v_target = cruise
        v_target = self._slew_limit_speed(v_target, ceiling=v_csv)
        return v_target, min(1.0, v_target / max(0.05, v_csv))


def run(v0: float, v_csv: float, obstacle_at_m: float, label: str):
    s = _Sim()
    s._ego_speed_mps = v0
    v = v0
    x = 0.0  # 주행거리
    log = []
    marks = {}

    for step in range(400):
        s._clock.ns += int(DT * 1e9)
        d = obstacle_at_m - x  # 장애물까지 남은 거리

        on_m, _off, _fgm = s._effective_avoid_gates()
        # 지나친 장애물은 전방 콘·min_forward_x 에서 빠지므로 더는 안 보인다.
        # 여기서 남은 거리로 계속 재면 통과 후에도 게이트가 켜진 채가 된다.
        s._obstacle_on = 0.0 <= d <= on_m

        # 모드 전이 (실제 노드의 게이트 순서를 흉내낸다)
        if s.mode == "GLOBAL" and s._obstacle_on:
            s.mode = "AVOID"
            marks["인지·로컬패스 전환"] = (x, d, v, on_m)
        elif s.mode == "AVOID" and d < -1.0:
            s.mode = "REJOIN"
            marks["장애물 통과, 복귀 시작"] = (x, d, v, 0.0)
        elif s.mode == "REJOIN" and d < -4.0:
            s.mode = "GLOBAL"
            marks["글로벌패스 복귀"] = (x, d, v, 0.0)

        v_cmd, scale = s.cruise_scale(v_csv)
        v = v_cmd  # 명령을 그대로 따라간다고 본다 (slew 가 이미 물리 한계)
        s._ego_speed_mps = v
        x += v * DT
        log.append((x, d, s.mode, s._avoid_cruise_latched, v, scale))
        if s.mode == "GLOBAL" and d < -6.0:
            break

    print(f"=== {label} ===")
    print(f"    진입 {v0} m/s, 이 구간 CSV 속도 {v_csv} m/s, 장애물 {obstacle_at_m} m 앞\n")
    for name, (px, pd, pv, pon) in marks.items():
        extra = f", 게이트 {pon:.1f} m" if pon else ""
        print(f"    [{name}] 주행 {px:5.1f} m, 장애물까지 {pd:5.2f} m, 속도 {pv:.2f} m/s{extra}")

    tgt = s.avoid_cruise_speed_high_mps if v0 > s.avoid_cruise_high_speed_th else s.avoid_cruise_speed_low_mps
    at_obs = [e for e in log if abs(e[1]) < 0.2]
    print(f"\n    목표 순항속도 {tgt} m/s")
    if at_obs:
        print(f"    장애물 도달 시점 속도  {at_obs[0][4]:.2f} m/s  "
              f"{'OK 이미 목표속도' if at_obs[0][4] <= tgt + 0.05 else 'X 아직 안 줄었다'}")
    avoid_v = [e[4] for e in log if e[2] in ("AVOID", "REJOIN")]
    if avoid_v:
        print(f"    회피~복귀 구간 속도    {min(avoid_v):.2f} ~ {max(avoid_v):.2f} m/s")
    tail = [e[4] for e in log if e[2] == "GLOBAL" and e[1] < -4.5]
    if tail:
        print(f"    글로벌 복귀 후 속도    {tail[-1]:.2f} m/s  (CSV {v_csv})")
    print()
    print("      주행거리  장애물까지   모드    래치   속도   배율")
    prev_mode = None
    for i, (px, pd, mode, latch, vv, sc) in enumerate(log):
        show = mode != prev_mode or i % 10 == 0
        prev_mode = mode
        if show and pd > -7.0:
            lt = f"{latch:.1f}" if latch else " - "
            print(f"      {px:7.2f}  {pd:8.2f}   {mode:<7} {lt}  {vv:5.2f}  {sc:5.3f}")
    print()


def flicker():
    """인지가 한 프레임 끊기면 어떻게 되나.

    래치는 `_avoid_speed_capped()` 가 False 인 순간 풀린다. 접근 중에
    검출이 한 프레임 깜빡이고 그때 모드가 아직 GLOBAL 이면, 다음 프레임에
    **그때 속도로** 다시 고른다 — 이미 감속했으니 저속으로 떨어진다.
    """
    s = _Sim()
    s._ego_speed_mps = 6.0
    s.mode, s._obstacle_on = "GLOBAL", True
    first = s._avoid_cruise_target()

    s._ego_speed_mps = 3.5  # 목표를 향해 감속하는 중
    held = s._avoid_cruise_target()

    s._clock.ns += int(DT * 1e9)
    s._obstacle_on = False  # 검출 한 프레임 유실 (모드는 아직 GLOBAL)
    s._avoid_cruise_target()
    s._clock.ns += int(DT * 1e9)
    s._obstacle_on = True
    after = s._avoid_cruise_target()

    # 진짜로 끝난 경우는 되쓰면 안 된다 — 다음 장애물은 새로 정한다.
    s.mode, s._obstacle_on = "GLOBAL", False
    s._avoid_cruise_target()
    s._clock.ns += int((CFG["avoid_cruise_regrab_sec"] + 0.5) * 1e9)
    s._ego_speed_mps, s._obstacle_on = 2.5, True
    later = s._avoid_cruise_target()

    print("=== 접근 중 검출 한 프레임 유실 ===\n")
    print(f"    6.0 m/s 로 인지 → 래치 {first} m/s")
    print(f"    3.5 m/s 로 감속 (유실 없음) → {held} m/s  {'OK 유지' if held == first else 'X'}")
    print(f"    한 프레임 유실 후 재인지 → {after} m/s  "
          f"{'OK 유지' if after == first else 'X 저속으로 재결정됨'}")
    print(f"    한참 뒤 다음 장애물(2.5 m/s) → {later} m/s  "
          f"{'OK 새로 결정' if later == CFG['avoid_cruise_speed_low_mps'] else 'X 묵은 값'}")
    print()


if __name__ == "__main__":
    run(6.0, 6.0, 30.0, "고속 진입 (직선, CSV 6 m/s)")
    run(3.5, 3.5, 20.0, "저속 진입 (코너, CSV 3.5 m/s)")
    run(8.0, 8.0, 30.0, "최고속 진입 (CSV 8 m/s)")
    flicker()
