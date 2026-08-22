#!/usr/bin/env python3
"""돌아가는 스택 옆에 같은 노드를 하나 더 띄워 재거나 프로파일한다.

    python3 debug/profile_node.py local_planner 20            # cProfile
    python3 debug/profile_node.py local_planner 20 --cpu      # 메인스레드 CPU%
    python3 debug/profile_node.py local_planner 20 --cpu --no-gate

py-spy 는 ptrace 권한이 필요한데(ptrace_scope=1) sudo 를 쓸 수 없다. 대신
**출력 토픽을 전부 /prof 아래로 돌린** 사본을 띄운다. 입력(스캔·TF·장애물)은
실제 스택과 같은 걸 받으므로 계산량이 같고, 발행은 아무도 안 듣는 곳으로
가므로 주행에는 닿지 않는다.

`--no-gate` 는 시각화 게이트를 강제로 열어 둔다. 같은 조건에서 게이트만
켜고 꺼서 A/B 를 잡을 때 쓴다.

주행 중에 돌려도 되지만 코어 하나를 더 쓴다.
"""
from __future__ import annotations

import cProfile
import importlib
import os
import pstats
import sys
import threading
import time

CLK = os.sysconf("SC_CLK_TCK")

# 출력 토픽 → /prof/... 로 돌린다. 실제 스택이 듣는 이름이 하나도 남으면 안 된다.
REMAP = {
    "local_planner": [
        "/local_path",
        "/planner/fgm_enable",
        "/planner/fgm_prefer_angle",
        "/planner/local_path_planned",
        "/planner/mode",
        "/planner/speed_condition",
        "/planner/speed_reason",
        "/planner/speed_scale",
        "/planner_path_override_active",
        "/raceline_csv_path",
    ],
    "stanley": [
        "/drive",
        "/waypoint_tracked_path",
        "/stanley/debug",
        "/control/raw_steer_cmd",
        "/control/filtered_steer_cmd",
        "/control/cross_track_error",
        "/control/heading_error",
        "/control/path_curvature",
    ],
    "emergency_brake": [
        "/emergency_brake",
        "/emergency_brake/ttc",
        "/aeb/escape_reverse",
    ],
    "fgm": [
        "/fgm_target",
        "/fgm_gap_marker",
        "/fgm_gap_markers",
        "/fgm_debug_scan",
    ],
    "integrated": [
        "/static_obstacles",
        "/dynamic_obstacles",
        "/visualization_marker_array",
    ],
}

ENTRY = {
    "local_planner": ("path_following.local_planner_node", "local_planner"),
    "stanley": ("path_following.stanley_waypoint_follow_node", "stanley"),
    "emergency_brake": ("path_following.emergency_brake_node", "aeb"),
    "fgm": ("path_following.fgm_node", "fgm"),
    "integrated": ("path_following.integrated_obstacle_node", "integrated"),
}


def main_thread_jiffies() -> int:
    pid = os.getpid()
    with open(f"/proc/{pid}/task/{pid}/stat") as f:
        body = f.read()
    parts = body[body.rindex(")") + 2 :].split()
    return int(parts[11]) + int(parts[12])


def force_gate_open() -> None:
    """시각화 게이트를 항상 열어 둔다 (최적화 전 거동 재현)."""
    import path_following.viz_gate as vg

    vg.has_listener = lambda pub: pub is not None
    # 노드 모듈들이 `from ... import has_listener` 로 이름을 이미 당겨 갔을
    # 수 있으니, 임포트된 모듈에서도 갈아 끼운다.
    for name, mod in list(sys.modules.items()):
        if name.startswith("path_following") and hasattr(mod, "has_listener"):
            mod.has_listener = vg.has_listener


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if not args or args[0] not in ENTRY:
        sys.exit(f"사용법: profile_node.py <{'|'.join(ENTRY)}> [초] [--cpu] [--no-gate]")

    key = args[0]
    dur = float(args[1]) if len(args) > 1 else 20.0
    cpu_only = "--cpu" in flags
    no_gate = "--no-gate" in flags

    mod_name, tag = ENTRY[key]
    suffix = "_nogate" if no_gate else "_prof"
    ros_args = ["prog", "--ros-args", "-r", f"__node:={tag}{suffix}"]
    for t in REMAP[key]:
        ros_args += ["-r", f"{t}:=/prof{t}"]
    sys.argv = ros_args

    import rclpy

    mod = importlib.import_module(mod_name)
    if no_gate:
        force_gate_open()

    def stop_later():
        time.sleep(dur + 2.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    threading.Thread(target=stop_later, daemon=True).start()

    result = {}

    def sample_cpu():
        # 노드가 뜨고 토픽이 붙을 시간을 준 뒤부터 잰다
        time.sleep(2.0)
        j0, t0 = main_thread_jiffies(), time.time()
        time.sleep(dur)
        j1, t1 = main_thread_jiffies(), time.time()
        result["pct"] = 100.0 * (j1 - j0) / CLK / (t1 - t0)

    prof = None
    if cpu_only:
        threading.Thread(target=sample_cpu, daemon=True).start()
    else:
        prof = cProfile.Profile()
        prof.enable()

    try:
        mod.main()
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:  # 종료 경합은 측정 결과와 무관하다
        print(f"(종료 시 예외: {type(e).__name__}: {e})", file=sys.stderr)
    finally:
        if prof is not None:
            prof.disable()

    label = f"{key}{' (게이트 강제 개방)' if no_gate else ' (게이트 동작)'}"
    if cpu_only:
        pct = result.get("pct")
        print(f"\n{label:38s} 메인스레드 CPU {pct:5.1f}%" if pct else "\n측정 실패")
        return

    out = f"/tmp/prof_{key}.txt"
    with open(out, "w") as f:
        pstats.Stats(prof, stream=f).sort_stats("tottime").print_stats(45)
    print(f"\n===== {label}: tottime 상위 =====")
    pstats.Stats(prof).sort_stats("tottime").print_stats(24)
    print(f"\n전체 저장: {out}")


if __name__ == "__main__":
    main()
