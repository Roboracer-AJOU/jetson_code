"""Launch helpers: pin localization nodes to CPU 0-4 / sensors to CPU 5."""

import os

from launch.actions import ExecuteProcess, LogInfo, TimerAction

# 8코어 배치 (nvpmodel MAXN 필요):
#   CPU 0-4 = 로컬 처리 (cartographer, odom, map, rviz) — 스케줄러가 알아서 분배
#   CPU 5   = 로컬 센서 드라이버 전용 (LiDAR, IMU). 경합을 없애 스캔 주기를 지킨다
#   CPU 6-7 = path_following 전용 (path_cpu_policy.py)
# Node(prefix=...) expects a single string (shlex-split later). A list joins
# without spaces and becomes 'taskset-c0-4'.
LOCAL_CPU_PREFIX = "taskset -c 0-4"
SENSOR_CPU_PREFIX = "taskset -c 5"
_POLICY = "/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"


def local_cpu_prefix():
    return LOCAL_CPU_PREFIX


def sensor_cpu_prefix():
    """LiDAR/IMU 드라이버 전용. 다른 로컬 노드와 코어를 공유하지 않는다."""
    return SENSOR_CPU_PREFIX


def cpu_policy_actions(*, apply_delay_sec: float = 3.0, start_daemon: bool = True):
    actions = [
        TimerAction(
            period=apply_delay_sec,
            actions=[
                LogInfo(msg="=== CPU policy: apply (local 0-4, sensor 5, path 6-7) ==="),
                ExecuteProcess(
                    cmd=["bash", _POLICY, "--once"],
                    output="screen",
                ),
            ],
        ),
    ]
    if start_daemon:
        actions.append(
            ExecuteProcess(
                cmd=["bash", _POLICY, "--daemon"],
                output="log",
            )
        )
    return actions


def ensure_policy_script_executable():
    try:
        os.chmod(_POLICY, 0o755)
    except OSError:
        pass
