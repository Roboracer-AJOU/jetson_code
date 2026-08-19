"""Launch helpers: pin localization nodes to CPU 0-4 / sensors to CPU 5."""

import os

from launch.actions import ExecuteProcess, LogInfo, TimerAction

# 8코어 배치 (nvpmodel MAXN 필요):
#   CPU 0-4 = 로컬 처리 (cartographer, odom, map, rviz) — 스케줄러가 알아서 분배
#   CPU 5   = 로컬 센서 드라이버 (LiDAR, IMU). path_following 과 공유
#   CPU 5-7 = path_following (path_cpu_policy.py). 5번은 센서와 공유, 6-7은 전용
# Node(prefix=...) expects a single string (shlex-split later). A list joins
# without spaces and becomes 'taskset-c0-4'.
LOCAL_CPU_PREFIX = "taskset -c 0-4"
SENSOR_CPU_PREFIX = "taskset -c 5"
_POLICY = "/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"


def local_cpu_prefix():
    return LOCAL_CPU_PREFIX


def sensor_cpu_prefix():
    """LiDAR/IMU 드라이버. 로컬 처리(0-4)와는 분리, path_following 과는 공유."""
    return SENSOR_CPU_PREFIX


def cpu_policy_actions(*, apply_delay_sec: float = 3.0, start_daemon: bool = True):
    actions = [
        TimerAction(
            period=apply_delay_sec,
            actions=[
                LogInfo(msg="=== CPU policy: apply (local 0-4, sensor 5, path 5-7) ==="),
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
