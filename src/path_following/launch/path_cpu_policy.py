"""Launch helpers: pin path-following nodes to CPU 5-7 and enforce CPU policy."""

import os

from launch.actions import ExecuteProcess, LogInfo, TimerAction

# 8코어 배치 (nvpmodel MAXN 필요):
#   CPU 0-4 = 로컬 처리 (local_cpu_policy.py)
#   CPU 5   = 로컬 센서 드라이버와 path_following 이 공유
#   CPU 5-7 = path_following (5번은 센서와 공유, 6-7은 전용)
# 코어 2개로는 패스 그룹이 195/200% 까지 차서 여유가 없었다. 로컬 처리는 0-4 에서
# 평균 22% 라 5번을 내줘도 되지만, 센서 드라이버는 5번에 그대로 남아 공유가 된다.
# Node(prefix=...) expects a single string (shlex-split later). A list joins
# without spaces and becomes 'taskset-c5-7'.
PATH_CPU_PREFIX = "taskset -c 5-7"
_POLICY = "/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"


def path_cpu_prefix():
    return PATH_CPU_PREFIX


def cpu_policy_actions(*, apply_delay_sec: float = 2.0, start_daemon: bool = True):
    """Apply policy after nodes start; optional daemon keeps control_node covered.

    별도 터미널에서 `ros2 run` 으로 띄우는 control_node / drive_monitor 는 런치
    prefix 가 안 붙으므로, 데몬이 주기적으로 CPU 5-7 로 다시 묶는다.
    """
    actions = [
        TimerAction(
            period=apply_delay_sec,
            actions=[
                LogInfo(msg="=== CPU policy: apply (path 5-7, local 0-4, sensor 5 공유) ==="),
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
