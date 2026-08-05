"""Launch helpers: pin localization nodes to cores 1-4 and enforce CPU policy."""

import os

from launch.actions import ExecuteProcess, LogInfo, TimerAction

# 코어1~4 = CPU 0-3
# Node(prefix=...) expects a single string (shlex-split later). A list joins
# without spaces and becomes 'taskset-c0-3'.
LOCAL_CPU_PREFIX = "taskset -c 0-3"
_POLICY = "/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"


def local_cpu_prefix():
    return LOCAL_CPU_PREFIX


def cpu_policy_actions(*, apply_delay_sec: float = 3.0, start_daemon: bool = True):
    actions = [
        TimerAction(
            period=apply_delay_sec,
            actions=[
                LogInfo(msg="=== CPU policy: apply (local cores 1-4, path 5-6) ==="),
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
