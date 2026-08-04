"""Launch helpers: pin path-following nodes to cores 5-6 and enforce CPU policy."""

import os

from launch.actions import ExecuteProcess, LogInfo, TimerAction

# 코어5~6 = CPU 4-5
PATH_CPU_PREFIX = ["taskset", "-c", "4-5"]
_POLICY = "/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"


def path_cpu_prefix():
    return list(PATH_CPU_PREFIX)


def cpu_policy_actions(*, apply_delay_sec: float = 2.0, start_daemon: bool = True):
    """Apply policy after nodes start; optional daemon keeps control_node covered."""
    actions = [
        TimerAction(
            period=apply_delay_sec,
            actions=[
                LogInfo(msg="=== CPU policy: apply (path cores 5-6, local 1-4) ==="),
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
