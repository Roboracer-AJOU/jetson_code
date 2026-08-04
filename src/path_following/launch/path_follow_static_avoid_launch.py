# 정적 장애 회피 스택 (static_obstacle_node)
# path_follow_stanley_launch.py 와 동일 구성 — 정적 전용임을 명확히 한 별칭 런치.
#
#   ros2 launch path_following path_follow_static_avoid_launch.py
#   bash ~/f1tenth_ajou/scripts/run_control_node.sh

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from path_cpu_policy import cpu_policy_actions, ensure_policy_script_executable, path_cpu_prefix

_QUIET = ["--ros-args", "--log-level", "warn"]
_CPU = path_cpu_prefix()


def generate_launch_description():
    ensure_policy_script_executable()
    enable_vehicle_control = LaunchConfiguration("enable_vehicle_control")
    status_log_hz = LaunchConfiguration("status_log_hz")
    verbose_logs = LaunchConfiguration("verbose_logs")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_vehicle_control",
                default_value="false",
                description="Run control_node in launch. 실차는 별도 터미널 ros2 run 권장.",
            ),
            DeclareLaunchArgument(
                "status_log_hz",
                default_value="2.0",
                description="Stanley STATUS 로그 주기(Hz).",
            ),
            DeclareLaunchArgument(
                "verbose_logs",
                default_value="false",
                description="local_planner 상세 로그.",
            ),
            Node(
                package="path_following",
                executable="static_obstacle_node",
                name="static_obstacle_node",
                output="screen",
                prefix=_CPU,
                arguments=_QUIET,
            ),
            Node(
                package="path_following",
                executable="fgm_node",
                name="fgm_node",
                output="screen",
                prefix=_CPU,
                arguments=_QUIET,
            ),
            Node(
                package="path_following",
                executable="local_planner_node",
                name="local_planner_node",
                output="screen",
                prefix=_CPU,
                arguments=_QUIET,
                parameters=[
                    {
                        "verbose_logs": ParameterValue(
                            verbose_logs, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="path_following",
                executable="stanley_waypoint_follow_node",
                name="stanley_waypoint_follow_node",
                output="screen",
                prefix=_CPU,
                parameters=[
                    {
                        "status_log_hz": ParameterValue(
                            status_log_hz, value_type=float
                        ),
                        "stanley_debug_log_hz": 0.0,
                    }
                ],
            ),
            Node(
                package="path_following",
                executable="stack_status_node",
                name="stack_status_node",
                output="screen",
                prefix=_CPU,
                parameters=[{"period_s": 1.0}],
            ),
            Node(
                package="path_following",
                executable="control_node",
                name="vehicle_control_node",
                output="screen",
                prefix=_CPU,
                condition=IfCondition(enable_vehicle_control),
                arguments=_QUIET,
            ),
            *cpu_policy_actions(apply_delay_sec=2.0, start_daemon=True),
        ]
    )
