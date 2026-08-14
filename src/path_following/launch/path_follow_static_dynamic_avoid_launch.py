# 정적+동적 장애 회피 스택 (integrated_obstacle_node)
#
#   터미널 1: 런치
#     ros2 launch path_following path_follow_static_dynamic_avoid_launch.py
#   터미널 2: 고정 레이아웃 모니터 (숫자만 갱신)
#     ros2 run path_following drive_monitor
#   터미널 3: control_node
#     bash ~/f1tenth_ajou/scripts/run_control_node.sh
#     # 또는: ros2 run path_following control_node  (데몬이 곧 코어/nice 적용)

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
    track = LaunchConfiguration("track")
    enable_aeb = LaunchConfiguration("enable_aeb")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_aeb",
                default_value="true",
                description=(
                    "비상 제동(AEB) 노드. /scan TTC 로 /emergency_brake 발행. "
                    "control_node 가 받아 역토크로 정지한다."
                ),
            ),
            DeclareLaunchArgument(
                "track",
                default_value="",
                description=(
                    "주행 라인: raceline | centerline | auto. "
                    "빈 값이면 track_sliding.DEFAULT_TRACK. "
                    "local_planner 와 stanley 에 동시에 적용된다."
                ),
            ),
            DeclareLaunchArgument(
                "enable_vehicle_control",
                default_value="false",
                description="Run control_node in launch. 실차는 별도 터미널 ros2 run 권장.",
            ),
            DeclareLaunchArgument(
                "status_log_hz",
                default_value="0.0",
                description="Stanley STATUS 스크롤 로그 Hz. 모니터 쓸 땐 0 권장.",
            ),
            DeclareLaunchArgument(
                "verbose_logs",
                default_value="false",
                description="local_planner 상세 로그.",
            ),
            Node(
                package="path_following",
                executable="integrated_obstacle_node",
                name="integrated_obstacle_node",
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
                        "track": ParameterValue(track, value_type=str),
                        "verbose_logs": ParameterValue(
                            verbose_logs, value_type=bool
                        ),
                        "status_log_hz": 0.0,
                    }
                ],
            ),
            Node(
                package="path_following",
                executable="emergency_brake_node",
                name="emergency_brake_node",
                output="screen",
                prefix=_CPU,
                arguments=_QUIET,
                condition=IfCondition(enable_aeb),
            ),
            Node(
                package="path_following",
                executable="stanley_waypoint_follow_node",
                name="stanley_waypoint_follow_node",
                output="screen",
                prefix=_CPU,
                arguments=_QUIET,
                parameters=[
                    {
                        "track": ParameterValue(track, value_type=str),
                        "status_log_hz": ParameterValue(
                            status_log_hz, value_type=float
                        ),
                        "stanley_debug_log_hz": 0.0,
                    }
                ],
            ),
            # stack_status_node: /scan 구독으로 CPU 큼 → 기본 미기동 (필요 시 수동 ros2 run)
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
