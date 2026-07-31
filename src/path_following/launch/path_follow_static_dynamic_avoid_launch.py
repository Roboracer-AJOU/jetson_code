# 정적+동적 장애 회피 스택 (integrated_obstacle_node)
#
#   터미널 1: 런치
#     ros2 launch path_following path_follow_static_dynamic_avoid_launch.py
#   터미널 2: 고정 레이아웃 모니터 (숫자만 갱신)
#     ros2 run path_following drive_monitor
#   터미널 3: control_node
#     ros2 run path_following control_node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_QUIET = ["--ros-args", "--log-level", "warn"]


def generate_launch_description():
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
                arguments=_QUIET,
            ),
            Node(
                package="path_following",
                executable="fgm_node",
                name="fgm_node",
                output="screen",
                arguments=_QUIET,
            ),
            Node(
                package="path_following",
                executable="local_planner_node",
                name="local_planner_node",
                output="screen",
                arguments=_QUIET,
                parameters=[
                    {
                        "verbose_logs": ParameterValue(
                            verbose_logs, value_type=bool
                        ),
                        "status_log_hz": 0.0,
                    }
                ],
            ),
            Node(
                package="path_following",
                executable="stanley_waypoint_follow_node",
                name="stanley_waypoint_follow_node",
                output="screen",
                arguments=_QUIET,
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
                parameters=[{"period_s": 1.0}],
            ),
            Node(
                package="path_following",
                executable="control_node",
                name="vehicle_control_node",
                output="screen",
                condition=IfCondition(enable_vehicle_control),
                arguments=_QUIET,
            ),
        ]
    )
