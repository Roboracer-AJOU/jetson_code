import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from local_cpu_policy import cpu_policy_actions, ensure_policy_script_executable, local_cpu_prefix
from localization_launch_common import (
    delayed_cartographer_stack,
    is_enabled,
    register_lidar_network_bringup,
    sensor_bringup_include,
    sensor_launch_arguments,
)

_LOCAL_CPU = local_cpu_prefix()


def _launch_setup(context, *args, **kwargs):
    enable_sensor_bringup = is_enabled(
        LaunchConfiguration('enable_sensor_bringup').perform(context)
    )
    enable_lidar_network_setup = is_enabled(
        LaunchConfiguration('enable_lidar_network_setup').perform(context)
    )
    cartographer_delay = float(
        LaunchConfiguration('cartographer_startup_delay_sec').perform(context)
    )
    use_rviz = is_enabled(LaunchConfiguration('use_rviz').perform(context))

    rviz_actions = []
    if use_rviz:
        rviz_config = os.path.join(
            get_package_share_directory('localization_layer'),
            'rviz',
            'localization.rviz',
        )
        rviz_actions.append(
            TimerAction(
                period=max(cartographer_delay + 1.0, 2.0),
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        prefix=_LOCAL_CPU,
                        arguments=['-d', rviz_config],
                    ),
                ],
            )
        )

    policy = cpu_policy_actions(apply_delay_sec=max(cartographer_delay + 2.0, 4.0), start_daemon=True)

    def _after_network(context):
        return [
            LogInfo(msg='=== localization: network ready, starting sensors ==='),
            sensor_bringup_include(),
            *delayed_cartographer_stack(context, cartographer_delay),
            *rviz_actions,
            *policy,
        ]

    if enable_sensor_bringup and enable_lidar_network_setup:
        return register_lidar_network_bringup(_after_network)

    if enable_sensor_bringup:
        return [
            LogInfo(msg='=== localization: starting sensors (network setup skipped) ==='),
            sensor_bringup_include(),
            *delayed_cartographer_stack(context, cartographer_delay),
            *rviz_actions,
            *policy,
        ]

    from localization_launch_common import localization_stack_with_map
    return [
        *localization_stack_with_map(context, 0.0),
        *policy,
    ]


def generate_launch_description():
    ensure_policy_script_executable()
    maps_dir = '/home/nvidia/f1tenth_ajou/maps'
    # path_following 의 raceline.csv / centerline.csv 와 static/integrated
    # obstacle_node 의 map_name 이 모두 이 맵 기준이다. 맵을 바꾸려면 네 곳을
    # 같이 바꿔야 한다 (여기 + 장애물 노드 2개 + CSV 재생성).
    default_pbstream = os.path.join(
        maps_dir,
        # 20260816: config/centerline.csv, raceline.csv 가 이 맵 좌표계로 뽑혀
        # 있다. 211739 로 두면 프레임 원점이 달라서 레이스라인 785 점 중 719 점이
        # 벽 안으로 떨어진다 — 출발과 동시에 벽으로 간다.
        'cartographer_map_20260820_014643.pbstream',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument(
            'pbstream_filename',
            default_value=default_pbstream,
            description='Absolute path to .pbstream map file',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic used by Cartographer',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Wheel odometry topic (vesc_wheel_odom -> Cartographer)',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used by Cartographer',
        ),
        DeclareLaunchArgument(
            'enable_sensor_bringup',
            default_value='true',
            description='Include sensor bringup for IMU/LiDAR/TF when true',
        ),
        *sensor_launch_arguments(),
        DeclareLaunchArgument(
            'cartographer_startup_delay_sec',
            default_value='6.0',
            description='Delay after sensor start before Cartographer',
        ),
        DeclareLaunchArgument(
            'enable_initial_pose_reset',
            default_value='true',
            description='Run localization pose manager (finish auto trajectory + set pose)',
        ),
        DeclareLaunchArgument(
            'wait_for_rviz_initial_pose',
            default_value='false',
            description='Wait for RViz 2D Pose Estimate instead of saved mapping origin',
        ),
        DeclareLaunchArgument(
            'use_saved_mapping_origin',
            default_value='true',
            description='Use <pbstream_stem>_origin.yaml when wait_for_rviz_initial_pose is false',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='nan',
            description='Optional manual initial pose x in map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='nan',
            description='Optional manual initial pose y in map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='nan',
            description='Optional manual initial pose yaw in radians',
        ),
        DeclareLaunchArgument(
            'initial_pose_startup_delay_sec',
            default_value='2.0',
            description='Delay after cartographer start before pose manager runs',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz on this machine (needs DISPLAY; use false over SSH)',
        ),
        LogInfo(msg=(
            '=== localization: LiDAR main + wheel odom + IMU(gyro) ===\n'
            '  ros2 launch localization_layer cartographer_localization_launch.py\n'
            '  → control_node 먼저 (speed/servo) 후 Localization OK 확인\n'
            '  안 맞으면: wait_for_rviz_initial_pose:=true + RViz 2D Pose Estimate'
        )),
        OpaqueFunction(function=_launch_setup),
    ])
