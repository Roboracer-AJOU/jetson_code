import os
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from local_cpu_policy import cpu_policy_actions, ensure_policy_script_executable, local_cpu_prefix
from localization_launch_common import (
    amcl_stack_with_map,
    delayed_amcl_stack,
    is_enabled,
    register_lidar_network_bringup,
    sensor_bringup_include,
    sensor_launch_arguments,
)

_LOCAL_CPU = local_cpu_prefix()


def _ensure_amcl_packages_installed() -> None:
    missing = []
    for pkg in ('nav2_amcl', 'nav2_lifecycle_manager'):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(pkg)
    if missing:
        raise RuntimeError(
            'AMCL packages not installed: '
            f"{', '.join(missing)}.\n"
            'Install once, then re-run launch:\n'
            '  sudo apt update\n'
            '  sudo apt install ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager'
        )


def _launch_setup(context, *args, **kwargs):
    _ensure_amcl_packages_installed()
    enable_sensor_bringup = is_enabled(
        LaunchConfiguration('enable_sensor_bringup').perform(context)
    )
    enable_lidar_network_setup = is_enabled(
        LaunchConfiguration('enable_lidar_network_setup').perform(context)
    )
    amcl_delay = float(
        LaunchConfiguration('amcl_startup_delay_sec').perform(context)
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
                period=max(amcl_delay + 1.0, 2.0),
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

    policy = cpu_policy_actions(apply_delay_sec=max(amcl_delay + 2.0, 4.0), start_daemon=True)

    def _after_network(context):
        return [
            LogInfo(msg='=== AMCL localization: network ready, starting sensors ==='),
            sensor_bringup_include(),
            *delayed_amcl_stack(context, amcl_delay),
            *rviz_actions,
            *policy,
        ]

    if enable_sensor_bringup and enable_lidar_network_setup:
        return register_lidar_network_bringup(_after_network)

    if enable_sensor_bringup:
        return [
            LogInfo(msg='=== AMCL localization: starting sensors (network setup skipped) ==='),
            sensor_bringup_include(),
            *delayed_amcl_stack(context, amcl_delay),
            *rviz_actions,
            *policy,
        ]

    return [
        *amcl_stack_with_map(context, 0.0),
        *policy,
    ]


def generate_launch_description():
    ensure_policy_script_executable()
    maps_dir = '/home/nvidia/f1tenth_ajou/maps'
    default_map_yaml = os.path.join(
        maps_dir,
        'cartographer_map_20260822_164229_rosmap.yaml',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument(
            'map_yaml_filename',
            default_value=default_map_yaml,
            description='ROS map yaml (points to png/pgm image). AMCL uses this, not pbstream.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic used by wheel odometry',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Wheel odometry topic (vesc_wheel_odom -> AMCL)',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Raw LaserScan topic (filtered to /scan_amcl for AMCL)',
        ),
        DeclareLaunchArgument(
            'amcl_scan_max_range_m',
            default_value='8.0',
            description='Max LiDAR range for AMCL (loop track: opposite wall causes drift if too high)',
        ),
        DeclareLaunchArgument(
            'enable_sensor_bringup',
            default_value='true',
            description='Include sensor bringup for IMU/LiDAR/TF when true',
        ),
        *sensor_launch_arguments(),
        DeclareLaunchArgument(
            'use_wheel_odom_tf',
            default_value='false',
            description='Keep false; vesc_wheel_odom publishes odom->base_link TF for AMCL',
        ),
        DeclareLaunchArgument(
            'amcl_startup_delay_sec',
            default_value='6.0',
            description='Delay after sensor start before AMCL stack',
        ),
        DeclareLaunchArgument(
            'wait_for_rviz_initial_pose',
            default_value='false',
            description='Wait for RViz 2D Pose Estimate instead of saved mapping origin',
        ),
        DeclareLaunchArgument(
            'use_saved_mapping_origin',
            default_value='true',
            description='Use matching *_origin.yaml when wait_for_rviz_initial_pose is false',
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
            'use_rviz',
            default_value='false',
            description='Launch RViz on this machine (needs DISPLAY; use false over SSH)',
        ),
        LogInfo(msg=(
            '=== AMCL localization (yaml + png map from Cartographer mapping) ===\n'
            '  ros2 launch localization_layer amcl_localization_launch.py\n'
            '  map_yaml_filename:=.../maps/<name>_rosmap.yaml  (yaml -> png)\n'
            '  Requires: sudo apt install ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager\n'
            '  Cartographer localization과 동시에 켜지 않게 할 것 (map->odom TF 충돌)\n'
            '  안 맞으면: wait_for_rviz_initial_pose:=true + RViz 2D Pose Estimate'
        )),
        OpaqueFunction(function=_launch_setup),
    ])
