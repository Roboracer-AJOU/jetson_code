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

from localization_launch_common import (
    delayed_cartographer_stack,
    is_enabled,
    register_lidar_network_bringup,
    sensor_bringup_include,
    sensor_launch_arguments,
)


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
                        arguments=['-d', rviz_config],
                    ),
                ],
            )
        )

    def _after_network(context):
        return [
            LogInfo(msg='=== localization: network ready, starting sensors ==='),
            sensor_bringup_include(),
            *delayed_cartographer_stack(context, cartographer_delay),
            *rviz_actions,
        ]

    if enable_sensor_bringup and enable_lidar_network_setup:
        return register_lidar_network_bringup(_after_network)

    if enable_sensor_bringup:
        return [
            LogInfo(msg='=== localization: starting sensors (network setup skipped) ==='),
            sensor_bringup_include(),
            *delayed_cartographer_stack(context, cartographer_delay),
            *rviz_actions,
        ]

    from localization_launch_common import localization_stack_with_map
    return localization_stack_with_map(context, 0.0)


def generate_launch_description():
    maps_dir = '/home/nvidia/f1tenth_ajou/maps'
    default_pbstream = os.path.join(
        maps_dir,
        'cartographer_map_20260803_202601.pbstream',
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
