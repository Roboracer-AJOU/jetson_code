#!/usr/bin/env python3
"""
맵핑 전용 센서 bringup.

네트워크 설정이 끝난 뒤 TF -> IMU -> LiDAR 순으로 올린다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from local_cpu_policy import local_cpu_prefix

_LOCAL_CPU = local_cpu_prefix()


def _sensor_stack_actions(
    *,
    use_ebimu,
    use_wheel_odom_tf,
    imu_startup_delay_sec,
    lidar_startup_delay_sec,
    channel_type,
    udp_ip,
    udp_port,
    serial_port,
    serial_baudrate,
    frame_id,
    inverted,
    angle_compensate,
    angle_offset,
    scan_mode,
    scan_frequency,
    ebimu_port,
    ebimu_baud,
):
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        prefix=_LOCAL_CPU,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'channel_type': channel_type,
            'udp_ip': udp_ip,
            'udp_port': ParameterValue(udp_port, value_type=int),
            'serial_port': serial_port,
            'serial_baudrate': ParameterValue(serial_baudrate, value_type=int),
            'frame_id': frame_id,
            'inverted': ParameterValue(inverted, value_type=bool),
            'angle_compensate': ParameterValue(angle_compensate, value_type=bool),
            'angle_offset': ParameterValue(angle_offset, value_type=float),
            'scan_mode': scan_mode,
            'scan_frequency': ParameterValue(scan_frequency, value_type=float),
        }],
    )

    ebimu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_driver',
        name='ebimu_driver',
        output='screen',
        prefix=_LOCAL_CPU,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'port': ebimu_port,
            'baud': ebimu_baud,
            'accel_in_g': True,
        }],
    )

    return [
        LogInfo(msg='=== mapping sensor bringup: starting TF ==='),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tf_manager_cpp'),
                    'launch',
                    'tf_manager.launch.py',
                )
            ),
            launch_arguments={
                'use_wheel_odom_tf': use_wheel_odom_tf,
            }.items(),
        ),
        TimerAction(
            period=imu_startup_delay_sec,
            condition=IfCondition(use_ebimu),
            actions=[
                LogInfo(msg='=== mapping sensor bringup: starting EBIMU ==='),
                ebimu_node,
            ],
        ),
        TimerAction(
            period=lidar_startup_delay_sec,
            actions=[
                LogInfo(msg='=== LiDAR starting (/scan) — verify: ros2 topic hz /scan ==='),
                lidar_node,
            ],
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    use_ebimu = LaunchConfiguration('use_ebimu')
    use_wheel_odom_tf = LaunchConfiguration('use_wheel_odom_tf')
    imu_startup_delay_sec = float(
        LaunchConfiguration('imu_startup_delay_sec').perform(context)
    )
    lidar_startup_delay_sec = float(
        LaunchConfiguration('lidar_startup_delay_sec').perform(context)
    )
    channel_type = LaunchConfiguration('channel_type')
    udp_ip = LaunchConfiguration('udp_ip')
    udp_port = LaunchConfiguration('udp_port')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    angle_offset = LaunchConfiguration('angle_offset')
    scan_mode = LaunchConfiguration('scan_mode')
    scan_frequency = LaunchConfiguration('scan_frequency')
    ebimu_port = LaunchConfiguration('ebimu_port')
    ebimu_baud = LaunchConfiguration('ebimu_baud')

    return _sensor_stack_actions(
        use_ebimu=use_ebimu,
        use_wheel_odom_tf=use_wheel_odom_tf,
        imu_startup_delay_sec=imu_startup_delay_sec,
        lidar_startup_delay_sec=lidar_startup_delay_sec,
        channel_type=channel_type,
        udp_ip=udp_ip,
        udp_port=udp_port,
        serial_port=serial_port,
        serial_baudrate=serial_baudrate,
        frame_id=frame_id,
        inverted=inverted,
        angle_compensate=angle_compensate,
        angle_offset=angle_offset,
        scan_mode=scan_mode,
        scan_frequency=scan_frequency,
        ebimu_port=ebimu_port,
        ebimu_baud=ebimu_baud,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('channel_type', default_value='udp'),
        DeclareLaunchArgument('udp_ip', default_value='192.168.11.2'),
        DeclareLaunchArgument('udp_port', default_value='8089'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='1000000'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='false'),
        DeclareLaunchArgument(
            'angle_offset',
            default_value='3.141592653589793',
            description='LiDAR scan angle offset (rad)',
        ),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
        DeclareLaunchArgument('scan_frequency', default_value='40.0'),
        DeclareLaunchArgument('ebimu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('ebimu_baud', default_value='115200'),
        DeclareLaunchArgument('use_ebimu', default_value='true'),
        DeclareLaunchArgument('use_wheel_odom_tf', default_value='false'),
        DeclareLaunchArgument(
            'imu_startup_delay_sec',
            default_value='0.3',
            description='Delay after TF before starting IMU (seconds)',
        ),
        DeclareLaunchArgument(
            'lidar_startup_delay_sec',
            default_value='1.0',
            description='Delay after TF before starting LiDAR (seconds)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
