#!/usr/bin/env python3
"""
sensor_layer.launch.py
──────────────────────
EBIMU IMU + SLLIDAR(기종별) + TF 관리 노드 통합 실행

포함 노드:
  1. tf_manager.launch.py (tf_manager_cpp) — static TF + wheel odom TF
  2. (include) sllidar_ros2/launch/sllidar_t1_launch.py — LiDAR T1 실행
  3. (include) ebimu_pkg/launch/ebimu.launch.py — EBIMU 9DOF IMU → /imu/data

사용법:
  ros2 launch sensor_layer sensor_layer.launch.py

T1(UDP) 파라미터 지정:
  ros2 launch sensor_layer sensor_layer.launch.py udp_ip:=192.168.11.2 udp_port:=8089
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():

    # ── LiDAR T1(UDP) 파라미터 (sllidar_t1_launch.py로 전달) ───────────
    channel_type = LaunchConfiguration("channel_type")
    udp_ip = LaunchConfiguration("udp_ip")
    udp_port = LaunchConfiguration("udp_port")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    angle_offset = LaunchConfiguration("angle_offset")
    scan_mode = LaunchConfiguration("scan_mode")
    scan_frequency = LaunchConfiguration("scan_frequency")

    # ── EBIMU 파라미터 ───────────────────────────────────────────────
    ebimu_port = LaunchConfiguration('ebimu_port')
    ebimu_baud = LaunchConfiguration('ebimu_baud')
    use_ebimu = LaunchConfiguration('use_ebimu')
    use_wheel_odom_tf = LaunchConfiguration('use_wheel_odom_tf')

    return LaunchDescription([

        # ── Declare Arguments ────────────────────────────────────────
        DeclareLaunchArgument("channel_type", default_value="udp", description="LiDAR 채널 타입 (T1은 udp)"),
        DeclareLaunchArgument("udp_ip", default_value="192.168.11.2", description="SLLIDAR T1 UDP IP"),
        DeclareLaunchArgument("udp_port", default_value="8089", description="SLLIDAR T1 UDP Port"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0", description="SLLIDAR serial port"),
        DeclareLaunchArgument("serial_baudrate", default_value="1000000", description="SLLIDAR serial baudrate"),
        DeclareLaunchArgument("frame_id", default_value="laser", description="LiDAR frame_id"),
        DeclareLaunchArgument("inverted", default_value="false", description="스캔 데이터 반전 여부"),
        DeclareLaunchArgument("angle_compensate", default_value="false", description="각도 보정 여부"),
        DeclareLaunchArgument("angle_offset", default_value="3.141592653589793", description="LiDAR 드라이버 레벨 스캔 각도 오프셋(rad). 원본 커밋값(π)으로 복원됨"),
        DeclareLaunchArgument("scan_mode", default_value="Sensitivity", description="스캔 모드"),
        DeclareLaunchArgument("scan_frequency", default_value="40.0", description="목표 스캔 주파수(Hz)"),

        DeclareLaunchArgument('ebimu_port', default_value='/dev/ttyUSB0', description='EBIMU 시리얼 포트'),
        DeclareLaunchArgument('ebimu_baud', default_value='115200', description='EBIMU 통신 속도'),
        DeclareLaunchArgument('use_ebimu', default_value='false', description='EBIMU 노드 실행 여부(true/false)'),
        DeclareLaunchArgument('use_wheel_odom_tf', default_value='true', description='wheel odom TF 실행 여부(true/false)'),

        LogInfo(msg='=== sensor_layer 시작: EBIMU IMU + SLLIDAR T1 + TF ==='),

        # ── 1) TF 관리자 include ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tf_manager_cpp"),
                    "launch",
                    "tf_manager.launch.py",
                )
            ),
            launch_arguments={
                "use_wheel_odom_tf": use_wheel_odom_tf,
            }.items(),
        ),

        # ── 2) LiDAR include ───────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("sllidar_ros2"),
                    "launch",
                    "sllidar_t1_launch.py",
                )
            ),
            launch_arguments={
                "channel_type": channel_type,
                "udp_ip": udp_ip,
                "udp_port": udp_port,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "angle_offset": angle_offset,
                "scan_mode": scan_mode,
                "scan_frequency": scan_frequency,
            }.items(),
        ),

        # ── 3) EBIMU IMU include ─────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ebimu_pkg"),
                    "launch",
                    "ebimu.launch.py",
                )
            ),
            condition=IfCondition(use_ebimu),
            launch_arguments={
                "port": ebimu_port,
                "baud": ebimu_baud,
            }.items(),
        ),
    ])
