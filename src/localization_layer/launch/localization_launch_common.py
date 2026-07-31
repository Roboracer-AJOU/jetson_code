"""Shared helpers for localization_layer launch files."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def is_enabled(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def sensor_launch_arguments() -> list:
    """LiDAR/IMU bringup arguments shared by mapping and localization launches."""
    return [
        DeclareLaunchArgument(
            'ebimu_port',
            default_value='/dev/ttyUSB0',
            description='EBIMU serial port',
        ),
        DeclareLaunchArgument(
            'ebimu_baud',
            default_value='115200',
            description='EBIMU baud rate',
        ),
        DeclareLaunchArgument(
            'use_ebimu',
            default_value='true',
            description='Run EBIMU node when sensor bringup is enabled',
        ),
        DeclareLaunchArgument(
            'lidar_channel_type',
            default_value='udp',
            description='LiDAR channel type',
        ),
        DeclareLaunchArgument(
            'lidar_udp_ip',
            default_value='192.168.11.2',
            description='LiDAR UDP IP',
        ),
        DeclareLaunchArgument(
            'lidar_udp_port',
            default_value='8089',
            description='LiDAR UDP port',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='LiDAR frame_id',
        ),
        DeclareLaunchArgument(
            'lidar_inverted',
            default_value='false',
            description='LiDAR inverted flag',
        ),
        DeclareLaunchArgument(
            'lidar_angle_compensate',
            default_value='false',
            description='LiDAR angle compensation flag',
        ),
        DeclareLaunchArgument(
            'lidar_scan_mode',
            default_value='Sensitivity',
            description='LiDAR scan mode',
        ),
        DeclareLaunchArgument(
            'lidar_scan_frequency',
            default_value='40.0',
            description='LiDAR scan frequency in Hz (localization default 40)',
        ),
        DeclareLaunchArgument(
            'use_wheel_odom_tf',
            default_value='false',
            description='Must stay false for Cartographer localization (no wheel odom)',
        ),
        DeclareLaunchArgument(
            'imu_startup_delay_sec',
            default_value='1.0',
            description='Delay after TF before starting IMU',
        ),
        DeclareLaunchArgument(
            'lidar_startup_delay_sec',
            default_value='1.0',
            description='Delay after TF before starting LiDAR',
        ),
        DeclareLaunchArgument(
            'enable_lidar_network_setup',
            default_value='true',
            description='Configure host IP for LiDAR UDP before starting sensors',
        ),
        DeclareLaunchArgument(
            'lidar_network_interface',
            default_value='enP8p1s0',
            description='Network interface connected to LiDAR',
        ),
        DeclareLaunchArgument(
            'lidar_host_ip',
            default_value='192.168.11.3',
            description='Host IP on LiDAR subnet',
        ),
        DeclareLaunchArgument(
            'lidar_network_prefix',
            default_value='24',
            description='Host IP prefix length for LiDAR subnet',
        ),
    ]


def sensor_bringup_include():
    localization_dir = get_package_share_directory('localization_layer')
    mapping_sensor_launch = os.path.join(
        localization_dir,
        'launch',
        'mapping_sensor_bringup_launch.py',
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_sensor_launch),
        launch_arguments={
            'channel_type': LaunchConfiguration('lidar_channel_type'),
            'udp_ip': LaunchConfiguration('lidar_udp_ip'),
            'udp_port': LaunchConfiguration('lidar_udp_port'),
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'inverted': LaunchConfiguration('lidar_inverted'),
            'angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
            'scan_mode': LaunchConfiguration('lidar_scan_mode'),
            'scan_frequency': LaunchConfiguration('lidar_scan_frequency'),
            'ebimu_port': LaunchConfiguration('ebimu_port'),
            'ebimu_baud': LaunchConfiguration('ebimu_baud'),
            'use_ebimu': LaunchConfiguration('use_ebimu'),
            'use_wheel_odom_tf': LaunchConfiguration('use_wheel_odom_tf'),
            'imu_startup_delay_sec': LaunchConfiguration('imu_startup_delay_sec'),
            'lidar_startup_delay_sec': LaunchConfiguration('lidar_startup_delay_sec'),
        }.items(),
    )


def lidar_network_setup_process() -> ExecuteProcess:
    localization_prefix = get_package_prefix('localization_layer')
    network_setup_script = os.path.join(
        localization_prefix,
        'lib',
        'localization_layer',
        'setup_lidar_network.sh',
    )
    return ExecuteProcess(
        cmd=[
            network_setup_script,
            LaunchConfiguration('lidar_network_interface'),
            LaunchConfiguration('lidar_host_ip'),
            LaunchConfiguration('lidar_network_prefix'),
            LaunchConfiguration('lidar_udp_ip'),
        ],
        output='screen',
        name='setup_lidar_network',
    )


def make_lidar_network_exit_handler(post_setup_actions_fn):
    """Return a ProcessExited handler that aborts launch if network setup fails."""

    def _on_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=(
                    'ERROR: LiDAR network setup failed '
                    f'(exit {event.returncode}). Jetson cannot reach LiDAR 192.168.11.2. '
                    'Check LiDAR power/Ethernet, then run once:\n'
                    '  sudo ros2 run localization_layer install_lidar_network.sh'
                )),
                Shutdown(reason='LiDAR network setup failed'),
            ]
        return post_setup_actions_fn(context)

    return _on_exit


def register_lidar_network_bringup(post_setup_actions_fn):
    network_setup = lidar_network_setup_process()
    return [
        LogInfo(msg='=== configuring LiDAR network ==='),
        network_setup,
        RegisterEventHandler(
            OnProcessExit(
                target_action=network_setup,
                on_exit=make_lidar_network_exit_handler(post_setup_actions_fn),
            )
        ),
    ]


def resolve_static_map_yaml(pbstream_path: str) -> str | None:
    """Find ROS map yaml for RViz (/static_map) from pbstream path."""
    stem = os.path.splitext(pbstream_path)[0]
    origin_yaml = f'{stem}_origin.yaml'
    if os.path.isfile(origin_yaml):
        try:
            import yaml

            with open(origin_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            ros_map = data.get('ros_map_yaml')
            if isinstance(ros_map, str):
                if not os.path.isabs(ros_map):
                    ros_map = os.path.join(os.path.dirname(origin_yaml), ros_map)
                if os.path.isfile(ros_map):
                    return ros_map
        except OSError:
            pass

    for candidate in (f'{stem}.yaml', f'{stem}_rosmap.yaml'):
        if os.path.isfile(candidate):
            return candidate
    return None


def build_localization_cartographer_nodes(context):
    localization_dir = get_package_share_directory('localization_layer')
    config_dir = os.path.join(localization_dir, 'config')
    config_basename = 'cartographer_2d_localization.lua'

    pbstream_filename = LaunchConfiguration('pbstream_filename')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_initial_pose_reset = LaunchConfiguration('enable_initial_pose_reset')
    use_saved_mapping_origin = LaunchConfiguration('use_saved_mapping_origin')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    pbstream_path = pbstream_filename.perform(context)
    if pbstream_path and not os.path.isfile(pbstream_path):
        raise RuntimeError(
            f'pbstream not found: {pbstream_path}. '
            'Use pbstream_filename:=/home/nvidia/f1tenth_ajou/maps/<your_map>.pbstream'
        )

    initial_pose_delay = float(
        LaunchConfiguration('initial_pose_startup_delay_sec').perform(context)
    )
    wait_for_rviz = is_enabled(
        LaunchConfiguration('wait_for_rviz_initial_pose').perform(context)
    )

    imu_topic_str = imu_topic.perform(context)
    scan_topic_str = scan_topic.perform(context)

    nodes = [
        LogInfo(msg=(
            'Cartographer localization: LiDAR only (/scan). '
            'IMU/odom 미사용. FixedRatioSampler 경고는 정상.'
        )),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename,
                '-load_state_filename', pbstream_filename,
                '-load_frozen_state=true',
                '-start_trajectory_with_default_topics=false',
            ],
            remappings=[
                ('imu', imu_topic_str),
                ('odom', '/dev/null_odom'),
                ('scan', scan_topic_str),
            ],
        ),
    ]

    if is_enabled(enable_initial_pose_reset.perform(context)):
        nodes.append(
            TimerAction(
                period=initial_pose_delay,
                actions=[
                    Node(
                        package='localization_layer',
                        executable='localization_initial_pose_setter.py',
                        name='localization_initial_pose_setter',
                        output='screen',
                        parameters=[{
                            'pbstream_filename': pbstream_path,
                            'configuration_directory': config_dir,
                            'configuration_basename': config_basename,
                            'use_saved_mapping_origin': use_saved_mapping_origin,
                            'wait_for_rviz_initial_pose': wait_for_rviz,
                            'scan_topic': scan_topic,
                            'initial_pose_x': initial_pose_x,
                            'initial_pose_y': initial_pose_y,
                            'initial_pose_yaw': initial_pose_yaw,
                            # OFF: auto refine/relock was snapping into unknown / outside walls.
                            # Cartographer matches pbstream; rosmap is only for RViz.
                            'refine_with_scan_matching': False,
                        }],
                    ),
                ],
            )
        )

    return nodes


def build_static_map_publisher_nodes(context):
    pbstream_path = LaunchConfiguration('pbstream_filename').perform(context)
    map_yaml = resolve_static_map_yaml(pbstream_path)
    if not map_yaml:
        raise RuntimeError(
            f'No ROS map yaml for {pbstream_path}. '
            f'Expected {os.path.splitext(pbstream_path)[0]}_rosmap.yaml'
        )

    return [
        LogInfo(msg=f'=== localization: publishing saved map on /map ({map_yaml}) ==='),
        Node(
            package='localization_layer',
            executable='static_map_publisher.py',
            name='static_map_publisher',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'topic': '/map',
                'publish_period_sec': 1.0,
            }],
        ),
    ]


def localization_stack_with_map(context, cartographer_delay_sec: float):
    """Publish /map immediately; start Cartographer after sensor warmup delay."""
    return [
        *build_static_map_publisher_nodes(context),
        TimerAction(
            period=cartographer_delay_sec,
            actions=build_localization_cartographer_nodes(context),
        ),
    ]


def delayed_cartographer_stack(context, delay_sec: float):
    return localization_stack_with_map(context, delay_sec)
