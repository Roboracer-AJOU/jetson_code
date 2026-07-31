import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from localization_launch_common import register_lidar_network_bringup


def _is_enabled(value: str) -> bool:
  return value.lower() in ('1', 'true', 'yes', 'on')


def _build_cartographer_stack(context):
  localization_dir = get_package_share_directory('localization_layer')
  config_dir_arg = LaunchConfiguration('configuration_directory').perform(context).strip()
  config_dir = config_dir_arg or os.path.join(localization_dir, 'config')

  enable_auto_save = LaunchConfiguration('enable_auto_save')
  map_save_dir = LaunchConfiguration('map_save_dir')
  map_file_prefix = LaunchConfiguration('map_file_prefix')
  include_unfinished_submaps = LaunchConfiguration('include_unfinished_submaps')
  save_on_shutdown = LaunchConfiguration('save_on_shutdown')
  save_interval_sec = LaunchConfiguration('save_interval_sec')
  export_ros_map = LaunchConfiguration('export_ros_map')
  export_ros_map_on_shutdown = LaunchConfiguration('export_ros_map_on_shutdown')
  ros_map_topic = LaunchConfiguration('ros_map_topic')
  ros_map_format = LaunchConfiguration('ros_map_format')
  ros_map_mode = LaunchConfiguration('ros_map_mode')
  ros_map_timeout_sec = LaunchConfiguration('ros_map_timeout_sec')
  write_state_timeout_sec = LaunchConfiguration('write_state_timeout_sec')
  shutdown_write_state_timeout_sec = LaunchConfiguration('shutdown_write_state_timeout_sec')
  shutdown_ros_map_timeout_sec = LaunchConfiguration('shutdown_ros_map_timeout_sec')
  imu_topic = LaunchConfiguration('imu_topic')
  odom_topic = LaunchConfiguration('odom_topic')
  scan_topic = LaunchConfiguration('scan_topic')
  configuration_basename = LaunchConfiguration('configuration_basename')

  imu_topic_str = imu_topic.perform(context)
  scan_topic_str = scan_topic.perform(context)

  return [
    LogInfo(msg='=== mapping: LiDAR only (no IMU/odom), starting Cartographer ==='),
    Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='log',
      arguments=[
        '-configuration_directory', config_dir,
        '-configuration_basename', configuration_basename,
      ],
      remappings=[
        ('imu', imu_topic_str),
        ('odom', '/unused_odom'),
        ('scan', scan_topic_str),
      ],
    ),
    Node(
      package='cartographer_ros',
      executable='cartographer_occupancy_grid_node',
      name='occupancy_grid_node',
      output='log',
      arguments=['-resolution', '0.05', '-publish_period_sec', '2.0'],
    ),
    Node(
      package='localization_layer',
      executable='map_auto_saver.py',
      name='map_auto_saver',
      output='log',
      condition=IfCondition(enable_auto_save),
      sigterm_timeout='60',
      sigkill_timeout='60',
      parameters=[{
        'map_save_dir': map_save_dir,
        'map_file_prefix': map_file_prefix,
        'include_unfinished_submaps': ParameterValue(
          include_unfinished_submaps, value_type=bool
        ),
        'save_on_shutdown': ParameterValue(save_on_shutdown, value_type=bool),
        'save_interval_sec': ParameterValue(save_interval_sec, value_type=float),
        'export_ros_map': ParameterValue(export_ros_map, value_type=bool),
        'ros_map_topic': ros_map_topic,
        'ros_map_format': ros_map_format,
        'ros_map_mode': ros_map_mode,
        'ros_map_timeout_sec': ParameterValue(ros_map_timeout_sec, value_type=float),
        'write_state_timeout_sec': ParameterValue(
          write_state_timeout_sec, value_type=float
        ),
        'export_ros_map_on_shutdown': ParameterValue(
          export_ros_map_on_shutdown, value_type=bool
        ),
        'shutdown_write_state_timeout_sec': ParameterValue(
          shutdown_write_state_timeout_sec, value_type=float
        ),
        'shutdown_ros_map_timeout_sec': ParameterValue(
          shutdown_ros_map_timeout_sec, value_type=float
        ),
      }],
    ),
  ]


def _build_sensor_bringup():
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


def _launch_setup(context, *args, **kwargs):
  enable_sensor_bringup = _is_enabled(
    LaunchConfiguration('enable_sensor_bringup').perform(context)
  )
  enable_lidar_network_setup = _is_enabled(
    LaunchConfiguration('enable_lidar_network_setup').perform(context)
  )
  cartographer_delay = float(
    LaunchConfiguration('cartographer_startup_delay_sec').perform(context)
  )

  cartographer_stack = _build_cartographer_stack(context)
  actions = []

  def _after_network(context):
    return [
      LogInfo(msg='=== mapping: network ready, starting sensors ==='),
      _build_sensor_bringup(),
      TimerAction(
        period=cartographer_delay,
        actions=cartographer_stack,
      ),
    ]

  if enable_sensor_bringup and enable_lidar_network_setup:
    return register_lidar_network_bringup(_after_network)

  if enable_sensor_bringup:
    actions.append(LogInfo(msg='=== mapping: starting sensors (network setup skipped) ==='))
    actions.append(_build_sensor_bringup())
    actions.append(TimerAction(
      period=cartographer_delay,
      actions=cartographer_stack,
    ))
    return actions

  actions.extend(cartographer_stack)
  return actions


def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument(
      'enable_auto_save',
      default_value='true',
      description='Enable automatic pbstream export helper node',
    ),
    DeclareLaunchArgument(
      'map_save_dir',
      default_value='/home/nvidia/f1tenth_ajou/maps',
      description='Directory to store exported pbstream files',
    ),
    DeclareLaunchArgument(
      'map_file_prefix',
      default_value='cartographer_map',
      description='Prefix for exported pbstream file names',
    ),
    DeclareLaunchArgument(
      'include_unfinished_submaps',
      default_value='false',
      description='Include unfinished submaps when exporting (false = cleaner walls)',
    ),
    DeclareLaunchArgument(
      'save_on_shutdown',
      default_value='true',
      description='Export pbstream automatically when launch is stopped',
    ),
    DeclareLaunchArgument(
      'save_interval_sec',
      default_value='60.0',
      description='Periodic export interval in seconds (0 = shutdown only)',
    ),
    DeclareLaunchArgument(
      'export_ros_map',
      default_value='true',
      description='Also export ROS map image+yaml after pbstream save',
    ),
    DeclareLaunchArgument(
      'export_ros_map_on_shutdown',
      default_value='true',
      description='Also export ROS map image+yaml on shutdown auto-save',
    ),
    DeclareLaunchArgument(
      'ros_map_topic',
      default_value='/map',
      description='Topic used by map_saver_cli',
    ),
    DeclareLaunchArgument(
      'ros_map_format',
      default_value='png',
      description='Output image format for map_saver_cli (png/pgm)',
    ),
    DeclareLaunchArgument(
      'ros_map_mode',
      default_value='trinary',
      description='map_saver_cli mode: trinary/scale/raw',
    ),
    DeclareLaunchArgument(
      'ros_map_timeout_sec',
      default_value='45.0',
      description='Timeout for map_saver_cli command',
    ),
    DeclareLaunchArgument(
      'write_state_timeout_sec',
      default_value='120.0',
      description='Timeout for /write_state during periodic saves (large maps need time)',
    ),
    DeclareLaunchArgument(
      'shutdown_write_state_timeout_sec',
      default_value='120.0',
      description='Timeout for /write_state service response during shutdown save',
    ),
    DeclareLaunchArgument(
      'shutdown_ros_map_timeout_sec',
      default_value='60.0',
      description='Timeout for map_saver_cli during shutdown map export',
    ),
    DeclareLaunchArgument(
      'imu_topic',
      default_value='/imu/data',
      description='IMU topic used by Cartographer',
    ),
    DeclareLaunchArgument(
      'odom_topic',
      default_value='/odom',
      description='Unused (use_odometry=false); remapped away',
    ),
    DeclareLaunchArgument(
      'scan_topic',
      default_value='/scan',
      description='LaserScan topic used by Cartographer',
    ),
    DeclareLaunchArgument(
      'configuration_basename',
      default_value='cartographer_2d_mapping_imu_lidar_no_odom.lua',
      description='Cartographer Lua config basename',
    ),
    DeclareLaunchArgument(
      'configuration_directory',
      default_value='',
      description='Cartographer Lua config directory (default: localization_layer/config)',
    ),
    DeclareLaunchArgument(
      'enable_sensor_bringup',
      default_value='true',
      description='Include mapping sensor bringup for IMU/LiDAR/TF when true',
    ),
    DeclareLaunchArgument(
      'ebimu_port',
      default_value='/dev/ttyUSB0',
      description='EBIMU serial port (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'ebimu_baud',
      default_value='115200',
      description='EBIMU baud rate (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'use_ebimu',
      default_value='true',
      description='Run EBIMU node when sensor bringup is enabled',
    ),
    DeclareLaunchArgument(
      'lidar_channel_type',
      default_value='udp',
      description='LiDAR channel type (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_udp_ip',
      default_value='192.168.11.2',
      description='LiDAR UDP IP (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_udp_port',
      default_value='8089',
      description='LiDAR UDP port (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_frame_id',
      default_value='laser',
      description='LiDAR frame_id (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_inverted',
      default_value='false',
      description='LiDAR inverted flag (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_angle_compensate',
      default_value='false',
      description='LiDAR angle compensation flag (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_scan_mode',
      default_value='Sensitivity',
      description='LiDAR scan mode (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'lidar_scan_frequency',
      default_value='20.0',
      description='LiDAR scan frequency in Hz for mapping (used when sensor bringup is enabled)',
    ),
    DeclareLaunchArgument(
      'use_wheel_odom_tf',
      default_value='false',
      description='Must stay false: Cartographer mapping does not use wheel odom',
    ),
    DeclareLaunchArgument(
      'imu_startup_delay_sec',
      default_value='0.3',
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
    DeclareLaunchArgument(
      'cartographer_startup_delay_sec',
      default_value='6.0',
      description='Delay so EBIMU Full stream is up before Cartographer (use_imu_data=true)',
    ),
    OpaqueFunction(function=_launch_setup),
  ])
