"""Launch the independent, subscribe-only telemetry CSV logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    log_hz = LaunchConfiguration("log_hz")
    output_root = LaunchConfiguration("output_root")
    imu_sample_hz = LaunchConfiguration("imu_sample_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_hz", default_value="20.0"),
            DeclareLaunchArgument(
                "output_root", default_value="~/f1tenth_ajou/logs"
            ),
            DeclareLaunchArgument("imu_sample_hz", default_value="20.0"),
            Node(
                package="path_following",
                executable="vehicle_measurement_node",
                name="vehicle_measurement_node",
                output="screen",
            ),
            Node(
                package="path_following",
                executable="csv_logger_node",
                name="csv_logger_node",
                output="screen",
                parameters=[
                    {
                        "log_hz": ParameterValue(log_hz, value_type=float),
                        "output_root": output_root,
                        "imu_sample_hz": ParameterValue(
                            imu_sample_hz, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
