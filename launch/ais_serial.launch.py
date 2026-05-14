from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("aivn_sensorpack_ais"),
        "config",
        "ais_serial.yaml",
    ])

    ais_serial_port_name = LaunchConfiguration("ais_serial_port_name")
    ais_baud_rate = LaunchConfiguration("ais_baud_rate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ais_serial_port_name",
            default_value="/dev/ttyUSB0",
            description="AIS serial device path, e.g. /dev/ttyUSB0 or /dev/serial/by-id/...",
        ),
        DeclareLaunchArgument(
            "ais_baud_rate",
            default_value="38400",
            description="AIS serial baud rate",
        ),
        Node(
            package="aivn_sensorpack_ais",
            executable="ais_serial_node",
            name="ais_serial_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "ais_serial_port_name": ais_serial_port_name,
                    "ais_baud_rate": ParameterValue(ais_baud_rate, value_type=int),
                },
            ],
        ),
    ])
