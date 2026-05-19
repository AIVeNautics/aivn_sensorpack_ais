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
    ais_data_bits = LaunchConfiguration("ais_data_bits")
    ais_parity = LaunchConfiguration("ais_parity")
    ais_stop_bits = LaunchConfiguration("ais_stop_bits")
    ais_xonxoff = LaunchConfiguration("ais_xonxoff")
    ais_rtscts = LaunchConfiguration("ais_rtscts")
    ais_dsrdtr = LaunchConfiguration("ais_dsrdtr")
    ais_debug_hex_dump = LaunchConfiguration("ais_debug_hex_dump")
    ais_debug_publish_reason = LaunchConfiguration("ais_debug_publish_reason")
    ais_debug_fragment = LaunchConfiguration("ais_debug_fragment")

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
        DeclareLaunchArgument(
            "ais_data_bits",
            default_value="8",
            description="AIS serial data bits: 5, 6, 7, or 8",
        ),
        DeclareLaunchArgument(
            "ais_parity",
            default_value="N",
            description="AIS serial parity: N, E, O, M, or S",
        ),
        DeclareLaunchArgument(
            "ais_stop_bits",
            default_value="1",
            description="AIS serial stop bits: 1 or 2",
        ),
        DeclareLaunchArgument(
            "ais_xonxoff",
            default_value="false",
            description="Enable software flow control",
        ),
        DeclareLaunchArgument(
            "ais_rtscts",
            default_value="false",
            description="Enable RTS/CTS flow control",
        ),
        DeclareLaunchArgument(
            "ais_dsrdtr",
            default_value="false",
            description="Enable DSR/DTR flow control",
        ),
        DeclareLaunchArgument(
            "ais_debug_hex_dump",
            default_value="false",
            description="Log raw received bytes as hex/ascii previews",
        ),
        DeclareLaunchArgument(
            "ais_debug_publish_reason",
            default_value="false",
            description="Log publish/drop reasons",
        ),
        DeclareLaunchArgument(
            "ais_debug_fragment",
            default_value="false",
            description="Log fragment assembly status",
        ),
        Node(
            package="aivn_sensorpack_ais",
            executable="ais_serial_node",
            name="ais_serial_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "ais_serial_port_name": ParameterValue(
                        ais_serial_port_name,
                        value_type=str,
                    ),
                    "ais_baud_rate": ParameterValue(
                        ais_baud_rate,
                        value_type=int,
                    ),
                    "ais_data_bits": ParameterValue(
                        ais_data_bits,
                        value_type=int,
                    ),
                    "ais_parity": ParameterValue(
                        ais_parity,
                        value_type=str,
                    ),
                    "ais_stop_bits": ParameterValue(
                        ais_stop_bits,
                        value_type=int,
                    ),
                    "ais_xonxoff": ParameterValue(
                        ais_xonxoff,
                        value_type=bool,
                    ),
                    "ais_rtscts": ParameterValue(
                        ais_rtscts,
                        value_type=bool,
                    ),
                    "ais_dsrdtr": ParameterValue(
                        ais_dsrdtr,
                        value_type=bool,
                    ),
                    "ais_debug_hex_dump": ParameterValue(
                        ais_debug_hex_dump,
                        value_type=bool,
                    ),
                    "ais_debug_publish_reason": ParameterValue(
                        ais_debug_publish_reason,
                        value_type=bool,
                    ),
                    "ais_debug_fragment": ParameterValue(
                        ais_debug_fragment,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])