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
        "ais_topic_bridge.yaml",
    ])

    input_topic_name = LaunchConfiguration("input_topic_name")
    ais_topic_name = LaunchConfiguration("ais_topic_name")
    ais_source_port = LaunchConfiguration("ais_source_port")
    ais_debug_publish_reason = LaunchConfiguration("ais_debug_publish_reason")
    ais_debug_fragment = LaunchConfiguration("ais_debug_fragment")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic_name",
            default_value="/sensor_pack/navigation/novatel_isro_p2/w2k_60003",
            description="Input std_msgs/String topic carrying AIS NMEA lines",
        ),
        DeclareLaunchArgument(
            "ais_topic_name",
            default_value="/sensor_pack/external/ais/ship",
            description="Output aivn_interfaces/msg/AisShip topic",
        ),
        DeclareLaunchArgument(
            "ais_source_port",
            default_value="w2k_60003",
            description="source_port field to stamp into published AisShip messages",
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use bag /clock time instead of wall time",
        ),
        Node(
            package="aivn_sensorpack_ais",
            executable="ais_topic_bridge_node",
            name="ais_topic_bridge_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "input_topic_name": ParameterValue(input_topic_name, value_type=str),
                    "ais_topic_name": ParameterValue(ais_topic_name, value_type=str),
                    "ais_source_port": ParameterValue(ais_source_port, value_type=str),
                    "ais_debug_publish_reason": ParameterValue(
                        ais_debug_publish_reason, value_type=bool
                    ),
                    "ais_debug_fragment": ParameterValue(
                        ais_debug_fragment, value_type=bool
                    ),
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
    ])
