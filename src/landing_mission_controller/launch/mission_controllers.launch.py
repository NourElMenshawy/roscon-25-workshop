from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    coordinator = LaunchConfiguration("coordinator_service")
    return LaunchDescription([
        DeclareLaunchArgument(
            "coordinator_service",
            default_value="/landing_coordinator/request_landing",
        ),
        DeclareLaunchArgument("drone_a_target_topic", default_value="/px4_0/target_pose"),
        DeclareLaunchArgument("drone_b_target_topic", default_value="/px4_1/target_pose"),
        DeclareLaunchArgument("drone_a_delay_ms", default_value="100"),
        DeclareLaunchArgument("drone_b_delay_ms", default_value="300"),
        Node(
            package="landing_mission_controller",
            executable="landing_mission_controller_node",
            namespace="drone_a",
            name="landing_mission_controller",
            parameters=[{
                "drone_id": "drone_a",
                "target_topic": LaunchConfiguration("drone_a_target_topic"),
                "coordinator_service": coordinator,
                "request_delay_ms": ParameterValue(
                    LaunchConfiguration("drone_a_delay_ms"), value_type=int
                ),
                "observation_timeout_ms": 1500,
                "response_timeout_ms": 3000,
            }],
            output="screen",
        ),
        Node(
            package="landing_mission_controller",
            executable="landing_mission_controller_node",
            namespace="drone_b",
            name="landing_mission_controller",
            parameters=[{
                "drone_id": "drone_b",
                "target_topic": LaunchConfiguration("drone_b_target_topic"),
                "coordinator_service": coordinator,
                "request_delay_ms": ParameterValue(
                    LaunchConfiguration("drone_b_delay_ms"), value_type=int
                ),
                "observation_timeout_ms": 1500,
                "response_timeout_ms": 3000,
            }],
            output="screen",
        ),
    ])
