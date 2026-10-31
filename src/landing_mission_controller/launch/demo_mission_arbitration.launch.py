from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_launch = (
        Path(get_package_share_directory("landing_mission_controller"))
        / "launch"
        / "mission_controllers.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("drone_a_delay_ms", default_value="100"),
        DeclareLaunchArgument("drone_b_delay_ms", default_value="300"),
        Node(
            package="landing_coordinator",
            executable="landing_coordinator_node",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(controller_launch)),
            launch_arguments={
                "drone_a_target_topic": "/demo/drone_a/target_pose",
                "drone_b_target_topic": "/demo/drone_b/target_pose",
                "drone_a_delay_ms": LaunchConfiguration("drone_a_delay_ms"),
                "drone_b_delay_ms": LaunchConfiguration("drone_b_delay_ms"),
            }.items(),
        ),
        Node(
            package="landing_mission_controller",
            executable="mock_target_publisher",
            namespace="drone_a",
            parameters=[{"target_topic": "/demo/drone_a/target_pose"}],
            output="screen",
        ),
        Node(
            package="landing_mission_controller",
            executable="mock_target_publisher",
            namespace="drone_b",
            parameters=[{"target_topic": "/demo/drone_b/target_pose"}],
            output="screen",
        ),
    ])
