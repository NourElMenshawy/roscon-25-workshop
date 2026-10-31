from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="landing_coordinator",
            executable="landing_coordinator_node",
            name="landing_coordinator",
            output="screen",
        ),
    ])
