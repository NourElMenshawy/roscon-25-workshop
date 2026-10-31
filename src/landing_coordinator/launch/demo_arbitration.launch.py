"""Run an isolated arbitration demonstration; no PX4/Gazebo required.

A is deliberately delayed less than B. The first request processed wins;
network and operating-system scheduling mean timing is not a strict priority.
"""

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
        Node(
            package="landing_coordinator",
            executable="mock_request_client",
            name="drone_a_requester",
            parameters=[{"drone_id": "drone_a", "delay_ms": 100}],
            output="screen",
        ),
        Node(
            package="landing_coordinator",
            executable="mock_request_client",
            name="drone_b_requester",
            parameters=[{"drone_id": "drone_b", "delay_ms": 300}],
            output="screen",
        ),
    ])
