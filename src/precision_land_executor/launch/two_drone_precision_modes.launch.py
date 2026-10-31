"""Register a mission-aware PX4 precision-landing mode executor for each drone.

Start PX4 SITL, Gazebo, the ArUco trackers, and landing coordinator
before this launch; start mission controllers after both executors register.
Enable automatic switching only after both drones are safely airborne in Hold.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    count = int(LaunchConfiguration("num_vehicles").perform(context))
    if count != 2:
        raise RuntimeError("This step-6 demo expects exactly two drones")

    actions = []
    for index in range(count):
        actions.append(
            Node(
                package="precision_land_executor",
                executable="precision_land_executor",
                namespace=f"px4_{index}",
                name="CustomModeDemo",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "drone_id": f"drone_{'ab'[index]}",
                        "px4_system_id": index + 1,
                        "enable_auto_switch": ParameterValue(
                            LaunchConfiguration("enable_auto_switch"),
                            value_type=bool,
                        ),
                        "target_freshness_ms": 1500,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_vehicles", default_value="2"),
            DeclareLaunchArgument("enable_auto_switch", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
