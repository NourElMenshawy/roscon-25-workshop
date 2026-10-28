from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "processing_mode",
                default_value="stdexec",
                description="Processing mode: sync or stdexec",
            ),

            DeclareLaunchArgument(
                "processing_delay_ms",
                default_value="0",
                description="Artificial processing delay in milliseconds",
            ),

            Node(
                package="precision_land_executor",
                executable="precision_land_executor",
                name="precision_land_executor",
                namespace="px4_0",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "processing_mode": ParameterValue(
                            LaunchConfiguration("processing_mode"),
                            value_type=str,
                        ),
                        "processing_delay_ms": ParameterValue(
                            LaunchConfiguration("processing_delay_ms"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )

