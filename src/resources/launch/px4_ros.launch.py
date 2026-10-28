from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "xrce_agent_executable",
                default_value="MicroXRCEAgent",
                description=(
                    "apt install Micro XRCE-DDS Agent"
                    "snap install micro-xrce-dds-agent"
                ),
            ),

            DeclareLaunchArgument(
                "xrce_agent_port",
                default_value="8888",
                description=(
                    "UDP port used by the "
                    "Micro XRCE-DDS Agent"
                ),
            ),

            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("xrce_agent_executable"),
                    "udp4",
                    "--port",
                    LaunchConfiguration(
                        "xrce_agent_port"
                    ),
                    "-v",
                    "1",
                ],
                name="microxrce_agent",
                output="screen",
            ),
        ]
    )
