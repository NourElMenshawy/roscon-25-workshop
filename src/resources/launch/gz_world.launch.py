from os import environ, path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _is_true(value: str) -> bool:
    return value.lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _join_paths(*entries: str) -> str:
    return ":".join(
        entry
        for entry in entries
        if entry
    )


def _launch_setup(context):
    px4_autopilot_path = path.expanduser(
        LaunchConfiguration(
            "px4_autopilot_path"
        ).perform(context)
    )

    world_name = LaunchConfiguration(
        "world"
    ).perform(context)

    extra_gz_resource_path = (
        LaunchConfiguration(
            "extra_gz_resource_path"
        ).perform(context)
    )

    bridge_config_file = (
        LaunchConfiguration(
            "bridge_config_file"
        ).perform(context)
    )

    headless = _is_true(
        LaunchConfiguration(
            "headless"
        ).perform(context)
    )

    # --------------------------------------------------
    # PX4 Gazebo paths
    # --------------------------------------------------

    px4_gz_root = path.join(
        px4_autopilot_path,
        "Tools",
        "simulation",
        "gz",
    )

    px4_worlds_path = path.join(
        px4_gz_root,
        "worlds",
    )

    px4_models_path = path.join(
        px4_gz_root,
        "models",
    )

    world_file = path.join(
        px4_worlds_path,
        f"{world_name}.sdf",
    )

    px4_gz_plugin_path = path.join(
        px4_autopilot_path,
        "build",
        "px4_sitl_default",
        "src",
        "modules",
        "simulation",
        "gz_plugins",
    )

    px4_gz_server_config_path = path.join(
        px4_autopilot_path,
        "src",
        "modules",
        "simulation",
        "gz_bridge",
        "server.config",
    )

    # --------------------------------------------------
    # Validation
    # --------------------------------------------------

    if not path.isdir(px4_autopilot_path):
        raise RuntimeError(
            "PX4 path does not exist: "
            f"{px4_autopilot_path}"
        )

    if not path.isfile(world_file):
        raise RuntimeError(
            "Gazebo world does not exist: "
            f"{world_file}"
        )

    if not path.isfile(
        px4_gz_server_config_path
    ):
        raise RuntimeError(
            "PX4 Gazebo server config does not exist: "
            f"{px4_gz_server_config_path}"
        )

    if not path.isfile(bridge_config_file):
        raise RuntimeError(
            "ROS/Gazebo bridge config does not exist: "
            f"{bridge_config_file}"
        )

    # --------------------------------------------------
    # Environment
    # --------------------------------------------------

    resource_path = _join_paths(
        environ.get(
            "GZ_SIM_RESOURCE_PATH",
            "",
        ),
        px4_models_path,
        px4_worlds_path,
        extra_gz_resource_path,
    )

    plugin_path = _join_paths(
        environ.get(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            "",
        ),
        px4_gz_plugin_path,
    )

    actions = [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            resource_path,
        ),

        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            plugin_path,
        ),

        #
        # This is critical for PX4.
        #
        # server.config loads:
        #   Physics
        #   UserCommands
        #   SceneBroadcaster
        #   Sensors
        #   IMU
        #   AirPressure
        #   Magnetometer
        #   ...
        #
        SetEnvironmentVariable(
            "GZ_SIM_SERVER_CONFIG_PATH",
            px4_gz_server_config_path,
        ),

        SetEnvironmentVariable(
            "GZ_IP",
            "127.0.0.1",
        ),
    ]

    # --------------------------------------------------
    # Gazebo
    #
    # Match PX4's own startup model:
    #
    #   gz sim -r <world.sdf>
    #
    # With headless:
    #
    #   gz sim -r <world.sdf> -s
    # --------------------------------------------------

    gz_cmd = [
        "gz",
        "sim",
        "-r",
        world_file,
    ]

    if headless:
        gz_cmd.append("-s")

    actions.append(
        ExecuteProcess(
            cmd=gz_cmd,
            name="gz_sim",
            output="screen",
            emulate_tty=True,
        )
    )

    # --------------------------------------------------
    # ROS <-> Gazebo bridge
    #
    # Gazebo itself is intentionally NOT owned by
    # ros_gz_sim here.
    # --------------------------------------------------

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros_bridge",
            output="screen",
            parameters=[
                {
                    "config_file":
                        bridge_config_file,
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_autopilot_path",
                default_value=EnvironmentVariable(
                    "PX4_PATH",
                    default_value=(
                        "~/PX4-Autopilot"
                    ),
                ),
            ),

            DeclareLaunchArgument(
                "world",
                default_value="default",
            ),

            DeclareLaunchArgument(
                "extra_gz_resource_path",
                default_value="",
            ),

            DeclareLaunchArgument(
                "bridge_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "resources"
                        ),
                        "cfg",
                        "clock_bridge.yaml",
                    ]
                ),
            ),

            DeclareLaunchArgument(
                "headless",
                default_value=EnvironmentVariable(
                    "HEADLESS",
                    default_value="false",
                ),
            ),

            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )
