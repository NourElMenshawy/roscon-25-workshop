from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def _create_px4_vehicle_actions(context):
    px4_autopilot_path = Path(
        LaunchConfiguration(
            "px4_autopilot_path"
        ).perform(context)
    ).expanduser()

    px4_instance = int(
        LaunchConfiguration(
            "px4_instance"
        ).perform(context)
    )

    px4_ns = LaunchConfiguration(
        "px4_ns"
    ).perform(context)

    world = LaunchConfiguration(
        "world"
    ).perform(context)

    model = LaunchConfiguration(
        "model"
    ).perform(context)

    px4_autostart_id = LaunchConfiguration(
        "px4_autostart_id"
    ).perform(context)

    xrce_agent_port = LaunchConfiguration(
        "xrce_agent_port"
    ).perform(context)

    spawn_pos_x = LaunchConfiguration(
        "spawn_pos_x"
    ).perform(context)

    spawn_pos_y = LaunchConfiguration(
        "spawn_pos_y"
    ).perform(context)

    spawn_pos_z = LaunchConfiguration(
        "spawn_pos_z"
    ).perform(context)

    px4_binary = (
        px4_autopilot_path
        / "build"
        / "px4_sitl_default"
        / "bin"
        / "px4"
    )

    if not px4_binary.is_file():
        raise RuntimeError(
            f"PX4 SITL binary does not exist: {px4_binary}"
        )

    #
    # PX4 expects the Gazebo simulation model name
    # with the "gz_" prefix.
    #
    # x500                  -> gz_x500
    # x500_mono_cam_down    -> gz_x500_mono_cam_down
    #
    px4_sim_model = f"gz_{model}"

    #
    # PX4 will spawn the model itself into the
    # already-running Gazebo world.
    #
    spawn_pose = (
        f"{spawn_pos_x},"
        f"{spawn_pos_y},"
        f"{spawn_pos_z},"
        "0,0,0"
    )

    px4_process = ExecuteProcess(
        name=f"px4_{px4_instance}",
        cmd=[
            str(px4_binary),
            "-i",
            str(px4_instance),
        ],
        cwd=str(px4_autopilot_path),
        output="screen",
        emulate_tty=True,
        additional_env={
            #
            # Gazebo is already owned by gz_world.launch.py.
            #
            "PX4_GZ_STANDALONE":
                "1",

            #
            # Tell PX4 which Gazebo model to spawn.
            #
            "PX4_SIM_MODEL":
                px4_sim_model,

            #
            # Existing Gazebo world.
            #
            "PX4_GZ_WORLD":
                world,

            #
            # PX4 airframe configuration.
            #
            "PX4_SYS_AUTOSTART":
                px4_autostart_id,

            #
            # Spawn location.
            #
            "PX4_GZ_MODEL_POSE":
                spawn_pose,

            #
            # ROS 2 / DDS namespace.
            #
            "PX4_UXRCE_DDS_NS":
                px4_ns,

            #
            # Micro XRCE-DDS Agent port.
            #
            "PX4_UXRCE_DDS_PORT":
                xrce_agent_port,

            #
            # Gazebo transport.
            #
            "GZ_IP":
                "127.0.0.1",
        },
    )

    return [
        px4_process,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_autopilot_path",
            ),

            DeclareLaunchArgument(
                "px4_instance",
                default_value="0",
            ),

            DeclareLaunchArgument(
                "px4_ns",
                default_value="px4_0",
            ),

            DeclareLaunchArgument(
                "world",
                default_value="default",
            ),

            DeclareLaunchArgument(
                "model",
                default_value="x500",
            ),

            DeclareLaunchArgument(
                "px4_autostart_id",
                default_value="4001",
            ),

            DeclareLaunchArgument(
                "xrce_agent_port",
                default_value="8888",
            ),

            DeclareLaunchArgument(
                "spawn_pos_x",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "spawn_pos_y",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "spawn_pos_z",
                default_value="0.0",
            ),

            OpaqueFunction(
                function=_create_px4_vehicle_actions
            ),
        ]
    )
