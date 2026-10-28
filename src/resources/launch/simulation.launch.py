from os import path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _create_vehicle_actions(context):
    resource_package = LaunchConfiguration(
        "resource_package"
    ).perform(context)

    pkg_share = get_package_share_directory(
        resource_package
    )

    num_vehicles = int(
        LaunchConfiguration(
            "num_vehicles"
        ).perform(context)
    )

    if num_vehicles < 1:
        raise RuntimeError(
            "num_vehicles must be >= 1"
        )

    gz_model = LaunchConfiguration(
        "gz_model"
    ).perform(context)

    urdf_model = LaunchConfiguration(
        "urdf_model"
    ).perform(context)

    has_downward_camera = LaunchConfiguration(
        "has_downward_camera"
    ).perform(context)

    px4_autopilot_path = path.expanduser(
        LaunchConfiguration(
            "px4_autopilot_path"
        ).perform(context)
    )

    model_path = path.expanduser(
        LaunchConfiguration(
            "model_path"
        ).perform(context)
    )

    world = LaunchConfiguration(
        "world"
    ).perform(context)

    px4_autostart_id = LaunchConfiguration(
        "px4_autostart_id"
    ).perform(context)

    xrce_agent_port = LaunchConfiguration(
        "xrce_agent_port"
    ).perform(context)

    origin_x = float(
        LaunchConfiguration(
            "spawn_origin_x"
        ).perform(context)
    )

    origin_y = float(
        LaunchConfiguration(
            "spawn_origin_y"
        ).perform(context)
    )

    origin_z = float(
        LaunchConfiguration(
            "spawn_origin_z"
        ).perform(context)
    )

    spacing = float(
        LaunchConfiguration(
            "vehicle_spacing"
        ).perform(context)
    )

    px4_vehicle_launch = path.join(
        pkg_share,
        "launch",
        "px4_vehicle.launch.py",
    )

    ros_vehicle_launch = path.join(
        pkg_share,
        "launch",
        "ros_vehicle.launch.py",
    )

    actions = []

    for instance in range(num_vehicles):
        entity_name = f"{gz_model}_{instance}"
        namespace = f"px4_{instance}"
        tf_prefix = f"{namespace}/"

        spawn_x = (
            origin_x
            + instance * spacing
        )
        spawn_y = origin_y
        spawn_z = origin_z

        # --------------------------------------------------
        # Simulation / PX4 side
        # --------------------------------------------------

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    px4_vehicle_launch
                ),
                launch_arguments={
                    "px4_autopilot_path":
                        px4_autopilot_path,

                    "px4_instance":
                        str(instance),

                    "px4_ns":
                        namespace,

                    "world":
                        world,

                    "model":
                        gz_model,

                    "entity_name":
                        entity_name,

                    "model_path":
                        model_path,

                    "px4_autostart_id":
                        px4_autostart_id,

                    "xrce_agent_port":
                        xrce_agent_port,

                    "spawn_pos_x":
                        str(spawn_x),

                    "spawn_pos_y":
                        str(spawn_y),

                    "spawn_pos_z":
                        str(spawn_z),
                }.items(),
            )
        )

        # --------------------------------------------------
        # ROS representation side
        # --------------------------------------------------

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    ros_vehicle_launch
                ),
                launch_arguments={
                    "resource_package":
                        resource_package,

                    "urdf_model":
                        urdf_model,

                    "namespace":
                        namespace,

                    "tf_prefix":
                        tf_prefix,

                    "gz_entity_name":
                        entity_name,

                    "has_downward_camera":
                        has_downward_camera,

                    "spawn_pos_x":
                        str(spawn_x),

                    "spawn_pos_y":
                        str(spawn_y),

                    "spawn_pos_z":
                        str(spawn_z),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory(
        "resources"
    )

    return LaunchDescription(
        [
            # --------------------------------------------------
            # General configuration
            # --------------------------------------------------

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
                "resource_package",
                default_value="resources",
            ),

            DeclareLaunchArgument(
                "world",
                default_value="default",
            ),

            # Gazebo representation
            DeclareLaunchArgument(
                "gz_model",
                default_value="x500",
                description=(
                    "PX4/Gazebo model"
                ),
            ),

            # ROS representation
            DeclareLaunchArgument(
                "urdf_model",
                default_value="x500",
                description=(
                    "ROS URDF model"
                ),
            ),

            DeclareLaunchArgument(
                "has_downward_camera",
                default_value="false",
            ),

            # --------------------------------------------------
            # Multi vehicle configuration
            # --------------------------------------------------

            DeclareLaunchArgument(
                "num_vehicles",
                default_value="1",
            ),

            DeclareLaunchArgument(
                "vehicle_spacing",
                default_value="3.0",
            ),

            DeclareLaunchArgument(
                "spawn_origin_x",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "spawn_origin_y",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "spawn_origin_z",
                default_value="0.0",
            ),

            # --------------------------------------------------
            # Gazebo model file
            # --------------------------------------------------

            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [
                        LaunchConfiguration(
                            "px4_autopilot_path"
                        ),
                        "Tools",
                        "simulation",
                        "gz",
                        "models",
                        LaunchConfiguration(
                            "gz_model"
                        ),
                        "model.sdf",
                    ]
                ),
            ),

            # --------------------------------------------------
            # PX4
            # --------------------------------------------------

            DeclareLaunchArgument(
                "px4_autostart_id",
                default_value="4001",
            ),

            DeclareLaunchArgument(
                "xrce_agent_port",
                default_value="8888",
            ),

            # --------------------------------------------------
            # Gazebo resources
            # --------------------------------------------------

            DeclareLaunchArgument(
                "extra_gz_resource_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "resources"
                        ),
                        "models",
                    ]
                ),
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
                "enable_foxglove",
                default_value="false",
            ),

            # ==================================================
            # SYSTEM SINGLETONS
            # ==================================================

            # Gazebo + ROS/Gazebo bridge
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    path.join(
                        pkg_share,
                        "launch",
                        "gz_world.launch.py",
                    )
                ),
                launch_arguments={
                    "px4_autopilot_path":
                        LaunchConfiguration(
                            "px4_autopilot_path"
                        ),

                    "world":
                        LaunchConfiguration(
                            "world"
                        ),

                    "extra_gz_resource_path":
                        LaunchConfiguration(
                            "extra_gz_resource_path"
                        ),

                    "bridge_config_file":
                        LaunchConfiguration(
                            "bridge_config_file"
                        ),
                }.items(),
            ),

            # One XRCE Agent for all vehicles
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    path.join(
                        pkg_share,
                        "launch",
                        "px4_ros.launch.py",
                    )
                ),
                launch_arguments={
                    "xrce_agent_port":
                        LaunchConfiguration(
                            "xrce_agent_port"
                        ),
                }.items(),
            ),

            # One visualization bridge
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    path.join(
                        pkg_share,
                        "launch",
                        "visualization.launch.py",
                    )
                ),
                condition=IfCondition(
                    LaunchConfiguration(
                        "enable_foxglove"
                    )
                ),
            ),

            # ==================================================
            # PER VEHICLE
            # ==================================================

            OpaqueFunction(
                function=_create_vehicle_actions
            ),
        ]
    )
