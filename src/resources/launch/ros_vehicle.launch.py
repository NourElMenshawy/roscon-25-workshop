from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_ros_vehicle_actions(context):
    resource_package = LaunchConfiguration(
        "resource_package"
    ).perform(context)

    urdf_model = LaunchConfiguration(
        "urdf_model"
    ).perform(context)

    namespace = LaunchConfiguration(
        "namespace"
    ).perform(context)

    tf_prefix = LaunchConfiguration(
        "tf_prefix"
    ).perform(context)

    gz_entity_name = LaunchConfiguration(
        "gz_entity_name"
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

    package_share = Path(
        get_package_share_directory(
            resource_package
        )
    )

    urdf_path = (
        package_share
        / "urdf"
        / f"{urdf_model}.urdf"
    )

    if not urdf_path.is_file():
        raise RuntimeError(
            f"URDF does not exist: "
            f"{urdf_path}"
        )

    robot_description = urdf_path.read_text(
        encoding="utf-8"
    )

    camera_sensor_frame = (
        f"{gz_entity_name}/camera_link/imager"
    )

    actions = []

    # ==========================================================
    # ROBOT STRUCTURE
    #
    # px4_0/base_link
    # ├── px4_0/motor_0
    # ├── px4_0/motor_1
    # ├── px4_0/motor_2
    # └── px4_0/motor_3
    # ==========================================================

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description":
                        robot_description,

                    "frame_prefix":
                        tf_prefix,

                    "use_sim_time":
                        True,
                }
            ],
        )
    )

    # ==========================================================
    # PX4 ESTIMATED POSE
    #
    # px4_0/odom
    #        ↓ static
    # px4_0/odom_ned
    #        ↓ dynamic VehicleOdometry
    # px4_0/base_link_frd
    #        ↓ static
    # px4_0/base_link
    # ==========================================================

    actions.append(
        Node(
            package="px4_tf",
            executable="px4_tf_publisher",
            namespace=namespace,
            name="px4_tf_publisher",
            output="screen",
            parameters=[
                {
                    "px4_tf_prefix":
                        tf_prefix,

                    "use_sim_time":
                        True,
                }
            ],
        )
    )

    # ==========================================================
    # APPLICATION / WORLD FRAME
    #
    # map
    #  ↓
    # px4_0/odom
    #
    # PX4 odometry is local, so we place its local origin at the
    # Gazebo spawn position.
    # ==========================================================

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=namespace,
            name="map_to_odom",
            output="screen",
            arguments=[
                "--x",
                spawn_pos_x,

                "--y",
                spawn_pos_y,

                "--z",
                spawn_pos_z,

                "--roll",
                "0.0",

                "--pitch",
                "0.0",

                "--yaw",
                "0.0",

                "--frame-id",
                "map",

                "--child-frame-id",
                f"{tf_prefix}odom",
            ],
            parameters=[
                {
                    "use_sim_time":
                        True,
                }
            ],
        )
    )

    # ==========================================================
    # OPTIONAL GAZEBO CAMERA SENSOR FRAME
    #
    # px4_0/base_link
    #        ↓
    # x500_mono_cam_down_0/camera_link/imager
    #
    # This is the adapter used by the workshop for the
    # down-facing mono camera.
    # ==========================================================

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=namespace,
            name="camera_sensor_tf",
            output="screen",

            condition=IfCondition(
                LaunchConfiguration(
                    "has_downward_camera"
                )
            ),

            arguments=[
                "--x",
                "0.0",

                "--y",
                "0.0",

                "--z",
                "0.0",

                "--qx",
                "-0.7071068",

                "--qy",
                "0.7071068",

                "--qz",
                "0.0",

                "--qw",
                "0.0",

                "--frame-id",
                f"{tf_prefix}base_link",

                "--child-frame-id",
                camera_sensor_frame,
            ],

            parameters=[
                {
                    "use_sim_time":
                        True,
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "resource_package",
                default_value="resources",
            ),

            DeclareLaunchArgument(
                "urdf_model",
                default_value="x500",
            ),

            DeclareLaunchArgument(
                "namespace",
                default_value="px4_0",
            ),

            DeclareLaunchArgument(
                "tf_prefix",
                default_value="px4_0/",
            ),

            DeclareLaunchArgument(
                "gz_entity_name",
                default_value="x500_0",
            ),

            DeclareLaunchArgument(
                "has_downward_camera",
                default_value="false",
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
                function=_create_ros_vehicle_actions
            ),
        ]
    )
