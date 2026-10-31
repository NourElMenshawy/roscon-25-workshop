from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("gz_model").perform(context)
    sensor = LaunchConfiguration("camera_sensor_name").perform(context)
    count = int(LaunchConfiguration("num_vehicles").perform(context))

    if count < 1:
        raise RuntimeError("num_vehicles must be >= 1")

    actions = []
    for index in range(count):
        namespace = f"px4_{index}"
        entity = f"{model}_{index}"
        camera_base = (
            f"/world/{world}/model/{entity}"
            f"/link/camera_link/sensor/{sensor}"
        )
        image_topic = f"{camera_base}/image"
        camera_info_topic = f"{camera_base}/camera_info"

        # Gazebo image -> /px4_N/camera
        actions.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                namespace=namespace,
                name="aruco_camera_image_bridge",
                arguments=[image_topic],
                remappings=[
                    (image_topic, f"/{namespace}/camera"),
                    (
                        f"{image_topic}/compressed",
                        f"/{namespace}/camera/compressed",
                    ),
                ],
                output="screen",
            )
        )

        # Gazebo camera calibration -> /px4_N/camera_info
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=namespace,
                name="aruco_camera_info_bridge",
                parameters=[
                    {
                        "bridge_names": ["camera_info"],
                        "bridges.camera_info.gz_topic_name": camera_info_topic,
                        "bridges.camera_info.ros_topic_name": (
                            f"/{namespace}/camera_info"
                        ),
                        "bridges.camera_info.gz_type_name": "gz.msgs.CameraInfo",
                        "bridges.camera_info.ros_type_name": (
                            "sensor_msgs/msg/CameraInfo"
                        ),
                        "bridges.camera_info.direction": "GZ_TO_ROS",
                        "bridges.camera_info.qos_profile": "SENSOR_DATA",
                    }
                ],
                output="screen",
            )
        )

        # Relative C++ topics resolve to /px4_N/{camera,camera_info,target_pose}.
        actions.append(
            Node(
                package="aruco_tracker",
                executable="aruco_tracker",
                namespace=namespace,
                name="aruco_tracker",
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_vehicles", default_value="2"),
            DeclareLaunchArgument("world_name", default_value="aruco"),
            DeclareLaunchArgument(
                "gz_model", default_value="x500_mono_cam_down"
            ),
            DeclareLaunchArgument(
                "camera_sensor_name", default_value="imager"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
