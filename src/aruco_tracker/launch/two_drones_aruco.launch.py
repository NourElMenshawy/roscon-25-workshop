"""Bridge two Gazebo downward cameras and launch one ArUco tracker per drone."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    count = int(LaunchConfiguration('num_vehicles').perform(context))
    if count != 2:
        raise RuntimeError('This competition expects two PX4 vehicles')

    world = LaunchConfiguration('world_name').perform(context)
    model = LaunchConfiguration('gz_model').perform(context)
    sensor = LaunchConfiguration('camera_sensor_name').perform(context)
    marker_size = float(LaunchConfiguration('marker_size_m').perform(context))
    if not world or not model or not sensor or marker_size <= 0.0:
        raise RuntimeError('Invalid world, model, camera sensor, or marker size')

    tracker_config = Path(get_package_share_directory('aruco_tracker')) / 'cfg' / 'params.yaml'
    if not tracker_config.is_file():
        raise RuntimeError(f'Missing ArUco tracker configuration: {tracker_config}')

    actions = []
    for index in range(count):
        namespace = f'px4_{index}'
        camera_base = (
            f'/world/{world}/model/{model}_{index}/link/camera_link/sensor/{sensor}'
        )
        image_topic = f'{camera_base}/image'
        camera_info_topic = f'{camera_base}/camera_info'

        actions.append(
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                name='aruco_image_bridge',
                output='screen',
                arguments=[image_topic],
                remappings=[
                    (image_topic, f'/{namespace}/camera'),
                    (f'{image_topic}/compressed', f'/{namespace}/camera/compressed'),
                ],
            )
        )
        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                name='aruco_camera_info_bridge',
                output='screen',
                parameters=[
                    {
                        'bridge_names': ['camera_info'],
                        'bridges.camera_info.ros_topic_name': f'/{namespace}/camera_info',
                        'bridges.camera_info.gz_topic_name': camera_info_topic,
                        'bridges.camera_info.ros_type_name': 'sensor_msgs/msg/CameraInfo',
                        'bridges.camera_info.gz_type_name': 'gz.msgs.CameraInfo',
                        'bridges.camera_info.direction': 'GZ_TO_ROS',
                        'bridges.camera_info.qos_profile': 'SENSOR_DATA',
                    }
                ],
            )
        )
        actions.append(
            Node(
                package='aruco_tracker',
                executable='aruco_tracker',
                namespace=namespace,
                name='aruco_tracker',
                output='screen',
                parameters=[str(tracker_config), {
                    'use_sim_time': True,
                    'marker_size_m': marker_size,
                }],
            )
        )

    return actions


def generate_launch_description():
    """Launch two independent camera bridges and tracker instances."""
    return LaunchDescription([
        DeclareLaunchArgument('num_vehicles', default_value='2'),
        DeclareLaunchArgument('world_name', default_value='aruco'),
        DeclareLaunchArgument('gz_model', default_value='x500_mono_cam_down'),
        DeclareLaunchArgument('camera_sensor_name', default_value='camera'),
        DeclareLaunchArgument('marker_size_m', default_value='0.5'),
        OpaqueFunction(function=_launch_setup),
    ])
