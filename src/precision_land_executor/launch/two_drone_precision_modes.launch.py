"""Launch the two PX4 precision modes using the existing tf2 frame tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    count = int(LaunchConfiguration('num_vehicles').perform(context))
    if count != 2:
        raise RuntimeError('This demo expects exactly two drones')

    actions = []
    for index in range(count):
        actions.append(
            Node(
                package='precision_land_executor',
                executable='precision_land_executor',
                namespace=f'px4_{index}',
                name='CustomModeDemo',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'drone_id': ['drone_a', 'drone_b'][index],
                        'px4_system_id': index + 1,
                        'enable_auto_switch': ParameterValue(
                            LaunchConfiguration('enable_auto_switch'), value_type=bool
                        ),
                        'target_freshness_ms': 1500,
                        'common_frame_id': LaunchConfiguration('site_frame_id'),
                        'local_frame_id': f'px4_{index}/odom_ned',
                        'site_separation_margin_m': ParameterValue(
                            LaunchConfiguration('site_separation_margin_m'),
                            value_type=float,
                        ),
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    """Keep PX4 activation separate from tf2 follower-site validation."""
    options = {
        'num_vehicles': '2',
        'enable_auto_switch': 'false',
        'site_frame_id': 'map',
        'site_separation_margin_m': '2.0',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value)
         for name, value in options.items()]
        + [OpaqueFunction(function=_launch_setup)]
    )
