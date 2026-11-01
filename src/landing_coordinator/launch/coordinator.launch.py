"""Launch the coordinator with explicit, unverified-by-default landing sites."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Provide two common-frame sites without assuming their geometry is safe."""
    defaults = {
        'site_frame_id': 'map',
        'site_a_x': '0.0',
        'site_a_y': '0.0',
        'site_a_z': '0.0',
        'site_a_radius_m': '0.75',
        'site_a_verified': 'false',
        'site_b_x': '0.0',
        'site_b_y': '0.0',
        'site_b_z': '0.0',
        'site_b_radius_m': '0.75',
        'site_b_verified': 'false',
    }
    parameters = {}
    for name in defaults:
        conversion = str if name == 'site_frame_id' else (
            bool if name.endswith('_verified') else float
        )
        parameters[name] = ParameterValue(
            LaunchConfiguration(name), value_type=conversion
        )
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value)
         for name, value in defaults.items()]
        + [
            Node(
                package='landing_coordinator',
                executable='landing_coordinator_node',
                name='landing_coordinator',
                output='screen',
                parameters=[parameters],
            ),
        ]
    )
