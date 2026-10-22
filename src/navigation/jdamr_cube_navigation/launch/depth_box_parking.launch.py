"""Launch perception-only depth box parking."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start a depth observer that has no velocity publisher."""
    package_share = get_package_share_directory('jdamr_cube_navigation')
    parameters = os.path.join(
        package_share, 'config', 'depth_box_parking.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('table_components', default_value='false'),
        DeclareLaunchArgument('table_tracking', default_value='false'),
        Node(
            package='jdamr_cube_navigation',
            executable='depth_box_parking',
            name='jdamr_depth_box_parking',
            output='screen',
            parameters=[parameters, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'table_components': ParameterValue(
                    LaunchConfiguration('table_components'), value_type=bool),
                'table_tracking': ParameterValue(
                    LaunchConfiguration('table_tracking'), value_type=bool)}],
        ),
    ])
