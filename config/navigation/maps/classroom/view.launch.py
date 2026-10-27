"""Show the saved classroom map and destinations without a motion interface."""

import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    asset_dir = Path(__file__).resolve().parent
    repo_root = asset_dir.parents[3]
    package_root = repo_root / 'src/navigation/jdamr_cube_navigation'
    marker_publisher = package_root / 'jdamr_cube_navigation/service_visualization.py'
    pythonpath = os.pathsep.join(
        filter(None, (str(package_root), os.environ.get('PYTHONPATH', ''))))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='nav2_map_server', executable='map_server',
            name='handoff_map_server', output='screen',
            parameters=[{'yaml_filename': str(asset_dir / 'map.yaml'),
                         'topic_name': 'map', 'frame_id': 'map'}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='handoff_map_lifecycle', output='screen',
            parameters=[{'autostart': True, 'node_names': ['handoff_map_server']}]),
        ExecuteProcess(
            cmd=[sys.executable, str(marker_publisher),
                 '--registry', str(asset_dir / 'locations.yaml'),
                 '--annotation', str(asset_dir / 'annotation.json')],
            additional_env={'PYTHONPATH': pythonpath}, output='screen'),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(asset_dir / 'offline_view.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
