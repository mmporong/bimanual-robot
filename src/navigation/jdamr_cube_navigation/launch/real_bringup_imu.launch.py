"""Run the real base with wheel velocity and bias-corrected gyro odometry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Give the EKF sole ownership of /odom and odom→base_footprint."""
    bringup = Path(get_package_share_directory('jdamr_cube_bringup'))
    navigation = Path(get_package_share_directory('jdamr_cube_navigation'))
    defaults = {
        'base_port': '/dev/ttyS0',
        'lidar_port': '/dev/ydlidar_g4',
        'wheel_radius': '0.0329',
        'wheel_separation': '0.510',
        'wheel_radius_ratio': '1.0',
        'discovery_range': 'LOCALHOST',
        'scan_self_filter_config': '',
    }
    base_arguments = {name: LaunchConfiguration(name) for name in defaults}
    base_arguments.update(publish_odom_tf='false', wheel_odom_topic='/wheel/odom')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/real_bringup.launch.py')),
            launch_arguments=base_arguments.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation / 'launch/ekf_odom_imu.launch.py')),
            launch_arguments={
                'wheel_odom_topic': '/wheel/odom',
                'filtered_odom_topic': '/odom',
            }.items()),
    ])
