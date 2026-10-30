"""
Odom + gyro EKF with explicit input and output topics.

Run only with the base driver started with publish_tf:=false, so that odom->base_footprint
comes from the EKF alone. base_link->imu_link comes from the URDF (imu_joint: z down,
board y forward, board x left); only the yaw rate is fused.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('jdamr_cube_navigation'),
                          'config', 'ekf_odom_imu.yaml')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('filtered_odom_topic', default_value='/odometry/filtered'),
        Node(package='jdamr_cube_navigation', executable='imu_bias_relay',
             name='imu_bias_relay', output='screen', parameters=[sim],
             remappings=[('odom', LaunchConfiguration('wheel_odom_topic'))]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[config, sim,
                                        {'odom0': LaunchConfiguration('wheel_odom_topic')}],
             remappings=[('odometry/filtered', LaunchConfiguration('filtered_odom_topic'))]),
    ])
