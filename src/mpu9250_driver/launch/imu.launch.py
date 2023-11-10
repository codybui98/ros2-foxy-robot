import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu9250_driver',
            executable='mpu9250.py',
            output='screen',
            parameters=[],
        ),
         Node(
            package='imu_filter_madgwick',
            executable='imu_filter_node',
            output='screen',
            parameters=[],
        ),
    ])