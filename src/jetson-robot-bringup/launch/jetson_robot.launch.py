import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    jetson_port = LaunchConfiguration('jetson_port', default='/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    robot_localization_file_path = get_package_share_directory('jetson-robot-bringup') 

    remappings = [('/odometry/filtered','/odometry/filtered'),
                  ('/odom', '/odom')]
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'jetson_port',
            default_value = jetson_port,
            description = 'Serial port for communication w microcontroller'
        ),
        DeclareLaunchArgument(
            'rplidar_port',
            default_value = rplidar_port,
            description = 'Serial port for communication w RPLiDAR sensor'
        ),
        # # lidar launchfile
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rp_lidar.launch.py']
            ),
            launch_arguments={'lidar_port': rplidar_port}.items()
        ),
        # robot control node
        Node(
            package='jetson-robot-bringup',
            executable='jetson_robot_control.py',
            parameters=[{
                'jetson_port': jetson_port
            }],
            arguments=[],
            output='screen'
        ),
        # led control node
        Node(
            package='jetson-robot-bringup',
            executable='test_led.py',
            parameters=[],
            arguments=[],
            output='screen'
        ),
        # static transform for laser scan
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.06', '0', '0.18', '0', '0', '0', 'base_link', 'laser'],
            output = 'screen'
        ),
        # static transform from link to footprint
        Node(   
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0', '0', '-0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        # Node(
        #     package='jetson-robot-bringup',
        #     executable='mpu9250.py',
        #     name = 'mpu9250',
        #     output='screen',
        #     parameters=[],
        # ),
        # Node(
        #     package = 'robot_localization',
        #     executable = 'ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(robot_localization_file_path, 'config', 'ekf.yaml'), 
        #     {'use_sim_time': False}],
        #     remappings=remappings,
        # )
    ])