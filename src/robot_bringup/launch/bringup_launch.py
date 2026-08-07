"""Bring up the full mapping stack: RPLidar A1M8, ESP32 bridge, EKF, static
TF frames, and slam_toolbox (online async).

Serial ports for the lidar and the ESP32 are both /dev/ttyUSB* on the Jetson
by default and their enumeration order is not guaranteed - set up udev rules
to create stable symlinks (e.g. /dev/rplidar, /dev/esp32) and pass those via
the launch arguments below instead of relying on ttyUSB0/ttyUSB1.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    esp32_port = LaunchConfiguration('esp32_port', default='/dev/esp32')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/rplidar')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/esp32',
                               description='Serial device for the ESP32 (udev symlink recommended)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar',
                               description='Serial device for the RPLidar (udev symlink recommended)'),

        # --- Static TF: sensor frames relative to base_link ---
        # Placeholders (0,0,0) - measure and update the actual mounting offsets.
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # --- RPLidar A1M8 ---
        Node(
            package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),

        # --- ESP32 bridge (motors + hall odom + IMU) ---
        Node(
            package='robot_bringup', executable='esp32_bridge_node', name='esp32_bridge',
            parameters=[
                os.path.join(bringup_share, 'config', 'robot_bringup.yaml'),
                {'serial_port': esp32_port},
            ],
            output='screen',
        ),

        # --- EKF: fuse wheel odom + IMU yaw rate -> odom->base_link tf ---
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml')],
            output='screen',
        ),

        # --- slam_toolbox: online async mapping ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch', 'online_async_launch.py',
                )
            ),
            launch_arguments={
                'slam_params_file': os.path.join(
                    bringup_share, 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
