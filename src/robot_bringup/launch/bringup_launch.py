"""Bring up the full mapping stack: robot_state_publisher (URDF), RPLidar
A1M8, ESP32 serial bridge, and slam_toolbox (online async).

Serial ports for the lidar and the ESP32 are both /dev/ttyUSB* on the Jetson
and their enumeration order is not guaranteed, so the defaults below point at
the udev symlinks installed by udev/99-argo-mini.rules (/dev/esp32,
/dev/rplidar) instead of raw ttyUSB0/ttyUSB1. See that file for setup.

serial_bridge_node fuses wheel encoders (+ gyro, once the firmware sends one)
itself and publishes odom->base_footprint directly, so there is no separate
ekf_node here - running one alongside would fight over who parents base_link.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    xacro_path = os.path.join(bringup_share, 'urdf', 'argo_mini.urdf.xacro')

    esp32_port = LaunchConfiguration('esp32_port', default='/dev/esp32')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/rplidar')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/esp32',
                               description='Serial device for the ESP32 (udev symlink, see udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar',
                               description='Serial device for the RPLidar (udev symlink, see udev/99-argo-mini.rules)'),

        # --- Robot description: base_footprint/base_link + laser/imu/ultrasonic
        # frames, all defined in urdf/argo_mini.urdf.xacro ---
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_path])}],
        ),

        # --- RPLidar A1M8 ---
        # rplidar_node only exists in the vendored 2.1.4 source; the
        # ros-jazzy-rplidar-ros apt package (rosdep-installed, see
        # package.xml) ships rplidar_composition instead.
        Node(
            package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
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

        # --- ESP32 serial bridge (RPM motor control + hall odom + ultrasonics) ---
        Node(
            package='robot_bringup', executable='serial_bridge_node', name='serial_bridge',
            parameters=[{
                'port': esp32_port,
                'baud': 115200,
            }],
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
