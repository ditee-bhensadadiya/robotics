"""Bring up the full mapping stack: robot_state_publisher (URDF), RPLidar
A1M8, ESP32 serial bridge, and slam_toolbox (online async).

Serial ports for the lidar and the ESP32 are both /dev/ttyUSB* on the Jetson
by default and their enumeration order is not guaranteed - set up udev rules
to create stable symlinks (e.g. /dev/rplidar, /dev/esp32) and pass those via
the launch arguments below instead of relying on ttyUSB0/ttyUSB1.

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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    xacro_path = os.path.join(bringup_share, 'urdf', 'argo_mini.urdf.xacro')

    esp32_port = LaunchConfiguration('esp32_port', default='/dev/ttyUSB1')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB1',
                               description='Serial device for the ESP32 (udev symlink recommended)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                               description='Serial device for the RPLidar (udev symlink recommended)'),

        # --- Robot description: base_footprint/base_link + laser/imu/ultrasonic
        # frames, all defined in urdf/argo_mini.urdf.xacro ---
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}],
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
