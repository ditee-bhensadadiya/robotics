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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    xacro_path = os.path.join(bringup_share, 'urdf', 'argo_mini.urdf.xacro')

    esp32_port = LaunchConfiguration('esp32_port', default='/dev/esp32')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/rplidar')
    left_tick_scale = LaunchConfiguration('left_tick_scale', default='1.135')
    angular_scale = LaunchConfiguration('angular_scale', default='0.5')
    disable_tank_turns = LaunchConfiguration('disable_tank_turns', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/esp32',
                               description='Serial device for the ESP32 (udev symlink, see udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar',
                               description='Serial device for the RPLidar (udev symlink, see udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('left_tick_scale', default_value='0.66',
                               description='Left wheel odometry tick correction factor - calibrate by driving straight and checking /odom drift'),
        DeclareLaunchArgument('angular_scale', default_value='0.2',
                               description='Multiplier applied to /cmd_vel angular.z - lower this if the robot turns more than commanded'),
        DeclareLaunchArgument('disable_tank_turns', default_value='true',
                               description='If true, never spin both wheels in opposite directions - stop the slower wheel and pivot with the other instead'),

        # --- Robot description: base_footprint/base_link + laser/imu/ultrasonic
        # frames, all defined in urdf/argo_mini.urdf.xacro ---
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}],
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
                'left_tick_scale': left_tick_scale,
                'angular_scale': angular_scale,
                'disable_tank_turns': disable_tank_turns,
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
