"""Bring up the full mapping stack: robot hardware (see hardware_launch.py)
plus slam_toolbox (online async).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    left_tick_scale = LaunchConfiguration('left_tick_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    disable_tank_turns = LaunchConfiguration('disable_tank_turns')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/esp32',
                               description='Serial device for the ESP32 (udev symlink, see udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar',
                               description='Serial device for the RPLidar (udev symlink, see udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('left_tick_scale', default_value='0.66',
                               description='Left wheel odometry tick correction factor - calibrate by driving straight and checking /odom drift'),
        DeclareLaunchArgument('angular_scale', default_value='0.2',
                               description='Multiplier applied to /cmd_vel angular.z - lower this if the robot turns more than commanded'),
        DeclareLaunchArgument('disable_tank_turns', default_value='true',
                               description='If true, never spin both wheels in opposite directions - stop the slower wheel and pivot with the other instead'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'hardware_launch.py')
            ),
            launch_arguments={
                'esp32_port': esp32_port,
                'lidar_port': lidar_port,
                'left_tick_scale': left_tick_scale,
                'angular_scale': angular_scale,
                'disable_tank_turns': disable_tank_turns,
            }.items(),
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
