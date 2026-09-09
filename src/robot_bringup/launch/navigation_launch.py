"""Bring up localization + navigation on a saved map: robot hardware (see
hardware_launch.py) plus nav2 (AMCL + planner/controller/BT stack via
nav2_bringup's bringup_launch.py).

Requires a saved map - see maps/README.md for how to save one with
map_saver_cli. Defaults to maps/office_map.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    esp32_port = LaunchConfiguration('esp32_port')
    lidar_port = LaunchConfiguration('lidar_port')
    left_tick_scale = LaunchConfiguration('left_tick_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    disable_tank_turns = LaunchConfiguration('disable_tank_turns')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    default_map = os.path.join(bringup_share, 'maps', 'office_map.yaml')
    default_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0',
                               description='Serial device for the ESP32 (or the udev symlink from udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                               description='Serial device for the RPLidar (or the udev symlink from udev/99-argo-mini.rules)'),
        DeclareLaunchArgument('left_tick_scale', default_value='0.66',
                               description='Left wheel odometry tick correction factor - calibrate by driving straight and checking /odom drift'),
        DeclareLaunchArgument('angular_scale', default_value='0.2',
                               description='Multiplier applied to /cmd_vel angular.z - lower this if the robot turns more than commanded'),
        DeclareLaunchArgument('disable_tank_turns', default_value='true',
                               description='If true, never spin both wheels in opposite directions - degrades the nav2 "spin" recovery to an off-center pivot, see config/nav2_params.yaml'),
        DeclareLaunchArgument('map', default_value=default_map,
                               description='Full path to the map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                               description='Full path to the nav2 params file to use'),
        DeclareLaunchArgument('autostart', default_value='true',
                               description='Automatically bring up the nav2 lifecycle nodes'),

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

        # --- nav2: map_server + AMCL + planner/controller/BT stack ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': 'False',
                'autostart': autostart,
                'slam': 'False',
            }.items(),
        ),
    ])
