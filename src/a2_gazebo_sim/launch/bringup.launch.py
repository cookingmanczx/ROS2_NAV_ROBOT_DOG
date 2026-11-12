"""One-shot bringup: Gazebo sim + Nav2 + RViz.

Nav2 is started on a timer, not in the same instant as Gazebo, because
map_server/AMCL/costmaps need ``map->odom->base_link`` from
robot_state_publisher + cmd_vel_driver before they can activate.

Usage:
  ros2 launch a2_gazebo_sim bringup.launch.py
  ros2 launch a2_gazebo_sim bringup.launch.py world:=small_house map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('a2_gazebo_sim')
    launch_dir = os.path.join(pkg_share, 'launch')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    nav = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav.launch.py')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'true',
                    'rviz': LaunchConfiguration('rviz'),
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small_house',
            description='World path, or a world name found in this package or /usr/share/gazebo-11/worlds'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Set to "false" to run Gazebo without gzclient'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'room1.yaml'),
            description='Map yaml to load with map_server'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Set to "false" to skip RViz'),
        DeclareLaunchArgument(
            'nav_delay', default_value='12.0',
            description='Seconds to wait for Gazebo/TF before starting Nav2'),
        sim,
        nav,
    ])
