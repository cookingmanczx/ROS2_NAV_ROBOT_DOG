"""Start Nav2 (map_server + AMCL + planner/controller) on the running sim.

Prerequisites:
  * ``ros2 launch a2_gazebo_sim sim.launch.py world:=small_house`` is running
  * slam_toolbox is NOT running (AMCL owns map->odom while navigating)
  * a map was saved with slam_toolbox (see maps/room1.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_share = get_package_share_directory('a2_gazebo_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # bt_navigator is hidden behind goal_guard, which validates goals first.
    # rclcpp action servers do not apply a remap rule placed on the action
    # name itself, so every endpoint the action expands to is remapped too.
    impl = '/navigate_to_pose_impl/_action'
    bringup = GroupAction([
        SetRemap(src='/navigate_to_pose', dst='/navigate_to_pose_impl'),
        SetRemap(src='/navigate_to_pose/_action/send_goal',
                 dst=impl + '/send_goal'),
        SetRemap(src='/navigate_to_pose/_action/cancel_goal',
                 dst=impl + '/cancel_goal'),
        SetRemap(src='/navigate_to_pose/_action/get_result',
                 dst=impl + '/get_result'),
        SetRemap(src='/navigate_to_pose/_action/feedback',
                 dst=impl + '/feedback'),
        SetRemap(src='/navigate_to_pose/_action/status',
                 dst=impl + '/status'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'map': LaunchConfiguration('map'),
                'slam': 'False',
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])

    goal_guard = Node(
        package='a2_gazebo_sim',
        executable='goal_guard.py',
        name='goal_guard',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # Seed AMCL with the pose the robot actually has now. Without this the
    # filter starts at the hard-coded origin and the map looks shifted.
    initial_pose = Node(
        package='a2_gazebo_sim',
        executable='initial_pose_from_odom.py',
        name='initial_pose_from_odom',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame': 'map',
            'odom_topic': '/odom',
        }],
        condition=IfCondition(LaunchConfiguration('seed_initial_pose')),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'room1.yaml'),
            description='Map yaml to load with map_server'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate Nav2 nodes'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Set to "false" to run without RViz2'),
        DeclareLaunchArgument(
            'seed_initial_pose', default_value='true',
            description='Seed AMCL with the current odom pose on start-up'),
        bringup,
        goal_guard,
        initial_pose,
        rviz,
    ])
