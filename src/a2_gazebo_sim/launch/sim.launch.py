"""Launch Gazebo Classic with the Unitree A2 + 2D lidar."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# With the stand pose from config/joint_defaults.yaml (thigh +0.9 rad,
# calf -1.8 rad) the foot is 2 * 0.275 * cos(0.9) = 0.342 m below base_link,
# and the foot sphere has radius 0.032 m, so base_link is 0.374 m above ground.
SPAWN_Z = '0.374'


def _resolve_world(world_name):
    """Resolve a world argument to an existing absolute path.

    Priority:
      1. existing path (absolute or relative to the current directory)
      2. this package's share/worlds/<name>.world
      3. /usr/share/gazebo-11/worlds/<name>.world
    """
    expanded = os.path.expanduser(world_name)
    if os.path.isfile(expanded):
        return expanded

    file_names = [world_name]
    if not world_name.endswith('.world'):
        file_names.append(world_name + '.world')

    search_dirs = [
        os.path.join(get_package_share_directory('a2_gazebo_sim'), 'worlds'),
        '/usr/share/gazebo-11/worlds',
    ]
    for directory in search_dirs:
        for name in file_names:
            candidate = os.path.join(directory, name)
            if os.path.isfile(candidate):
                return candidate

    raise RuntimeError(
        "Cannot find world '%s'. Searched: %s" % (world_name, search_dirs))


def _robot_description():
    """Expand the xacro and rewrite package:// mesh URIs to file://.

    Gazebo Classic's client cannot resolve ``model://`` URIs for models that are
    spawned at runtime (only for models declared inside a .world file), so the
    A2 meshes would silently fail to render. Rewriting the URIs at launch time
    keeps the official a2_description URDF on disk untouched.
    """
    xacro_file = os.path.join(
        get_package_share_directory('a2_gazebo_sim'), 'urdf', 'a2_lidar.urdf.xacro')
    urdf = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
    a2_share = get_package_share_directory('a2_description')
    return urdf.replace('package://a2_description/', 'file://' + a2_share + '/')


def _launch_setup(context, *args, **kwargs):
    world_path = _resolve_world(LaunchConfiguration('world').perform(context))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_path,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    robot_description = _robot_description()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('a2_gazebo_sim'),
                         'config', 'joint_defaults.yaml'),
            {'use_sim_time': True},
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_a2',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'a2',
            '-timeout', '120',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', SPAWN_Z,
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # The model is static: its pose is integrated from /cmd_vel and applied
    # through gazebo_ros SetEntityState. Initial pose must match spawn pose.
    cmd_vel_driver = Node(
        package='a2_gazebo_sim',
        executable='cmd_vel_driver.py',
        name='cmd_vel_driver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'entity_name': 'a2',
            'spawn_z': float(SPAWN_Z),
            'initial_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'initial_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'initial_yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
        }],
    )

    return [gazebo, robot_state_publisher, joint_state_publisher,
            spawn_entity, cmd_vel_driver]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small_house',
            description='World path, or a world name found in this package or /usr/share/gazebo-11/worlds'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='Spawn x position of the robot'),
        DeclareLaunchArgument(
            'y', default_value='0.0',
            description='Spawn y position of the robot'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Spawn yaw of the robot'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Set to "false" to run headless (no gzclient)'),
        OpaqueFunction(function=_launch_setup),
    ])
