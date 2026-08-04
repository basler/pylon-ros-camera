#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_profile_config(config_file, profile):
    """Resolve the camera configuration file for a given profile.

    An explicit, non-empty ``config_file`` always takes precedence. Otherwise
    the file is selected from the profile: ``3d`` (case-insensitive) uses
    ``profile_3d.yaml`` and any other value (including ``2d`` and unknown
    profiles) falls back to ``default.yaml``.
    """
    if config_file:
        return config_file

    config_dir = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config'
    )
    if (profile or '').strip().lower() == '3d':
        return os.path.join(config_dir, 'profile_3d.yaml')
    return os.path.join(config_dir, 'default.yaml')


def _launch_node(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.
    This is required to evaluate `respawn` as boolean.
    """
    
    # adapt if needed
    debug = False

    # launch configuration variables
    node_name = LaunchConfiguration('node_name')
    camera_id = LaunchConfiguration('camera_id')

    config_file = LaunchConfiguration('config_file')
    profile = LaunchConfiguration('profile')
    resolved_config_file = resolve_profile_config(
        config_file.perform(context), profile.perform(context))

    mtu_size = LaunchConfiguration('mtu_size')
    startup_user_set = LaunchConfiguration('startup_user_set')
    enable_status_publisher = LaunchConfiguration('enable_status_publisher')
    enable_current_params_publisher = LaunchConfiguration('enable_current_params_publisher')

    respawn = LaunchConfiguration('respawn')
    respawn_str = respawn.perform(context)
    respawn_bool = respawn_str.lower() == 'true'

    # log format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    if debug:
        launch_prefix = ['xterm -e gdb -ex run --args']
    else:
        launch_prefix = ''

    return [
            Node(
                package='pylon_ros2_camera_wrapper',
                namespace=camera_id,
                executable='pylon_ros2_camera_wrapper',
                name=node_name,
                output='screen',
                respawn=respawn_bool,
                emulate_tty=True,
                prefix=launch_prefix,
                parameters=[
                    resolved_config_file,
                    {
                        'mtu_size': mtu_size,
                        'startup_user_set': startup_user_set,
                        'enable_status_publisher': enable_status_publisher,
                        'enable_current_params_publisher': enable_current_params_publisher
                    }
                ]
            ),
        ]

def generate_launch_description():

    # launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='pylon_ros2_camera_node',
        description='Name of the wrapper node.'
    )

    declare_camera_id_cmd = DeclareLaunchArgument(
        'camera_id',
        default_value='my_camera',
        description='Id of the camera. Used as node namespace.'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Camera parameters structured in a .yaml file. If left '
                    'empty, the file is selected from the "profile" argument.'
    )

    declare_profile_cmd = DeclareLaunchArgument(
        'profile',
        default_value='2d',
        description='Camera profile used to pick a default config file when '
                    '"config_file" is empty: "2d" -> default.yaml, '
                    '"3d" -> profile_3d.yaml.'
    )

    declare_mtu_size_cmd = DeclareLaunchArgument(
        'mtu_size',
        default_value='1500',
        description='Maximum transfer unit size. To enable jumbo frames, set it to a high value (8192 recommended)'
    )

    declare_startup_user_set_cmd = DeclareLaunchArgument(
        'startup_user_set',
        # possible value: Default, UserSet1, UserSet2, UserSet3, CurrentSetting
        default_value='CurrentSetting',
        description='Specific user set defining user parameters to run the camera.'
    )

    declare_enable_status_publisher_cmd = DeclareLaunchArgument(
        'enable_status_publisher',
        default_value='false',
        description='Enable/Disable the status publishing.'
    )

    declare_enable_current_params_publisher_cmd = DeclareLaunchArgument(
        'enable_current_params_publisher',
        default_value='false',
        description='Enable/Disable the current parameter publishing.'
    )

    declare_respawn_cmd = DeclareLaunchArgument(
        'respawn',
        default_value='false',
        description='If true, the node will be respawned if it exits.'
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_camera_id_cmd)

    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_profile_cmd)
    ld.add_action(declare_mtu_size_cmd)
    ld.add_action(declare_startup_user_set_cmd)
    ld.add_action(declare_enable_status_publisher_cmd)
    ld.add_action(declare_enable_current_params_publisher_cmd)

    ld.add_action(declare_respawn_cmd)

    ld.add_action(OpaqueFunction(function=_launch_node))

    return ld
