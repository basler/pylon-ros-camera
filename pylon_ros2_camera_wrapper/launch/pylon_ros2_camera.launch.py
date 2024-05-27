#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.

    This is required to evaluate `respawn` as boolean.

    """

    # adapt if needed
    debug = False

    node_name = LaunchConfiguration('node_name')
    camera_id = LaunchConfiguration('camera_id')
    config_file = LaunchConfiguration('config_file')

    mtu_size = LaunchConfiguration('mtu_size')
    startup_user_set = LaunchConfiguration('startup_user_set')
    enable_status_publisher = LaunchConfiguration('enable_status_publisher')
    enable_current_params_publisher = LaunchConfiguration('enable_current_params_publisher')

    respawn = LaunchConfiguration('respawn')
    respawn_str = respawn.perform(context)
    respawn_bool = respawn_str.lower() == 'true'

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
                    config_file,
                    {
                        'gige/mtu_size': mtu_size,
                        'startup_user_set': startup_user_set,
                        'enable_status_publisher': enable_status_publisher,
                        'enable_current_params_publisher': enable_current_params_publisher
                    }
                ]
            ),
        ]


def generate_launch_description():

    default_config_file = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config',
        'default.yaml'
    )

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
        default_value=default_config_file,
        description='Camera parameters structured in a .yaml file.'
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
        default_value='true',
        description='Enable/Disable the status publishing.'
    )

    declare_enable_current_params_publisher_cmd = DeclareLaunchArgument(
        'enable_current_params_publisher',
        default_value='true',
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
    ld.add_action(declare_mtu_size_cmd)
    ld.add_action(declare_startup_user_set_cmd)
    ld.add_action(declare_enable_status_publisher_cmd)
    ld.add_action(declare_enable_current_params_publisher_cmd)

    ld.add_action(declare_respawn_cmd)

    ld.add_action(OpaqueFunction(function=_launch_node))

    return ld
