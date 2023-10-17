#!/usr/bin/env python3

import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # adapt if needed
    frame_id = 'pylon_camera'
    publish_tf2 = True
    debug = False
    respawn = False

    default_config_file = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config',
        'my_blaze.yaml'
    )

    # launch configuration variables
    node_name = LaunchConfiguration('node_name')
    camera_id = LaunchConfiguration('camera_id')

    config_file = LaunchConfiguration('config_file')

    enable_status_publisher = LaunchConfiguration('enable_status_publisher')
    enable_current_params_publisher = LaunchConfiguration('enable_current_params_publisher')

    # launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='pylon_ros2_camera_node',
        description='Name of the wrapper node.'
    )

    declare_camera_id_cmd = DeclareLaunchArgument(
        'camera_id',
        default_value='my_blaze',
        description='Id of the camera. Used as node namespace.'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Camera parameters structured in a .yaml file.'
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

    # log format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    if (debug == True):
        launch_prefix = ['xterm -e gdb -ex run --args']
    else:
        launch_prefix = ''

    # node
    if (publish_tf2 == True):
        static_transform_node_args = ['0', '0', '0', '0', str(math.pi), str(math.pi / 2), 'map', frame_id]
    else:
        static_transform_node_args = ['0', '0', '0', '0', '0', '0', 'map', frame_id]

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = static_transform_node_args
    )

    pylon_ros2_camera_node = Node(
        package='pylon_ros2_camera_wrapper',        
        namespace=camera_id,
        executable='pylon_ros2_camera_wrapper',
        name=node_name,
        output='screen',
        respawn=respawn,
        emulate_tty=True,
        prefix=launch_prefix,
        parameters=[
            config_file,
            {
                'enable_status_publisher': enable_status_publisher,
                'enable_current_params_publisher': enable_current_params_publisher
            }
        ]
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_camera_id_cmd)

    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_enable_status_publisher_cmd)
    ld.add_action(declare_enable_current_params_publisher_cmd)

    ld.add_action(static_transform_node)
    ld.add_action(pylon_ros2_camera_node)

    return ld
