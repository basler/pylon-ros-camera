#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # adapt if needed
    respawn = False

    # launch configuration variables
    node_name = LaunchConfiguration('node_name')

    # launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ip_auto_config',
        description='Name of node.'
    )

    # node
    ip_auto_config_node = Node(
        package='pylon_ros2_camera_component',
        executable='ip_auto_config',
        name=node_name,
        output='screen',
        emulate_tty=True,
        respawn=respawn,
        prefix=['xterm -e gdb -ex run --args']
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(declare_node_name_cmd)

    ld.add_action(ip_auto_config_node)

    return ld
