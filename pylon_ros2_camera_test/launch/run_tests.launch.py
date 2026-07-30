#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (C) 2024, Basler AG. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * No contributors' name may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ─────────────────────────────────────────────────────────────────────────────
# run_tests.launch.py
#
# Single-command integration test entry point for the pylon_ros2_camera driver.
#
# Launches:
#   1. pylon_ros2_camera_wrapper   – the driver (same arguments as the standard
#                                    pylon_ros2_camera.launch.py)
#   2. camera_test_2d              – 2D test node (skipped when camera_type:=3d)
#   3. camera_test_3d              – 3D test node (skipped when camera_type:=2d)
#
# With camera_type:=auto (default) both test nodes start.  Each one waits up
# to camera_detection_timeout seconds for its action server:
#   • grab_images_raw   → 2D camera present   → runs 2D tests
#   • grab_blaze_data   → 3D camera present   → runs 3D tests
# The node that does not find its camera prints "not detected, skipping" and
# exits cleanly – no failure, safe for CI without hardware.
#
# Usage examples:
#
#   # Auto-detect camera type (default)
#   ros2 launch pylon_ros2_camera_test run_tests.launch.py
#
#   # Force 2D only
#   ros2 launch pylon_ros2_camera_test run_tests.launch.py camera_type:=2d
#
#   # Force 3D only
#   ros2 launch pylon_ros2_camera_test run_tests.launch.py camera_type:=3d
#
#   # Custom camera namespace / config
#   ros2 launch pylon_ros2_camera_test run_tests.launch.py \
#       camera_id:=my_camera  node_name:=pylon_ros2_camera_node \
#       config_file:=/path/to/my_config.yaml
#
#   # Run tests against an already-running driver (do not start a new one)
#   ros2 launch pylon_ros2_camera_test run_tests.launch.py include_driver:=false
#
# ─────────────────────────────────────────────────────────────────────────────

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Helpers ───────────────────────────────────────────────────────────────────

def _resolve_camera_id(context: LaunchContext) -> str:
    """Return the effective ROS namespace for the camera.

    If the user did not explicitly set camera_id, use device_user_id as the
    namespace (they are the same string in the common case).  If device_user_id
    is also empty (no specific camera requested), fall back to 'my_camera'.
    """
    camera_id      = LaunchConfiguration('camera_id').perform(context)
    device_user_id = LaunchConfiguration('device_user_id').perform(context)

    if camera_id == '__auto__':
        return device_user_id if device_user_id else 'my_camera'
    return camera_id


# ── Driver node (identical to pylon_ros2_camera.launch.py) ───────────────────

def _launch_driver(context: LaunchContext):
    camera_id      = _resolve_camera_id(context)
    node_name      = LaunchConfiguration('node_name')
    device_user_id = LaunchConfiguration('device_user_id')
    config_file      = LaunchConfiguration('config_file')
    mtu_size         = LaunchConfiguration('mtu_size')
    startup_user_set              = LaunchConfiguration('startup_user_set')
    enable_status_publisher       = LaunchConfiguration('enable_status_publisher')
    enable_current_params_pub     = LaunchConfiguration('enable_current_params_publisher')

    config_file               = LaunchConfiguration('config_file')
    mtu_size                  = LaunchConfiguration('mtu_size')
    startup_user_set          = LaunchConfiguration('startup_user_set')
    enable_status_publisher   = LaunchConfiguration('enable_status_publisher')
    enable_current_params_pub = LaunchConfiguration('enable_current_params_publisher')

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    return [
        Node(
            package='pylon_ros2_camera_wrapper',
            namespace=camera_id,
            executable='pylon_ros2_camera_wrapper',
            name=node_name,
            output='screen',
            emulate_tty=True,
            parameters=[
                config_file,
                {
                    'device_user_id': device_user_id,
                    'gige/mtu_size': mtu_size,
                    'startup_user_set': startup_user_set,
                    'enable_status_publisher': enable_status_publisher,
                    'enable_current_params_publisher': enable_current_params_pub,
                },
            ],
        ),
    ]


# ── Shared exit-tracking state ────────────────────────────────────────────────
# A mutable dict is used so the on_exit closure can update it after the
# OpaqueFunction returns.  This is intentionally module-level so every
# closure shares the same object.
_test_state = {'finished': 0, 'expected': 0}


def _on_test_node_exit(event, context):
    """Called each time a test node process exits.

    If a node exits with a non-zero code it means a hard failure (e.g.
    device_user_id was set but the camera was not reachable).  Trigger an
    immediate shutdown so the remaining test nodes and the driver are stopped
    right away.

    When the last expected test node exits cleanly, also emit Shutdown.
    """
    if event.returncode != 0:
        return EmitEvent(event=ShutdownEvent(
            reason=f'Test node exited with error (return code {event.returncode})'))

    _test_state['finished'] += 1
    if _test_state['finished'] >= _test_state['expected']:
        return EmitEvent(event=ShutdownEvent(reason='All test nodes finished'))


def _launch_tests(context: LaunchContext):
    """Create the test node(s) and register their exit handlers."""
    camera_type       = LaunchConfiguration('camera_type').perform(context)
    camera_id         = _resolve_camera_id(context)
    node_name         = LaunchConfiguration('node_name').perform(context)
    detection_timeout = LaunchConfiguration('camera_detection_timeout').perform(context)
    device_user_id    = LaunchConfiguration('device_user_id').perform(context)

    run_2d = camera_type != '3d'
    run_3d = camera_type != '2d'

    # Reset counter for this launch (important if the launch file is re-used).
    _test_state['finished'] = 0
    _test_state['expected'] = (1 if run_2d else 0) + (1 if run_3d else 0)

    # fail_on_no_camera: when device_user_id is set the user asked for a
    # specific camera.  If neither node detects it (and it's not a type
    # mismatch), treat that as a fatal error and stop immediately.
    # Without device_user_id the first available camera is used and a missing
    # camera is a benign skip.
    fail_on_no_camera = bool(device_user_id)
    base_params = {
        'camera_id': camera_id,
        'camera_node_name': node_name,
        'camera_detection_timeout': int(detection_timeout),
        'device_user_id': device_user_id,
        'fail_on_no_camera': fail_on_no_camera,
    }

    actions = []

    if run_2d:
        test_2d = Node(
            package='pylon_ros2_camera_test',
            executable='camera_test_2d',
            name='camera_test_2d',
            output='screen',
            emulate_tty=True,
            parameters=[base_params],
        )
        actions.append(test_2d)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=test_2d, on_exit=_on_test_node_exit)))

    if run_3d:
        test_3d = Node(
            package='pylon_ros2_camera_test',
            executable='camera_test_3d',
            name='camera_test_3d',
            output='screen',
            emulate_tty=True,
            parameters=[base_params],
        )
        actions.append(test_3d)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=test_3d, on_exit=_on_test_node_exit)))

    return actions


# ── Launch description ────────────────────────────────────────────────────────

def generate_launch_description():

    default_config_file = os.path.join(
        get_package_share_directory('pylon_ros2_camera_wrapper'),
        'config',
        'default.yaml',
    )

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # ── Arguments ────────────────────────────────────────────────────────────

    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='pylon_ros2_camera_node',
        description='Name of the camera driver node.',
    )
    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='__auto__',
        description=(
            'ROS namespace for the camera node. '
            'Defaults to device_user_id when that is set, otherwise "my_camera".'
        ),
    )
    declare_device_user_id = DeclareLaunchArgument(
        'device_user_id',
        default_value='',
        description=(
            'Device User ID of the camera to connect to. '
            'Empty string connects to the first available camera. '
            'Set this when multiple cameras are connected to select the correct one.'
        ),
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the driver YAML config file.',
    )
    declare_mtu_size = DeclareLaunchArgument(
        'mtu_size',
        default_value='1500',
        description='GigE MTU size (use 8192 for jumbo frames).',
    )
    declare_startup_user_set = DeclareLaunchArgument(
        'startup_user_set',
        default_value='CurrentSetting',
        description='Camera user set on startup: Default, UserSet1-3, CurrentSetting.',
    )
    declare_enable_status_pub = DeclareLaunchArgument(
        'enable_status_publisher',
        default_value='true',
        description='Enable the component_status publisher.',
    )
    declare_enable_params_pub = DeclareLaunchArgument(
        'enable_current_params_publisher',
        default_value='true',
        description='Enable the current_params publisher.',
    )
    declare_camera_type = DeclareLaunchArgument(
        'camera_type',
        default_value='auto',
        description=(
            'Which test node(s) to start. '
            '"auto" starts both and each self-detects. '
            '"2d" starts only the 2D test node. '
            '"3d" starts only the 3D test node.'
        ),
    )
    declare_detection_timeout = DeclareLaunchArgument(
        'camera_detection_timeout',
        default_value='15',
        description='Seconds to wait for the camera to connect before skipping '
                    '(a blaze can take several seconds through its GenTL producer).',
    )

    # ── Common parameters forwarded to every test node ────────────────────────

    def test_params(context: LaunchContext):
        return [
            {
                'camera_id': LaunchConfiguration('camera_id').perform(context),
                'camera_node_name': LaunchConfiguration('node_name').perform(context),
                'camera_detection_timeout': int(
                    LaunchConfiguration('camera_detection_timeout').perform(context)
                ),
                'device_user_id': LaunchConfiguration('device_user_id').perform(context),
            }
        ]

    # ── Assemble ──────────────────────────────────────────────────────────────

    ld = LaunchDescription()

    ld.add_action(declare_node_name)
    ld.add_action(declare_camera_id)
    ld.add_action(declare_device_user_id)
    ld.add_action(declare_config_file)
    ld.add_action(declare_mtu_size)
    ld.add_action(declare_startup_user_set)
    ld.add_action(declare_enable_status_pub)
    ld.add_action(declare_enable_params_pub)
    ld.add_action(declare_camera_type)
    ld.add_action(declare_detection_timeout)

    ld.add_action(OpaqueFunction(function=_launch_driver))
    ld.add_action(OpaqueFunction(function=_launch_tests))

    return ld
