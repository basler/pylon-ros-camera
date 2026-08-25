# Copyright (C) 2022, Basler AG. All rights reserved.
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
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

"""Unit tests for the launch-file profile configuration resolution."""

import importlib.util
from pathlib import Path


def _load_launch_module():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    launch_file = launch_dir / 'pylon_ros2_camera.launch.py'
    spec = importlib.util.spec_from_file_location(
        'pylon_ros2_camera_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_resolve_profile_config_explicit_override():
    """An explicit config file always takes precedence over the profile."""
    launch_module = _load_launch_module()
    custom_cfg = '/tmp/custom_camera.yaml'
    assert launch_module.resolve_profile_config(custom_cfg, '3d') == custom_cfg


def test_resolve_profile_config_2d_default():
    """The 2d profile resolves to the default configuration file."""
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '2d')
    assert resolved.endswith('/config/default_2d.yaml')


def test_resolve_profile_config_3d_default():
    """The 3d profile resolves to the 3D profile configuration file."""
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '3d')
    assert resolved.endswith('/config/default_3d.yaml')


def test_resolve_profile_config_case_insensitive():
    """Profile matching is case-insensitive."""
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '3D')
    assert resolved.endswith('/config/default_3d.yaml')


def test_resolve_profile_config_unknown_profile_falls_back_to_2d():
    """An unknown profile falls back to the default configuration file."""
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', 'future-profile')
    assert resolved.endswith('/config/default_2d.yaml')
