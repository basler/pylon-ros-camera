import importlib.util
from pathlib import Path


def _load_launch_module():
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'pylon_ros2_camera.launch.py'
    spec = importlib.util.spec_from_file_location('pylon_ros2_camera_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_resolve_profile_config_explicit_override():
    launch_module = _load_launch_module()
    custom_cfg = '/tmp/custom_camera.yaml'
    assert launch_module.resolve_profile_config(custom_cfg, '3d') == custom_cfg


def test_resolve_profile_config_2d_default():
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '2d')
    assert resolved.endswith('/config/default.yaml')


def test_resolve_profile_config_3d_default():
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '3d')
    assert resolved.endswith('/config/profile_3d.yaml')


def test_resolve_profile_config_case_insensitive():
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', '3D')
    assert resolved.endswith('/config/profile_3d.yaml')


def test_resolve_profile_config_unknown_profile_falls_back_to_2d():
    launch_module = _load_launch_module()
    resolved = launch_module.resolve_profile_config('', 'future-profile')
    assert resolved.endswith('/config/default.yaml')
