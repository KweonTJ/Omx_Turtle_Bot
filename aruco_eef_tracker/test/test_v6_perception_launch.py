"""Non-actuating launch contract: ID isolation and no legacy control path."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('launch')
from launch import LaunchContext


@pytest.fixture
def setup_launch(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('v6_launch', root/'launch/eef_aruco_v6_dual.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(root))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'start_camera': 'false', 'image_topic': '/omx_v6/eef_camera/image_raw',
        'camera_info_topic': '/omx_v6/eef_camera/camera_info', 'camera_info_url': '',
        'video_device': '/dev/test-camera',
    })
    return module, context


def test_independent_ids_no_legacy_topics(setup_launch):
    module, context = setup_launch
    nodes = module.nodes(context)
    assert len(nodes) == 2
    for marker, node in enumerate(nodes):
        assert node['package'] == 'aruco_eef_tracker'
        assert node['executable'] == 'aruco_eef_tracker_node'
        params = node['parameters'][0]
        assert params['allowed_marker_ids'] == [marker]
        assert params['marker_id'] == marker
        assert params['dictionary'] == 'DICT_4X4_50'
        assert params['marker_size_m'] == .04
        assert params['max_reprojection_error_px'] == 1.5
        assert all(v.startswith('/omx_v6/') for k, v in params.items() if k.endswith('_topic'))


def test_camera_requires_explicit_matching_calibration(setup_launch):
    module, context = setup_launch
    context.launch_configurations['start_camera'] = 'true'
    with pytest.raises(ValueError, match='calibrated camera_info_url'):
        module.nodes(context)


def test_camera_is_opt_in(setup_launch):
    module, _ = setup_launch
    args = module.generate_launch_description().entities
    camera = next(arg for arg in args if getattr(arg, 'name', '') == 'start_camera')
    assert ''.join(s.text for s in camera.default_value) == 'false'
