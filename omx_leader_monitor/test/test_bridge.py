import importlib.util
from pathlib import Path
import subprocess
import os

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bridge', ROOT / 'scripts/topic_bridge.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_routes_select_exactly_one_image():
    for mode in ('raw', 'compressed'):
        routes = bridge.load_routes(ROOT / 'config/bridge_topics.yaml', mode)
        assert len(routes) == 6
        images = [dest for _, dest, kind in routes if 'Image' in kind]
        assert len(images) == 1
        assert images[0].endswith('/compressed') == (mode == 'compressed')
        assert all(dest.startswith('/leader_monitor/input/') for _, dest, _ in routes)


@pytest.mark.parametrize('change', ['control_destination', 'duplicate', 'loop', 'type', 'missing'])
def test_reject_unsafe_or_invalid_route(tmp_path, change):
    route = {'source': '/joint_states', 'destination': '/leader_monitor/input/joint_states',
             'type': 'sensor_msgs/msg/JointState'}
    config = {'topics': [route]}
    if change == 'control_destination':
        route['destination'] = '/cmd_vel'
    elif change == 'duplicate':
        config['topics'].append(dict(route))
    elif change == 'loop':
        route['source'] = '/leader_monitor/input/joint_states'
    elif change == 'type':
        route['type'] = 'std_msgs/msg/String'
    else:
        del route['source']
    path = tmp_path / 'routes.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        bridge.load_routes(path, 'raw')


@pytest.mark.parametrize('arguments', [
    ['--demo', '--bridge-from', '25'], ['--domain', '999'], ['--domain', '-1'], ['--bogus']])
def test_shell_refuses_bad_arguments(arguments):
    result = subprocess.run(['bash', str(ROOT / 'scripts/run_leader_pick_place_monitor.sh'), *arguments],
                            capture_output=True, timeout=5)
    assert result.returncode == 2


def test_shell_valid_launch_has_no_empty_assignments(tmp_path):
    # Mock only the ros2 command after sourcing; never start DDS or a GUI here.
    workspace = tmp_path / 'fake workspace'
    (workspace / 'install').mkdir(parents=True)
    mock_bin = tmp_path / 'bin'
    mock_bin.mkdir()
    (workspace / 'install/setup.bash').write_text(f'export PATH="{mock_bin}:$PATH"\n')
    mock = mock_bin / 'ros2'
    mock.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    mock.chmod(0o755)
    env = dict(os.environ, DISPLAY=':99', ROS_LOG_DIR=str(tmp_path / 'logs'), ROS_DOMAIN_ID='16')
    for args in (['--demo'], ['--bridge-from', '25']):
        result = subprocess.run(['bash', str(ROOT / 'scripts/run_leader_pick_place_monitor.sh'),
            '--workspace', str(workspace), *args], env=env, capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
        assert 'leader_monitor.launch.py' in result.stdout
        assert not any(line.endswith(':=') for line in result.stdout.splitlines())
        assert ('source_domain:=25' in result.stdout) == ('--bridge-from' in args)
