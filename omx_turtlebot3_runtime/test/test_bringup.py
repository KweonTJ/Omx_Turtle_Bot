"""Single-launch ordering and physical readiness gates; no robot connection."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from omx_turtlebot3_runtime.commissioning import commissioning_issues, thresholds_calibrated
from omx_turtlebot3_runtime.launch_support import build_actions, runtime_python, calibrated_camera

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def launch_inputs(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'launch-logs'))
    context = LaunchContext()
    context.launch_configurations.update({
        'mode': 'read_only', 'config': str(ROOT/'config/runtime.yaml'),
        'scene': str(ROOT/'assets/scene.xml'),
        'model_dir': str(ROOT.parent/'omx_turtlebot3_policy/models/omx_turtlebot3_v6'),
        'python_executable': sys.executable, 'start_camera': 'false',
        'camera_info_url': 'file://'+str(ROOT/'config/eef_usb_camera.yaml'),
        'video_device': '/dev/test-only-camera', 'duration_s': '1',
        'ros_domain_id': '207', 'log_dir': str(tmp_path/'new-run'),
    })
    kwargs = dict(runtime_share=ROOT, policy_share=ROOT.parent/'omx_turtlebot3_policy',
                  runtime_prefix=ROOT)
    return context, kwargs


def command(action, context):
    return [perform_substitutions(context, value) for value in action.cmd]


def test_only_startup_check_runs_before_native_runtime(launch_inputs):
    context, kwargs = launch_inputs
    actions = build_actions(context, **kwargs)
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 1
    check = processes[0]
    assert '--check-startup' in command(check, context)
    assert '--execute' not in command(check, context)
    after = actions[1].event_handler.handle(SimpleNamespace(returncode=0), context)
    runtime = next(a for a in after if isinstance(a, ExecuteProcess))
    assert '--backend' in command(runtime, context) and 'cpp_onnx' in command(runtime, context)
    assert '--check-startup' not in command(runtime, context)
    assert '--execute' not in command(runtime, context)
    assert '--exit-after-mission' not in command(runtime, context)


def test_execute_launch_exits_after_the_one_mission(launch_inputs, tmp_path):
    context, kwargs = launch_inputs
    cfg = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    for key in cfg['commissioning']:
        if key.endswith('_verified'):
            cfg['commissioning'][key] = True
    cfg['commissioning']['evidence_directory'] = 'SYNTHETIC_LAUNCH_TEST_ONLY'
    cfg['estimation'].update(current_idle_raw=0, current_grip_delta_raw=10,
                             current_release_delta_raw=5)
    path = tmp_path/'synthetic_launch.yaml'
    path.write_text(yaml.safe_dump(cfg))
    context.launch_configurations.update(mode='execute', config=str(path))
    actions = build_actions(context, **kwargs)
    after = actions[1].event_handler.handle(SimpleNamespace(returncode=0), context)
    runtime = next(a for a in after if isinstance(a, ExecuteProcess))
    assert '--exit-after-mission' in command(runtime, context)
    assert '--execute' in command(runtime, context)


def test_failed_startup_never_starts_camera_or_motor_io(launch_inputs):
    context, kwargs = launch_inputs
    actions = build_actions(context, **kwargs)
    with pytest.raises(RuntimeError, match='were not started'):
        actions[1].event_handler.handle(SimpleNamespace(returncode=1), context)


def test_default_execute_rejected_before_creating_processes(launch_inputs):
    context, kwargs = launch_inputs
    context.launch_configurations['mode'] = 'execute'
    with pytest.raises(ValueError, match='Execution blocked before hardware startup'):
        build_actions(context, **kwargs)


def test_runtime_failure_is_a_launch_failure(launch_inputs):
    context, kwargs = launch_inputs
    actions = build_actions(context, **kwargs)
    with pytest.raises(RuntimeError, match='runtime failed'):
        actions[2].event_handler.handle(SimpleNamespace(returncode=1), context)
    assert actions[2].event_handler.handle(SimpleNamespace(returncode=0), context)


def test_camera_and_controller_share_one_launch_after_check(launch_inputs, monkeypatch):
    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda _: '/test-installed')
    context, kwargs = launch_inputs
    context.launch_configurations['start_camera'] = 'true'
    actions = build_actions(context, **kwargs)
    after = actions[1].event_handler.handle(SimpleNamespace(returncode=0), context)
    assert len([a for a in after if isinstance(a, Node)]) == 1
    assert len([a for a in after if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]) == 1
    with pytest.raises(RuntimeError, match='camera exited unexpectedly'):
        actions[3].event_handler.handle(SimpleNamespace(returncode=1), context)


@pytest.mark.parametrize('argument,value', [('duration_s', 'nan'), ('duration_s', '-1'),
    ('ros_domain_id', '233'), ('start_camera', 'yes'), ('mode', 'auto')])
def test_invalid_launch_options_rejected(launch_inputs, argument, value):
    context, kwargs = launch_inputs
    context.launch_configurations[argument] = value
    with pytest.raises(ValueError):
        build_actions(context, **kwargs)


def test_log_directory_is_not_overwritten(launch_inputs):
    context, kwargs = launch_inputs
    Path(context.launch_configurations['log_dir']).mkdir()
    with pytest.raises(ValueError, match='never overwritten'):
        build_actions(context, **kwargs)


def test_python_preserves_venv_symlink_and_explicit_failure(tmp_path):
    executable = tmp_path/'venv/bin/python'
    executable.parent.mkdir(parents=True)
    executable.symlink_to(sys.executable)
    assert runtime_python('/unused', str(executable)) == str(executable)
    with pytest.raises(ValueError, match='No executable Python'):
        runtime_python('/unused', str(tmp_path/'missing'))


def test_missing_verification_cannot_be_removed_to_enable_execution():
    config = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    config['commissioning'].pop('joint_mapping_verified')
    assert 'joint_mapping_verified' in commissioning_issues(config)
    assert not thresholds_calibrated(dict(current_idle_raw=0, current_grip_delta_raw=True,
                                         current_release_delta_raw=0))


def test_camera_calibration_matches_existing_driver_contract(tmp_path):
    original = calibrated_camera(ROOT/'config/eef_usb_camera.yaml')
    original['image_width'] = 640
    changed = tmp_path/'camera.yaml'
    changed.write_text(yaml.safe_dump(original))
    with pytest.raises(ValueError, match='320x240'):
        calibrated_camera(changed)
