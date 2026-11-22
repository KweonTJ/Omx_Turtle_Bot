import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

ROOT=Path(__file__).resolve().parents[1]


def inputs(tmp_path,monkeypatch,execute):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'launch_ros'))
    spec=importlib.util.spec_from_file_location('policy_trial_launch',ROOT/'launch/ros_policy_pick_place.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    context.launch_configurations.update(execute=execute,start_camera='false',
        python_executable=sys.executable,log_dir=str(tmp_path/'trial'),
        ros_domain_id='225',mission_timeout_s='300',video_device='/dev/test-only',
        gripper_down_joint4_offset_rad='0.20',pause_before_grasp='false',pause_before_rotate='false',marker_id='0')
    return module,context


@pytest.mark.parametrize('execute',['false','true'])
@pytest.mark.parametrize('paused',['false','true'])
@pytest.mark.parametrize('retract_paused',['false','true'])
@pytest.mark.parametrize('marker',['0','1'])
def test_one_trial_launch_uses_coupled_wrist_and_explicit_execution(tmp_path,monkeypatch,execute,paused,retract_paused,marker):
    module,context=inputs(tmp_path,monkeypatch,execute)
    context.launch_configurations['pause_before_grasp']=paused
    context.launch_configurations['pause_before_rotate']=retract_paused
    context.launch_configurations['marker_id']=marker
    actions=module.build(context)
    assert len([x for x in actions if isinstance(x,ExecuteProcess)])==1 # Only model check initially.
    children=actions[1].event_handler.handle(SimpleNamespace(returncode=0),context)
    commands=[[perform_substitutions(context,s) for s in x.cmd]
              for x in children if isinstance(x,ExecuteProcess)]
    gate=next(x for x in commands if 'omx_turtlebot3_runtime.ros_trial_gate' in x)
    observer=next(x for x in commands if 'omx_turtlebot3_runtime.ros_policy_observer' in x)
    assert gate[gate.index('--selected-id')+1]==marker
    assert observer[observer.index('--selected-id')+1]==marker
    assert ('--execute' in gate)==(execute=='true')
    profile=yaml.safe_load((tmp_path/'trial/controller_profile.yaml').read_text())['mp_control_node']['ros__parameters']
    assert profile['hold_joint4_during_extension'] is False
    assert profile['pregrasp_preserve_gripper_roll'] is True
    assert profile['pregrasp_move_duration_s']==.35
    assert profile['pregrasp_joint3_max_step_rad']==.25
    assert profile['handoff_rotate_odd_angle_deg']==profile['handoff_rotate_even_angle_deg']==-90.
    assert profile['gripper_down_joint4_offset_rad']==.2
    assert profile['pause_before_grasp'] is (paused=='true')
    assert profile['pause_before_rotate'] is (retract_paused=='true')
    assert profile['handoff_lift_joint2_delta_rad']==0.
    assert profile['handoff_retract_joint3_delta_rad']==.65
    assert profile['handoff_retract_hold_z'] is True
    assert profile['handoff_lower_before_release'] is False
    assert profile['handoff_place_duration_s']==2.
    assert profile['pregrasp_republish_period_s']>=profile['pregrasp_move_duration_s']+profile['pregrasp_settle_s']
    assert profile['auto_start'] is False and profile['auto_start_on_bbox'] is False
    assert profile['handoff_joint_trajectory_topic']=='/omx_v6/nominal_joint_trajectory'
    assert profile['joint_trajectory_topic']=='/arm_controller/joint_trajectory_raw'


def test_invalid_onnx_never_starts_trial_components(tmp_path,monkeypatch):
    module,context=inputs(tmp_path,monkeypatch,'true')
    actions=module.build(context)
    with pytest.raises(RuntimeError,match='mission not started'):
        actions[1].event_handler.handle(SimpleNamespace(returncode=1),context)
    assert not (tmp_path/'trial').exists()
