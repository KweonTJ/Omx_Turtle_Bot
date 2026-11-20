from pathlib import Path
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.planning import Planner
from omx_turtlebot3_runtime.robot_contract import RobotUrdf, mp_controller_trajectory

ROOT = Path(__file__).resolve().parents[1]

def planner():
    return Planner(ROOT/'assets/scene.xml',
        yaml.safe_load((ROOT/'config/training_contract.yaml').read_text()),
        yaml.safe_load((ROOT/'config/runtime.yaml').read_text()))

def test_installed_urdf_camera_equals_fk_through_all_joints():
    p = planner()
    for q in (p.stay, np.array([.3, -.4, .2, -.6]), np.array([-.2, .7, -.3, .1])):
        position, rotation = p.camera_pose(q)
        expected = p.policy_from_base @ p.robot_urdf.frame('eef_usb_camera_optical_frame',
            dict(zip([f'joint{i}' for i in range(1, 5)], q)))
        np.testing.assert_allclose(position, expected[:3, 3], atol=1e-8)
        np.testing.assert_allclose(rotation, expected[:3, :3], atol=1e-8)

def test_urdf_has_no_mujoco_only_57mm_optical_offset():
    p = planner()
    np.testing.assert_allclose(p.model.cam_pos[p.camera_id], [0, 0, 0], atol=1e-10)
    _, optical = p.camera_pose(p.stay)
    assert optical[0, 2] > .99

def test_urdf_changes_require_explicit_contract_update():
    with pytest.raises(Fault, match='checksum'):
        RobotUrdf(ROOT/'assets/robot_real.urdf', '0'*64)

def test_urdf_manual_mount_mix_rejected():
    p = planner()
    p.cfg['vision']['camera_body_position_in_link5'] = [0, 0, 0]
    with pytest.raises(Fault, match='cannot be mixed'):
        Planner(ROOT/'assets/scene.xml', p.t, p.cfg)

def test_mp_roundtrip_preserves_controller_target_and_joint4():
    reference = np.array([.1, .2, .1, -.3])
    target = reference + [.01, .01, .05, -.06]
    output, record = mp_controller_trajectory(reference, target)
    assert record['raw_q'][2] == pytest.approx(.05)
    np.testing.assert_allclose(output, target, atol=1e-12)
    assert record['joint4_compensated_again'] is False

def test_mp_converter_rejects_large_or_nonfinite_goals():
    for target in ([0, 0, .21, 0], [0, 0, np.nan, 0]):
        with pytest.raises(Fault):
            mp_controller_trajectory(np.zeros(4), target)

def test_mp_converter_rejects_raw_space_clipping():
    with pytest.raises(Fault, match='bounds'):
        mp_controller_trajectory([0, 0, 1.37, 0], [0, 0, 1.20, 0])

def test_rejected_waypoint_identifies_stage():
    with pytest.raises(Fault, match='lift: Target not reachable'):
        planner()._plan_ik('lift', [2, 0, .3], 0.)
