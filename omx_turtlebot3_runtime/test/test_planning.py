from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from omx_turtlebot3_runtime.core import PoseCache, ObjectPose, Phase, Fault
from omx_turtlebot3_runtime.planning import Planner

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def planner():
    runtime = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    return Planner(ROOT/'assets/scene.xml', training, runtime)


def test_fk_ik_roundtrip(planner):
    for point in ([.22, .05, .34], [.1, -.06, .35], [-.116, -.27, .30]):
        q = planner.ik(point)
        np.testing.assert_allclose(planner.fk(q), point, atol=.001)


def test_bad_limits_and_unreachable(planner):
    with pytest.raises(Fault):
        planner.check(np.full(4, 10.), Phase.RECOGNIZE)
    with pytest.raises(Fault):
        planner.ik([2, 0, .3])


def test_stay_path_checks_measured_opening_and_rejects_unmapped_start(planner):
    planner.stay_path(planner.stay+np.array([.02, 0, 0, 0]), .01)
    with pytest.raises(Fault, match='outside commissioned model limits'):
        planner.stay_path(np.array([-.1994, -2.0893, -.0215, .5446]), .01)
    with pytest.raises(Fault, match='gripper feedback'):
        planner.stay_path(planner.stay, float('nan'))


def test_marker_object_roundtrip(planner):
    q = planner.stay
    camera, optical = planner.camera_pose(q)
    obj = np.array([.22, .05, .32])
    rotation = Rotation.from_euler('z', .12).as_matrix()
    v = planner.cfg['vision']
    marker = obj+rotation @ v['object_to_marker_translation']
    t = optical.T @ (marker-camera)
    r = optical.T @ rotation @ np.asarray(v['object_to_marker_rotation'])
    center, quat = planner.object_from_marker(q, t, r)
    np.testing.assert_allclose(center, obj, atol=1e-9)
    np.testing.assert_allclose(Rotation.from_quat(quat).as_matrix(), rotation, atol=1e-9)


def test_requires_both_measured_objects(planner):
    with pytest.raises(Fault):
        planner.update_objects(PoseCache(), 0)


def test_estimator_cannot_use_a_different_deck_than_collision_scene():
    runtime = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    runtime['estimation']['deck_top_z'] = .20
    with pytest.raises(Fault, match='geometry differs'):
        Planner(ROOT/'assets/scene.xml', training, runtime)


def candidate_planner():
    runtime = yaml.safe_load((ROOT/'config/runtime_front_facing_candidate.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    return Planner(ROOT/'assets/scene.xml', training, runtime)


def test_camera_fk_equals_rendered_collision_mount(planner):
    for q in (planner.stay, np.array([.2, -.5, .5, .1])):
        position, rotation = planner.camera_pose(q)
        np.testing.assert_allclose(position, planner.data.cam_xpos[planner.camera_id], atol=1e-10)
        np.testing.assert_allclose(rotation[:, 2], -planner.data.cam_xmat[planner.camera_id].reshape(3, 3)[:, 2])


def test_forward_candidate_faces_front_at_stay_and_follows_wrist():
    p = candidate_planner()
    _, rotation = p.camera_pose(p.stay)
    heading = np.array([np.cos(p.stay[0]), np.sin(p.stay[0]), 0.])
    assert rotation[:, 2]@heading > .995
    body = p.model.body('eef_usb_camera_link').id
    assert not np.allclose(p.model.body_quat[body], [1., 0., 0., 0.])
    q = p.stay.copy(); q[3] += .2
    _, moved = p.camera_pose(q)
    assert np.linalg.norm(moved-rotation) > .1


def test_rotation_without_matching_camera_geometry_is_rejected():
    runtime = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    runtime['vision']['optical_rotation_in_link5'] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    with pytest.raises(Fault, match='differs from camera'):
        Planner(ROOT/'assets/scene.xml', training, runtime)


def test_candidate_camera_offset_cannot_drift_from_collision_body():
    runtime = yaml.safe_load((ROOT/'config/runtime_front_facing_candidate.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    runtime['vision']['lens_position_in_link5'][0] += .02
    with pytest.raises(Fault, match='differs from camera'):
        Planner(ROOT/'assets/scene.xml', training, runtime)
