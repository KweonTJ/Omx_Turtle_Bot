from copy import deepcopy
from pathlib import Path
import math
import numpy as np
import pytest
import yaml

from omx_turtlebot3_runtime.core import (RobotSample, ObjectPose, PoseCache, Evidence,
    Phase, Fault, observation, residual_step, SlewLimiter, commissioning_issues)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def cfg():
    return yaml.safe_load((ROOT/'config/runtime.yaml').read_text())


def sample(stamp=10., finger=.004, current=30.):
    return RobotSample(stamp, np.zeros(4), np.zeros(4), finger, 0., current, 12., (0, 0), int(stamp*1000))


def update(cache, stamp, position, marker=0):
    if stamp > cache.frame_stamp:
        cache.begin_frame(stamp)
    pose = ObjectPose(marker, stamp, np.array(position, dtype=float), np.array([0., 0., 0., 1.]), .1)
    return cache.update(pose, stamp)


def evidence(cfg):
    c = deepcopy(cfg['estimation'])
    c.update(current_idle_raw=0., current_grip_delta_raw=20., current_release_delta_raw=5.)
    return Evidence(c)


def test_default_config_cannot_arm(cfg):
    issues = commissioning_issues(cfg)
    assert 'joint_mapping_verified' in issues and 'current_thresholds_missing' in issues


def test_unmeasured_camera_candidate_cannot_be_enabled_by_other_flags(cfg):
    for k in cfg['commissioning']:
        if k.endswith('_verified'):
            cfg['commissioning'][k] = True
    cfg['commissioning']['evidence_directory'] = '/tmp/example-not-a-measurement'
    cfg['estimation'].update(current_idle_raw=0., current_grip_delta_raw=20., current_release_delta_raw=5.)
    cfg['vision']['camera_mount_candidate'] = True
    assert 'camera_mount_is_unmeasured_candidate' in commissioning_issues(cfg)


def test_cache_rejects_unknown_replay_nonfinite_and_bad_pose():
    cache = PoseCache()
    update(cache, 10., [.22, 0, .32])
    assert not update(cache, 10., [.22, 0, .32])
    for pose in [ObjectPose(2, 10., np.zeros(3), np.array([0, 0, 0, 1]), 0.),
                 ObjectPose(0, 10., np.full(3, np.nan), np.array([0, 0, 0, 1]), 0.),
                 ObjectPose(0, 11., np.zeros(3), np.array([0, 0, 0, 1]), 0.),
                 ObjectPose(0, 10., np.zeros(3), np.zeros(4), 0.)]:
        with pytest.raises(Fault):
            cache.update(pose, 10.)


def test_slots_observation_and_occlusion():
    cache = PoseCache()
    update(cache, 10., [.22, -.05, .32], 0)
    update(cache, 10., [.23, .05, .33], 1)
    obs = observation(sample(), np.zeros(3), np.ones(4), Phase.GRIP, 0, 1,
                      cache, 0, np.ones(3), True, False, np.zeros(4), 10., 1, 10.)
    assert obs.shape == (69,)
    np.testing.assert_array_equal(obs[35:37], [1, 0])
    np.testing.assert_allclose(obs[37:40], [.22, -.05, .32])
    np.testing.assert_array_equal(obs[46:48], [1, 0])
    np.testing.assert_array_equal(obs[65:67], [1, 1])
    cache.begin_frame(10.1)
    obs = observation(sample(10.1), np.zeros(3), np.ones(4), Phase.GRIP, 0, 1,
                      cache, 0, np.ones(3), False, False, np.zeros(4), 10., 1, 10.1)
    np.testing.assert_array_equal(obs[65:67], [0, 0])
    np.testing.assert_array_equal(obs[33:35], [1, 1])  # Cached is not currently visible.


def test_rotation_never_lowers_joints():
    training = dict(joint_step=.012, residual_scale=.12, physics_timestep=.002, frame_skip=10)
    delta, action = residual_step(np.zeros(4), np.ones(4), np.ones(4), Phase.ROTATE_RIGHT,
                                  0, 1, np.zeros(4), training, .02)
    np.testing.assert_array_equal(delta[1:], 0)
    np.testing.assert_array_equal(action[1:], 0)


def test_slew_and_deadline(cfg):
    limiter = SlewLimiter(cfg['control'])
    for _ in range(100):
        step = limiter.step(np.ones(4), .05)
        assert np.max(np.abs(step)) <= .12*.05+1e-10
    with pytest.raises(Fault):
        limiter.step(np.ones(4), .5)


@pytest.mark.parametrize('finger,current', [(-.01, 30.), (.004, 0.), (.019, 30.)])
def test_empty_close_or_no_load_or_wrong_width_never_grasps(cfg, finger, current):
    e, cache = evidence(cfg), PoseCache()
    for t in np.arange(10., 11., .05):
        update(cache, t, [.22, 0, .32])
        e.update(Phase.GRIP, sample(t, finger, current), np.array([.22, 0, .326]), cache, 0, t)
    assert not e.contact and not e.grasp_confirmed


def establish_contact(e, cache):
    for t in np.arange(10., 10.5, .05):
        update(cache, t, [.22, 0, .32])
        e.update(Phase.GRIP, sample(t), np.array([.22, 0, .326]), cache, 0, t)
    assert e.contact and not e.grasp_confirmed


def test_stall_requires_visual_lift_proof(cfg):
    e, cache = evidence(cfg), PoseCache()
    establish_contact(e, cache)
    for t in np.arange(10.5, 11., .05):
        update(cache, t, [.22, 0, .32])  # Object did not move despite arm lift.
        e.update(Phase.RETRACT, sample(t), np.array([.22, 0, .351]), cache, 0, t)
    assert not e.grasp_confirmed


def test_verified_lift_and_load_loss(cfg):
    e, cache = evidence(cfg), PoseCache()
    establish_contact(e, cache)
    for t in np.arange(10.5, 10.9, .05):
        update(cache, t, [.22, 0, .345])
        e.update(Phase.RETRACT, sample(t), np.array([.22, 0, .351]), cache, 0, t)
    assert e.grasp_confirmed
    with pytest.raises(Fault, match='evidence lost'):
        for t in np.arange(11., 11.5, .05):
            e.update(Phase.ROTATE_RIGHT, sample(t, current=0), np.array([.22, 0, .351]), cache, 0, t)


@pytest.mark.parametrize('position,finger,current', [([-.116, -.32, .20], .019, 0),
    ([.2, -.32, .095], .019, 0), ([-.116, -.32, .095], .004, 30)])
def test_wrong_height_outside_deck_or_closed_gripper_not_placed(cfg, position, finger, current):
    e, cache = evidence(cfg), PoseCache()
    for t in np.arange(10., 11.5, .05):
        update(cache, t, position)
        e.update(Phase.PLACE, sample(t, finger, current), np.array(position), cache, 0, t)
    assert not e.placed


def test_placement_needs_distinct_frames_and_stationary_released_object(cfg):
    e, cache = evidence(cfg), PoseCache()
    p = [-.116, -.32, .095]
    update(cache, 10., p)
    update(cache, 10.05, p)
    for t in np.arange(10.05, 11.5, .05):
        e.update(Phase.PLACE, sample(t, .019, 0), np.array(p), cache, 0, t)
    assert not e.placed
    for t in np.arange(12., 13.5, .05):
        update(cache, t, p)
        e.update(Phase.PLACE, sample(t, .019, 0), np.array(p), cache, 0, t)
    assert e.placed


def test_uncalibrated_estimator_cannot_confirm(cfg):
    e = Evidence(cfg['estimation'])
    e.update(Phase.GRIP, sample(), np.zeros(3), PoseCache(), 0, 10.)
    assert not e.contact and not e.placed and not e.grasp_confirmed
