"""Golden-vector parity tests for the deployed PPO runtime."""

from pathlib import Path

import numpy as np
from omx_rl_control.action_limiter import ActionLimitConfig
from omx_rl_control.action_limiter import ActionLimiter
from omx_rl_control.kinematics import OpenManipulatorKinematics
from omx_rl_control.observation_builder import ObservationBuilder
from omx_rl_control.observation_builder import ObservationConfig
from omx_rl_control.reference_controller import ReferenceController
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GOLDEN = yaml.safe_load(
    (PACKAGE_ROOT / 'test/data/policy_golden.yaml').read_text(
        encoding='utf-8'
    )
)
JOINT_LOW = np.array(
    [-2.82743, -1.79071, -0.942478, -1.79071], dtype=np.float64
)
JOINT_HIGH = np.array(
    [2.82743, 1.57080, 1.38230, 2.04204], dtype=np.float64
)
ACTION_SCALE = np.full(4, 0.014, dtype=np.float64)


def _array(name):
    return np.asarray(GOLDEN[name], dtype=np.float64)


def _kinematics():
    return OpenManipulatorKinematics(JOINT_LOW, JOINT_HIGH)


def _reference():
    return ReferenceController(
        _kinematics(),
        np.array([0.0, 0.0, 1.38, -1.38]),
        ACTION_SCALE,
        np.array([
            [0.0, -0.5, 0.5, 0.0],
            [0.0, 0.5, 0.2, -0.7],
        ]),
        waypoint_tolerance=0.08,
        pregrasp_height_offset=0.025,
        action_limit=1.0,
        final_action_limit=1.0,
    )


def test_fk_and_observation_match_training_environment():
    """ROS-frame FK and the full 33D vector must match MuJoCo."""
    frame_offset = _array('frame_offset_xyz')
    qpos = _array('arm_qpos')
    eef_ros = _kinematics().forward(qpos)
    np.testing.assert_allclose(
        eef_ros + frame_offset,
        _array('eef_training_frame'),
        atol=1.0e-12,
    )

    builder = ObservationBuilder(ObservationConfig(
        joint_low=JOINT_LOW,
        joint_high=JOINT_HIGH,
        joint_velocity_scale=np.full(4, 2.0),
        workspace_min=np.array([0.10, -0.18, 0.08]),
        workspace_max=np.array([0.42, 0.18, 0.52]),
        arm_base_xy=np.array([-0.08, 0.0]),
        policy_frame_offset=frame_offset,
    ))
    observation = builder.build(
        qpos,
        _array('arm_qvel'),
        GOLDEN['gripper_position'],
        GOLDEN['gripper_velocity'],
        eef_ros,
        _array('active_target_ros_base_link'),
        0.0,
        GOLDEN['grasped'],
        GOLDEN['phase_index'],
        _array('previous_action'),
    )
    assert observation.dtype == np.float32
    np.testing.assert_array_equal(observation, _array('observation'))


def test_reference_and_ik_match_training_environment():
    """Waypoints, normalized reference, and final IK must stay aligned."""
    reference = _reference()
    reference.set_target(_array('vision_position_ros_base_link'), force=True)
    np.testing.assert_allclose(
        reference.waypoints,
        _array('approach_joint_waypoints'),
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        reference.active_target() + _array('frame_offset_xyz'),
        np.asarray(GOLDEN['approach_eef_targets_training_frame'])[0],
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        reference.action(_array('arm_target_before')),
        _array('reference_action'),
        atol=1.0e-12,
    )

    reference.advance(reference.waypoints[0])
    reference.advance(reference.waypoints[1])
    assert reference.final_stage
    np.testing.assert_allclose(
        reference.pregrasp_joints,
        _array('reference_pregrasp_joints'),
        atol=1.0e-6,
    )


def test_target_correction_is_applied_at_final_waypoint():
    """Vision correction must update final IK without restarting waypoints."""
    reference = _reference()
    original = _array('vision_position_ros_base_link')
    reference.set_target(original, force=True)
    first_solution = reference.pregrasp_joints.copy()
    reference.advance(reference.waypoints[0])
    corrected = original + np.array([0.008, -0.005, 0.0])
    assert reference.set_target(corrected)
    np.testing.assert_array_equal(reference.pregrasp_joints, first_solution)
    assert reference.stage == 1

    reference.advance(reference.waypoints[1])
    assert reference.final_stage
    assert not np.allclose(reference.pregrasp_joints, first_solution)


def test_one_action_step_matches_training_environment():
    """Residual combination, EMA, and integration must match one env step."""
    limiter = ActionLimiter(ActionLimitConfig(
        joint_low=JOINT_LOW,
        joint_high=JOINT_HIGH,
        action_scale=ACTION_SCALE,
        filter_coefficient=0.18,
        residual_scale=0.10,
        control_period_s=0.02,
        max_velocity=np.full(4, 0.70),
        max_acceleration=np.full(4, 8.0),
    ))
    limiter.reset(_array('arm_target_before'))
    next_target = limiter.step(
        _array('raw_policy_action'),
        _array('reference_action'),
    )
    np.testing.assert_allclose(
        limiter.filtered_action,
        _array('filtered_action'),
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        next_target,
        _array('arm_target_after_one_step'),
        atol=1.0e-12,
    )


def test_zero_residual_ignores_policy_action():
    """Reference-only A/B mode must not depend on the PPO output."""
    config = ActionLimitConfig(
        joint_low=JOINT_LOW,
        joint_high=JOINT_HIGH,
        action_scale=ACTION_SCALE,
        filter_coefficient=0.18,
        residual_scale=0.0,
        control_period_s=0.02,
        max_velocity=np.full(4, 0.70),
        max_acceleration=np.full(4, 8.0),
    )
    with_policy = ActionLimiter(config)
    without_policy = ActionLimiter(config)
    initial = _array('arm_target_before')
    reference = _array('reference_action')
    with_policy.reset(initial)
    without_policy.reset(initial)

    actual = with_policy.step(_array('raw_policy_action'), reference)
    expected = without_policy.step(np.zeros(4), reference)

    np.testing.assert_allclose(actual, expected, atol=1.0e-12)
