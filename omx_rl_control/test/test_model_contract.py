"""Tests for the versioned deployed policy contract."""

from pathlib import Path

import numpy as np
from omx_rl_control.model_contract import ContractError
from omx_rl_control.model_contract import load_policy_contract
from omx_rl_control.model_contract import validate_policy_spaces
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT = (
    PACKAGE_ROOT / 'models/policies/arm_delivery_pitch_hold_v4'
)
PICK_ONLY_ARTIFACT = (
    PACKAGE_ROOT / 'models/policies/arm_grasp_pitch_hold_v3'
)
LEGACY_ARTIFACT = (
    PACKAGE_ROOT / 'models/policies/arm_delivery_residual_v3_robot_stay'
)


def test_deployed_policy_and_metadata_are_consistent():
    """Load the checked policy only after every payload hash is verified."""
    from stable_baselines3 import PPO

    contract = load_policy_contract(ARTIFACT)
    assert contract.version == 'arm_delivery_pitch_hold_v4'
    assert contract.observation_size == 33
    assert contract.action_schema_version == 3
    assert contract.action_size == 4
    assert contract.supported_commands == ('PICK', 'PLACE')
    assert contract.delivery_sequence_trained
    assert contract.control_mode == (
        'reference_plus_residual_with_joint4_pitch_coupling'
    )
    assert contract.gripper_pitch_hold_enabled
    assert contract.gripper_pitch_target == pytest.approx(0.0)
    assert contract.gripper_pitch_joint4_max_step == pytest.approx(0.028)
    assert contract.joint_names == ('joint1', 'joint2', 'joint3', 'joint4')
    assert contract.control_period_s == pytest.approx(0.02)
    assert contract.stay_joint_positions == pytest.approx(
        (0.104311, 0.027612, -0.001534, -1.638291)
    )
    assert contract.ros_to_training_offset_xyz == (0.0, 0.0, 0.016)
    assert contract.training_frame == 'base_footprint'
    assert contract.ros_target_frame == 'base_link'

    model = PPO.load(contract.policy_path, device='cpu')
    validate_policy_spaces(model, contract)
    golden = yaml.safe_load(
        (PACKAGE_ROOT / 'test/data/policy_golden.yaml').read_text(
            encoding='utf-8'
        )
    )
    action, _ = model.predict(
        np.asarray(golden['observation'], dtype=np.float32),
        deterministic=True,
    )
    assert np.asarray(action).shape == (4,)
    assert np.isfinite(action).all()


def test_previous_pick_only_artifact_remains_loadable():
    """Keep the previous pitch-hold policy available for rollback."""
    contract = load_policy_contract(PICK_ONLY_ARTIFACT)
    assert contract.action_schema_version == 3
    assert contract.supported_commands == ('PICK',)
    assert not contract.delivery_sequence_trained


def test_previous_schema_two_artifact_remains_loadable():
    """Keep the prior fully robust policy available for rollback."""
    contract = load_policy_contract(LEGACY_ARTIFACT)
    assert contract.action_schema_version == 2
    assert contract.control_mode == 'reference_plus_residual'
    assert contract.supported_commands == ('PICK', 'PLACE')
    assert contract.delivery_sequence_trained
    assert not contract.gripper_pitch_hold_enabled


def test_manifest_v2_allows_colcon_symlink_install(tmp_path):
    """Accept verified payload symlinks while rejecting path traversal."""
    artifact = tmp_path / 'arm_delivery_pitch_hold_v4'
    artifact.mkdir()
    for source in ARTIFACT.iterdir():
        (artifact / source.name).symlink_to(
            source,
            target_is_directory=source.is_dir(),
        )
    contract = load_policy_contract(artifact)
    assert contract.version == 'arm_delivery_pitch_hold_v4'


def test_missing_artifact_is_rejected(tmp_path):
    """A partial deployment must never reach policy inference."""
    with pytest.raises(ContractError, match='Missing policy artifact'):
        load_policy_contract(tmp_path)
