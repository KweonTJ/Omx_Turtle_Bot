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
    PACKAGE_ROOT / 'models/policies/arm_delivery_residual_v2'
)


def test_deployed_policy_and_metadata_are_consistent():
    """Load the checked policy only after every payload hash is verified."""
    from stable_baselines3 import PPO

    contract = load_policy_contract(ARTIFACT)
    assert contract.version == 'arm_delivery_residual_v2'
    assert contract.observation_size == 33
    assert contract.action_size == 4
    assert contract.joint_names == ('joint1', 'joint2', 'joint3', 'joint4')
    assert contract.control_period_s == pytest.approx(0.02)
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
    np.testing.assert_allclose(
        action,
        np.asarray(golden['raw_policy_action']),
        atol=1.0e-7,
    )


def test_missing_artifact_is_rejected(tmp_path):
    """A partial deployment must never reach policy inference."""
    with pytest.raises(ContractError, match='Missing policy artifact'):
        load_policy_contract(tmp_path)
