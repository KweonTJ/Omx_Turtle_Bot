"""Tests for deterministic runtime state definitions."""

from omx_rl_control.state_machine import BASE_HOLD_STATES
from omx_rl_control.state_machine import BASE_INTERLOCK_STATES
from omx_rl_control.state_machine import policy_phase
from omx_rl_control.state_machine import PolicyPhase
from omx_rl_control.state_machine import RuntimeState
import pytest


@pytest.mark.parametrize(
    ('state', 'phase'),
    [
        (RuntimeState.PICK_REACH, PolicyPhase.PICK_REACH),
        (RuntimeState.PICK_TO_STAY, PolicyPhase.PICK_TO_STAY),
        (RuntimeState.PLACE_REACH, PolicyPhase.PLACE_REACH),
        (RuntimeState.PLACE_TO_STAY, PolicyPhase.PLACE_TO_STAY),
    ],
)
def test_policy_phase_order_matches_training(state, phase):
    """One-hot phase indices are part of the immutable policy input."""
    assert policy_phase(state) == phase


def test_non_policy_state_has_no_phase():
    """Idle states must not be passed to PPO as an invented phase."""
    with pytest.raises(ValueError, match='has no policy phase'):
        policy_phase(RuntimeState.WAIT_PICK)


def test_base_is_held_during_every_arm_or_fault_state():
    """The base interlock covers arm motion, gripper IO, and failures."""
    required = {
        RuntimeState.ALIGN_STAY,
        RuntimeState.PICK_REACH,
        RuntimeState.PICK_TO_STAY,
        RuntimeState.PLACE_REACH,
        RuntimeState.PLACE_TO_STAY,
        RuntimeState.OPEN_GRIPPER,
        RuntimeState.CLOSE_GRIPPER,
        RuntimeState.OPEN_RELEASE,
        RuntimeState.HOLD,
        RuntimeState.FAULT,
        RuntimeState.E_STOP,
    }
    assert required <= BASE_HOLD_STATES


def test_base_stop_is_rechecked_during_mission_arm_states():
    """Every mission arm or gripper state retains the stop interlock."""
    required = {
        RuntimeState.OPEN_GRIPPER,
        RuntimeState.PICK_REACH,
        RuntimeState.CLOSE_GRIPPER,
        RuntimeState.VERIFY_GRASP,
        RuntimeState.PICK_TO_STAY,
        RuntimeState.PLACE_REACH,
        RuntimeState.OPEN_RELEASE,
        RuntimeState.PLACE_TO_STAY,
    }
    assert required == BASE_INTERLOCK_STATES
