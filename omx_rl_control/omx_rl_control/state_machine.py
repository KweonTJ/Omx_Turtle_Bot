"""State and policy-phase definitions for arm delivery."""

from enum import Enum
from enum import IntEnum


class PolicyPhase(IntEnum):
    """One-hot order used in the trained observation."""

    PICK_REACH = 0
    PICK_TO_STAY = 1
    PLACE_REACH = 2
    PLACE_TO_STAY = 3


class RuntimeState(str, Enum):
    """Deterministic states surrounding PPO arm motion."""

    NOT_READY = 'NOT_READY'
    ALIGN_STAY = 'ALIGN_STAY'
    STAY_EMPTY = 'STAY_EMPTY'
    WAIT_PICK = 'WAIT_PICK'
    OPEN_GRIPPER = 'OPEN_GRIPPER'
    PICK_REACH = 'PICK_REACH'
    CLOSE_GRIPPER = 'CLOSE_GRIPPER'
    VERIFY_GRASP = 'VERIFY_GRASP'
    PICK_TO_STAY = 'PICK_TO_STAY'
    WAIT_DELIVERY = 'WAIT_DELIVERY'
    PLACE_REACH = 'PLACE_REACH'
    OPEN_RELEASE = 'OPEN_RELEASE'
    PLACE_TO_STAY = 'PLACE_TO_STAY'
    COMPLETE = 'COMPLETE'
    HOLD = 'HOLD'
    FAULT = 'FAULT'
    E_STOP = 'E_STOP'


POLICY_STATES = {
    RuntimeState.PICK_REACH: PolicyPhase.PICK_REACH,
    RuntimeState.PICK_TO_STAY: PolicyPhase.PICK_TO_STAY,
    RuntimeState.PLACE_REACH: PolicyPhase.PLACE_REACH,
    RuntimeState.PLACE_TO_STAY: PolicyPhase.PLACE_TO_STAY,
}


BASE_HOLD_STATES = {
    RuntimeState.ALIGN_STAY,
    RuntimeState.WAIT_PICK,
    RuntimeState.OPEN_GRIPPER,
    RuntimeState.PICK_REACH,
    RuntimeState.CLOSE_GRIPPER,
    RuntimeState.VERIFY_GRASP,
    RuntimeState.PICK_TO_STAY,
    RuntimeState.PLACE_REACH,
    RuntimeState.OPEN_RELEASE,
    RuntimeState.PLACE_TO_STAY,
    RuntimeState.HOLD,
    RuntimeState.FAULT,
    RuntimeState.E_STOP,
}


BASE_INTERLOCK_STATES = {
    RuntimeState.OPEN_GRIPPER,
    RuntimeState.PICK_REACH,
    RuntimeState.CLOSE_GRIPPER,
    RuntimeState.VERIFY_GRASP,
    RuntimeState.PICK_TO_STAY,
    RuntimeState.PLACE_REACH,
    RuntimeState.OPEN_RELEASE,
    RuntimeState.PLACE_TO_STAY,
}


def policy_phase(state: RuntimeState) -> PolicyPhase:
    """Return the policy phase for an active arm-motion state."""
    try:
        return POLICY_STATES[state]
    except KeyError as error:
        raise ValueError(f'State has no policy phase: {state.value}') from error
