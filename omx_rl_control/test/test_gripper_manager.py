"""Tests for gripper action result interpretation."""

from action_msgs.msg import GoalStatus

from omx_rl_control.gripper_manager import gripper_result_succeeded


def test_reached_goal_succeeds() -> None:
    """A normally completed position goal is successful."""
    assert gripper_result_succeeded(
        GoalStatus.STATUS_SUCCEEDED,
        reached_goal=True,
        stalled=False,
        allow_stall=False,
    )


def test_allowed_contact_stall_succeeds_when_controller_aborts() -> None:
    """Gazebo reports object contact as ABORTED with stalled set."""
    assert gripper_result_succeeded(
        GoalStatus.STATUS_ABORTED,
        reached_goal=False,
        stalled=True,
        allow_stall=True,
    )


def test_disallowed_stall_still_fails() -> None:
    """Opening and release commands must not accept a stall."""
    assert not gripper_result_succeeded(
        GoalStatus.STATUS_ABORTED,
        reached_goal=False,
        stalled=True,
        allow_stall=False,
    )


def test_canceled_command_never_succeeds() -> None:
    """A cancellation is not converted into a successful grasp."""
    assert not gripper_result_succeeded(
        GoalStatus.STATUS_CANCELED,
        reached_goal=False,
        stalled=True,
        allow_stall=True,
    )
