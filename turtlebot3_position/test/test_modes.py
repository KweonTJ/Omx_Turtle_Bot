"""Pure mode-transition and interactive-command tests (no ROS spin)."""

import math

import pytest

from turtlebot3_position.core import (
    DeliveryModeState,
    DeliveryPhase,
    parse_console_command,
)


@pytest.fixture
def waypoints():
    return {
        'pickup': (0.80, 0.93),
        'tower1': (0.20, 0.20),
        'tower2': (1.55, 0.93),
        'tower3': (1.55, 0.90),
        'safe': (0.80, 0.20),
    }


def test_delivery_phase_contains_every_required_phase():
    assert {phase.value for phase in DeliveryPhase} == {
        'IDLE',
        'TO_PICKUP',
        'WAIT_PICKUP',
        'TO_TOWER',
        'WAIT_DELIVERY',
        'TO_SAFE',
        'SAFE',
        'MANUAL',
        'ESTOP',
    }


@pytest.mark.parametrize('tower', ['tower1', 'tower2', 'tower3'])
def test_delivery_request_starts_at_pickup_for_each_tower(tower):
    mode = DeliveryModeState()

    accepted, event = mode.request_delivery(tower)

    assert accepted is True
    assert event == f'accepted:{tower}'
    assert mode.phase is DeliveryPhase.TO_PICKUP
    assert mode.requested_tower == tower
    assert mode.automatic_active is True


@pytest.mark.parametrize(
    'value', ['tower4', 'pickup', 'safe', 'safezone', '', 'nonsense'])
def test_invalid_delivery_request_is_rejected_without_state_change(value):
    mode = DeliveryModeState()

    accepted, event = mode.request_delivery(value)

    assert accepted is False
    assert event == 'invalid_request'
    assert mode.phase is DeliveryPhase.IDLE
    assert mode.requested_tower is None


@pytest.mark.parametrize('start_phase', [DeliveryPhase.IDLE, DeliveryPhase.SAFE])
def test_delivery_request_is_allowed_only_from_idle_or_safe(start_phase):
    mode = DeliveryModeState(phase=start_phase)
    accepted, _ = mode.request_delivery('tower1')
    assert accepted is True
    assert mode.phase is DeliveryPhase.TO_PICKUP


@pytest.mark.parametrize(
    'busy_phase',
    [
        DeliveryPhase.TO_PICKUP,
        DeliveryPhase.WAIT_PICKUP,
        DeliveryPhase.TO_TOWER,
        DeliveryPhase.WAIT_DELIVERY,
        DeliveryPhase.TO_SAFE,
        DeliveryPhase.MANUAL,
        DeliveryPhase.ESTOP,
    ],
)
def test_delivery_request_is_rejected_while_busy_manual_or_estopped(busy_phase):
    mode = DeliveryModeState(
        phase=busy_phase,
        requested_tower='tower2' if busy_phase is not DeliveryPhase.MANUAL else None,
    )

    accepted, event = mode.request_delivery('tower1')

    assert accepted is False
    assert event == f'busy:{busy_phase.value}'
    assert mode.phase is busy_phase
    assert mode.requested_tower != 'tower1'


@pytest.mark.parametrize('tower', ['tower1', 'tower2', 'tower3'])
def test_automatic_delivery_follows_pickup_tower_safe_order(tower):
    mode = DeliveryModeState()
    accepted, accepted_event = mode.request_delivery(tower)
    assert (accepted, accepted_event) == (True, f'accepted:{tower}')

    next_target, event = mode.arrive()
    assert next_target is None
    assert event == 'arrived:pickup'
    assert mode.phase is DeliveryPhase.WAIT_PICKUP
    assert mode.requested_tower == tower

    next_target, event = mode.complete_wait()
    assert next_target == tower
    assert event == 'pickup_wait_complete'
    assert mode.phase is DeliveryPhase.TO_TOWER

    next_target, event = mode.arrive()
    assert next_target is None
    assert event == f'arrived:{tower}'
    assert mode.phase is DeliveryPhase.WAIT_DELIVERY

    next_target, event = mode.complete_wait()
    assert next_target == 'safe'
    assert event == 'delivery_wait_complete'
    assert mode.phase is DeliveryPhase.TO_SAFE

    next_target, event = mode.arrive()
    assert next_target is None
    assert event == 'arrived:safe'
    assert mode.phase is DeliveryPhase.SAFE
    assert mode.requested_tower is None
    assert mode.automatic_active is False


def test_wait_transition_does_nothing_outside_wait_phases():
    mode = DeliveryModeState()
    assert mode.complete_wait() == (None, None)
    assert mode.phase is DeliveryPhase.IDLE


def test_pickup_wait_cannot_advance_without_requested_tower():
    mode = DeliveryModeState(phase=DeliveryPhase.WAIT_PICKUP)
    assert mode.complete_wait() == (None, None)
    assert mode.phase is DeliveryPhase.WAIT_PICKUP


@pytest.mark.parametrize(
    'automatic_phase',
    [
        DeliveryPhase.TO_PICKUP,
        DeliveryPhase.WAIT_PICKUP,
        DeliveryPhase.TO_TOWER,
        DeliveryPhase.WAIT_DELIVERY,
        DeliveryPhase.TO_SAFE,
    ],
)
def test_manual_goal_cancels_any_active_automatic_mission(automatic_phase):
    mode = DeliveryModeState(
        phase=automatic_phase, requested_tower='tower2')

    event = mode.start_manual()

    assert event == 'mission_cancelled:manual_goal'
    assert mode.phase is DeliveryPhase.MANUAL
    assert mode.requested_tower is None
    assert mode.automatic_active is False


@pytest.mark.parametrize(
    'nonautomatic_phase',
    [DeliveryPhase.IDLE, DeliveryPhase.SAFE, DeliveryPhase.MANUAL],
)
def test_manual_goal_enters_or_remains_manual_without_false_cancel_event(
        nonautomatic_phase):
    mode = DeliveryModeState(phase=nonautomatic_phase)

    event = mode.start_manual()

    assert event is None
    assert mode.phase is DeliveryPhase.MANUAL
    assert mode.requested_tower is None


def test_manual_goal_is_rejected_during_estop():
    mode = DeliveryModeState(phase=DeliveryPhase.ESTOP)

    event = mode.start_manual()

    assert event == 'manual_goal_rejected:estop'
    assert mode.phase is DeliveryPhase.ESTOP
    assert mode.requested_tower is None


def test_manual_arrival_stays_manual_and_never_selects_another_waypoint():
    mode = DeliveryModeState()
    mode.start_manual()

    next_target, event = mode.arrive()

    assert next_target is None
    assert event == 'manual_arrived:goal'
    assert mode.phase is DeliveryPhase.MANUAL
    assert mode.requested_tower is None
    assert mode.complete_wait() == (None, None)


@pytest.mark.parametrize('manual_name', ['pickup', 'tower1', 'tower2', 'tower3', 'safe'])
def test_every_named_manual_goal_is_a_single_destination(manual_name, waypoints):
    command = parse_console_command(manual_name, waypoints)
    assert command.kind == 'goal'
    assert command.waypoint == manual_name
    assert command.target == waypoints[manual_name]

    mode = DeliveryModeState()
    mode.start_manual()
    next_target, event = mode.arrive()
    assert (next_target, event) == (None, 'manual_arrived:goal')
    assert mode.phase is DeliveryPhase.MANUAL
    assert mode.complete_wait() == (None, None)


def test_estop_remembers_phase_but_does_not_resume_until_explicitly_requested():
    mode = DeliveryModeState()
    mode.request_delivery('tower3')

    event = mode.enter_estop()

    assert event == 'emergency_stop'
    assert mode.phase is DeliveryPhase.ESTOP
    assert mode.phase_before_estop is DeliveryPhase.TO_PICKUP
    assert mode.request_delivery('tower1')[0] is False
    assert mode.start_manual() == 'manual_goal_rejected:estop'

    # The ROS interlock calls this only after a fresh enable=true.
    resumed = mode.resume_after_estop()
    assert resumed is DeliveryPhase.TO_PICKUP


def test_duplicate_estop_does_not_overwrite_the_saved_phase():
    mode = DeliveryModeState(phase=DeliveryPhase.TO_TOWER,
                             requested_tower='tower1')
    assert mode.enter_estop() == 'emergency_stop'
    assert mode.enter_estop() is None
    assert mode.phase_before_estop is DeliveryPhase.TO_TOWER


@pytest.mark.parametrize(
    'active_phase',
    [
        DeliveryPhase.TO_PICKUP,
        DeliveryPhase.WAIT_PICKUP,
        DeliveryPhase.TO_TOWER,
        DeliveryPhase.WAIT_DELIVERY,
        DeliveryPhase.TO_SAFE,
        DeliveryPhase.MANUAL,
    ],
)
def test_disable_cancels_automatic_and_manual_missions(active_phase):
    mode = DeliveryModeState(phase=active_phase, requested_tower='tower1')

    event = mode.cancel('disable')

    assert event == 'mission_cancelled:disable'
    assert mode.phase is DeliveryPhase.IDLE
    assert mode.requested_tower is None


def test_disable_while_estopped_clears_the_saved_mission():
    mode = DeliveryModeState(phase=DeliveryPhase.TO_TOWER,
                             requested_tower='tower2')
    mode.enter_estop()

    event = mode.cancel('disable')

    assert event == 'mission_cancelled:disable'
    assert mode.phase is DeliveryPhase.ESTOP
    assert mode.phase_before_estop is DeliveryPhase.IDLE
    assert mode.requested_tower is None
    assert mode.resume_after_estop() is DeliveryPhase.IDLE


@pytest.mark.parametrize(
    ('text', 'canonical'),
    [
        ('tower1', 'tower1'),
        ('tower2', 'tower2'),
        ('tower3', 'tower3'),
        ('1', 'tower1'),
        ('2', 'tower2'),
        ('3', 'tower3'),
        ('pickup', 'pickup'),
        ('safe', 'safe'),
        ('safezone', 'safe'),
    ],
)
def test_console_waypoint_aliases_use_parameter_supplied_coordinates(
        text, canonical, waypoints):
    command = parse_console_command(text, waypoints)
    assert command.kind == 'goal'
    assert command.waypoint == canonical
    assert command.target == waypoints[canonical]
    assert command.yaw is None


def test_console_waypoints_are_not_hardcoded_in_the_parser():
    custom = {
        'pickup': (-10.0, 11.0),
        'tower1': (12.0, -13.0),
        'tower2': (14.0, -15.0),
        'tower3': (16.0, -17.0),
        'safe': (18.0, -19.0),
    }
    for name, coordinate in custom.items():
        assert parse_console_command(name, custom).target == coordinate


def test_console_direct_xy_is_in_metres_without_yaw(waypoints):
    command = parse_console_command('0.875 0.565', waypoints)
    assert command.kind == 'goal'
    assert command.target == (0.875, 0.565)
    assert command.yaw is None
    assert command.waypoint is None


def test_console_direct_xy_yaw_converts_degrees_to_radians(waypoints):
    command = parse_console_command('0.8 0.93 90', waypoints)
    assert command.kind == 'goal'
    assert command.target == (0.8, 0.93)
    assert math.isclose(command.yaw, math.pi / 2.0, abs_tol=1.0e-12)


@pytest.mark.parametrize('text', ['disable', 'stop', ' DISABLE '])
def test_console_disable_and_stop_commands(text, waypoints):
    command = parse_console_command(text, waypoints)
    assert command.kind == 'disable'
    assert command.target is None
    assert command.yaw is None


@pytest.mark.parametrize('text', ['quit', 'exit', ' EXIT '])
def test_console_quit_and_exit_commands(text, waypoints):
    command = parse_console_command(text, waypoints)
    assert command.kind == 'quit'
    assert command.target is None


@pytest.mark.parametrize(
    'text',
    [
        '',
        'unknown',
        '0.1',
        '0.1 0.2 30 extra',
        'x y',
        'nan 0.2',
        'inf 0.2',
        '0.1 -inf 30',
    ],
)
def test_console_rejects_invalid_or_nonfinite_direct_goals(text, waypoints):
    assert parse_console_command(text, waypoints) is None


def test_manual_tower_console_command_never_starts_automatic_delivery(waypoints):
    command = parse_console_command('tower2', waypoints)
    mode = DeliveryModeState()

    assert command.kind == 'goal'
    mode.start_manual()

    assert mode.phase is DeliveryPhase.MANUAL
    assert mode.requested_tower is None
    assert mode.automatic_active is False
