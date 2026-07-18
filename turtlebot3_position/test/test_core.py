"""Hardware-independent tests for :mod:`turtlebot3_position.core`."""

import math

import pytest

from turtlebot3_position.core import (
    ControlInterlock,
    normalize_angle,
    parse_manual_goal,
    parse_position,
    parse_tower,
    quaternion_to_yaw,
    target_error,
    yaw_to_quaternion,
)


@pytest.mark.parametrize(
    ('line', 'expected'),
    [
        ('X=0.875, Y=0.565', (0.875, 0.565)),
        ('X = -1.25, Y = 2.5', (-1.25, 2.5)),
        (
            '01=1.23, 02=1.45, 03=0.98, 04=1.62, '
            'X=0.875, Y=0.565',
            (0.875, 0.565),
        ),
    ],
)
def test_parse_position_accepts_finite_metre_coordinates(line, expected):
    assert parse_position(line) == expected


@pytest.mark.parametrize(
    'line',
    [
        'X=?, Y=?',
        'X=nan, Y=0.5',
        'X=NaN, Y=0.5',
        'X=inf, Y=0.5',
        'X=+inf, Y=0.5',
        'X=-inf, Y=0.5',
        'X=Infinity, Y=0.5',
        'X=-Infinity, Y=0.5',
        'NaN',
        'Infinity',
        'X=hello, Y=0.5',
        'X=0.5, Y=world',
        'X=0.5, Y=2junk',
        'X=0.5, Y=2.3.4',
        'prefixX=0.5, Y=0.5',
        'X=0.5',
        'Y=0.5',
        '',
        'not a position',
    ],
)
def test_parse_position_rejects_invalid_or_nonfinite_coordinates(line):
    assert parse_position(line) is None


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (-3.0 * math.pi, -math.pi),
        (2.0 * math.pi + 0.25, 0.25),
        (-2.0 * math.pi - 0.25, -0.25),
    ],
)
def test_normalize_angle_uses_half_open_radian_range(angle, expected):
    result = normalize_angle(angle)
    assert -math.pi <= result < math.pi
    assert math.isclose(result, expected, abs_tol=1.0e-12)


@pytest.mark.parametrize(
    'yaw',
    [-math.pi, -2.1, -0.5, 0.0, 0.5, 2.1, math.pi],
)
def test_quaternion_yaw_round_trip(yaw):
    quaternion = yaw_to_quaternion(yaw)
    assert len(quaternion) == 4
    recovered, valid = quaternion_to_yaw(*quaternion)
    assert valid is True
    assert math.isclose(
        normalize_angle(recovered), normalize_angle(yaw), abs_tol=1.0e-12)


def test_quaternion_to_yaw_normalizes_non_unit_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi / 3.0)
    recovered, valid = quaternion_to_yaw(2.0 * x, 2.0 * y, 2.0 * z, 2.0 * w)
    assert valid is True
    assert math.isclose(recovered, math.pi / 3.0, abs_tol=1.0e-12)


@pytest.mark.parametrize(
    'quaternion',
    [
        (0.0, 0.0, 0.0, 0.0),
        (math.nan, 0.0, 0.0, 1.0),
        (0.0, math.inf, 0.0, 1.0),
    ],
)
def test_quaternion_to_yaw_rejects_invalid_quaternion(quaternion):
    _, valid = quaternion_to_yaw(*quaternion)
    assert valid is False


def test_target_error_returns_distance_and_wrapped_heading_error():
    distance, heading_error = target_error(
        (0.0, 0.0), (1.0, 1.0), 0.0)
    assert math.isclose(distance, math.sqrt(2.0), abs_tol=1.0e-12)
    assert math.isclose(heading_error, math.pi / 4.0, abs_tol=1.0e-12)

    distance, heading_error = target_error(
        (0.0, 0.0), (-1.0, -0.01), math.pi - 0.01)
    assert distance > 1.0
    assert -math.pi <= heading_error < math.pi
    assert abs(heading_error) < 0.03


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        ('tower1', 'tower1'),
        ('TOWER_2', 'tower2'),
        (' tower3 ', 'tower3'),
        ('1', 'tower1'),
        ('2', 'tower2'),
        ('3', 'tower3'),
    ],
)
def test_parse_tower_accepts_delivery_tower_aliases(value, expected):
    assert parse_tower(value) == expected


@pytest.mark.parametrize(
    'value', ['pickup', 'safe', 'safezone', 'tower4', '0', '4', '', 'garbage'])
def test_parse_tower_rejects_non_delivery_requests(value):
    assert parse_tower(value) is None


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        ('tower1', 'tower1'),
        ('TOWER_2', 'tower2'),
        (' tower_3 ', 'tower3'),
        ('1', 'tower1'),
        ('2', 'tower2'),
        ('3', 'tower3'),
        ('pickup', 'pickup'),
        ('PICKUP', 'pickup'),
        ('safe', 'safe'),
        ('safezone', 'safe'),
        (' SAFE_ZONE ', 'safe'),
    ],
)
def test_parse_manual_goal_accepts_waypoints_and_aliases(value, expected):
    assert parse_manual_goal(value) == expected


@pytest.mark.parametrize('value', ['tower4', '4', 'home', '', '0.1 0.2'])
def test_parse_manual_goal_rejects_unknown_names(value):
    assert parse_manual_goal(value) is None


def test_enable_false_immediately_prevents_motion_and_clears_arrival():
    interlock = ControlInterlock(enabled=True, arrived=True)
    assert interlock.can_move is True

    interlock.set_enabled(False)

    assert interlock.enabled is False
    assert interlock.can_move is False
    assert interlock.arrived is False


def test_enable_true_allows_motion_when_safety_is_clear():
    interlock = ControlInterlock()
    assert interlock.can_move is False

    interlock.set_enabled(True)

    assert interlock.enabled is True
    assert interlock.can_move is True


def test_safety_stop_latches_and_requires_a_new_enable_true():
    interlock = ControlInterlock(enabled=True, arrived=True)

    interlock.set_safety_stop(True)
    assert interlock.safety_stop is True
    assert interlock.enabled is False
    assert interlock.can_move is False
    assert interlock.arrived is False

    interlock.set_safety_stop(False)
    assert interlock.safety_stop is False
    assert interlock.can_move is False

    interlock.set_enabled(True)
    assert interlock.can_move is True


def test_enable_true_cannot_override_an_active_safety_stop():
    interlock = ControlInterlock(enabled=True)
    interlock.set_safety_stop(True)

    interlock.set_enabled(True)

    assert interlock.can_move is False


def test_repeated_inactive_safety_level_does_not_interrupt_motion_or_arrival():
    interlock = ControlInterlock(enabled=True, arrived=True)

    interlock.set_safety_stop(False)

    assert interlock.enabled is True
    assert interlock.can_move is True
    assert interlock.arrived is True


def test_new_goal_clears_previous_arrival():
    interlock = ControlInterlock(enabled=True, arrived=True)
    interlock.new_goal()
    assert interlock.arrived is False
