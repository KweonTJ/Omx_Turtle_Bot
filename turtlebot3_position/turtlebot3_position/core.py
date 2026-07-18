"""ROS-independent parsing, geometry, interlock, and delivery mode logic."""

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Mapping, Optional, Tuple


_NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
POSITION_RE = re.compile(
    rf'(?<!\w)X\s*=\s*(?P<x>{_NUMBER})\s*,\s*'
    rf'Y\s*=\s*(?P<y>{_NUMBER})(?![\w.])',
    re.IGNORECASE,
)


def parse_position(line):
    """Return a finite ``(x, y)`` pair in metres from ESP32 output."""
    if not isinstance(line, str):
        return None
    match = POSITION_RE.search(line)
    if match is None:
        return None
    try:
        point = (float(match.group('x')), float(match.group('y')))
    except ValueError:
        return None
    return point if all(math.isfinite(value) for value in point) else None


def normalize_angle(angle):
    """Wrap a radian angle into the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x, y, z, w):
    """Return ``(yaw, valid)`` after validating and normalising a quaternion."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return 0.0, False
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6:
        return 0.0, False
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return yaw, math.isfinite(yaw)


def yaw_to_quaternion(yaw):
    """Return an ``(x, y, z, w)`` quaternion for a planar radian yaw."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def target_error(current, target, yaw):
    """Return target distance and wrapped bearing error in metres/radians."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    distance = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    return distance, normalize_angle(bearing - yaw)


def _compact_name(value):
    if not isinstance(value, str):
        return ''
    return value.strip().lower().replace('_', '').replace(' ', '')


def parse_tower(value):
    """Return a canonical tower name for tower1/2/3 and 1/2/3 aliases."""
    compact = _compact_name(value)
    aliases = {'1': 'tower1', '2': 'tower2', '3': 'tower3'}
    if compact in aliases:
        return aliases[compact]
    if compact in aliases.values():
        return compact
    return None


def parse_manual_goal(value):
    """Return the canonical name of a supported manual waypoint."""
    compact = _compact_name(value)
    aliases = {
        'tower1': 'tower1',
        '1': 'tower1',
        'tower2': 'tower2',
        '2': 'tower2',
        'tower3': 'tower3',
        '3': 'tower3',
        'pickup': 'pickup',
        'safe': 'safe',
        'safezone': 'safe',
    }
    return aliases.get(compact)


@dataclass
class ControlInterlock:
    """Track enable, safety latch, and arrival independently of ROS."""

    enabled: bool = False
    safety_stop: bool = False
    arrived: bool = False

    def set_enabled(self, enabled):
        if enabled and self.safety_stop:
            self.enabled = False
        else:
            self.enabled = bool(enabled)
        if not self.enabled:
            self.arrived = False

    def set_safety_stop(self, active):
        active = bool(active)
        was_active = self.safety_stop
        self.safety_stop = active
        if active:
            self.arrived = False
            # Clearing safety later never restores this latch automatically.
            self.enabled = False
        elif was_active:
            # A real true-to-false edge keeps motion disabled and not arrived.
            self.arrived = False

    def new_goal(self):
        self.arrived = False

    @property
    def can_move(self):
        return self.enabled and not self.safety_stop


class DeliveryPhase(Enum):
    """Automatic delivery phases plus manual and emergency-stop modes."""

    IDLE = 'IDLE'
    TO_PICKUP = 'TO_PICKUP'
    WAIT_PICKUP = 'WAIT_PICKUP'
    TO_TOWER = 'TO_TOWER'
    WAIT_DELIVERY = 'WAIT_DELIVERY'
    TO_SAFE = 'TO_SAFE'
    SAFE = 'SAFE'
    MANUAL = 'MANUAL'
    ESTOP = 'ESTOP'


_AUTOMATIC_ACTIVE_PHASES = frozenset({
    DeliveryPhase.TO_PICKUP,
    DeliveryPhase.WAIT_PICKUP,
    DeliveryPhase.TO_TOWER,
    DeliveryPhase.WAIT_DELIVERY,
    DeliveryPhase.TO_SAFE,
})


@dataclass
class DeliveryModeState:
    """Pure automatic/manual delivery state machine used by the ROS node."""

    phase: DeliveryPhase = DeliveryPhase.IDLE
    requested_tower: Optional[str] = None
    phase_before_estop: DeliveryPhase = DeliveryPhase.IDLE

    @property
    def automatic_active(self):
        return self.phase in _AUTOMATIC_ACTIVE_PHASES

    def request_delivery(self, value):
        tower = parse_tower(value)
        if tower is None:
            return False, 'invalid_request'
        if self.phase not in (DeliveryPhase.IDLE, DeliveryPhase.SAFE):
            return False, f'busy:{self.phase.value}'
        self.requested_tower = tower
        self.phase = DeliveryPhase.TO_PICKUP
        return True, f'accepted:{tower}'

    def start_manual(self):
        if self.phase == DeliveryPhase.ESTOP:
            return 'manual_goal_rejected:estop'
        event = (
            'mission_cancelled:manual_goal'
            if self.automatic_active else None
        )
        self.requested_tower = None
        self.phase = DeliveryPhase.MANUAL
        return event

    def arrive(self):
        if self.phase == DeliveryPhase.TO_PICKUP:
            self.phase = DeliveryPhase.WAIT_PICKUP
            return None, 'arrived:pickup'
        if self.phase == DeliveryPhase.TO_TOWER:
            tower = self.requested_tower
            if tower is None:
                return None, None
            self.phase = DeliveryPhase.WAIT_DELIVERY
            return None, f'arrived:{tower}'
        if self.phase == DeliveryPhase.TO_SAFE:
            self.phase = DeliveryPhase.SAFE
            self.requested_tower = None
            return None, 'arrived:safe'
        if self.phase == DeliveryPhase.MANUAL:
            return None, 'manual_arrived:goal'
        return None, None

    def complete_wait(self):
        if self.phase == DeliveryPhase.WAIT_PICKUP:
            if self.requested_tower is None:
                return None, None
            self.phase = DeliveryPhase.TO_TOWER
            return self.requested_tower, 'pickup_wait_complete'
        if self.phase == DeliveryPhase.WAIT_DELIVERY:
            self.phase = DeliveryPhase.TO_SAFE
            return 'safe', 'delivery_wait_complete'
        return None, None

    def enter_estop(self):
        if self.phase == DeliveryPhase.ESTOP:
            return None
        self.phase_before_estop = self.phase
        self.phase = DeliveryPhase.ESTOP
        return 'emergency_stop'

    def resume_after_estop(self):
        if self.phase == DeliveryPhase.ESTOP:
            self.phase = self.phase_before_estop
        return self.phase

    def cancel(self, reason='disable'):
        if self.phase == DeliveryPhase.ESTOP:
            active = self.phase_before_estop not in (
                DeliveryPhase.IDLE, DeliveryPhase.SAFE)
            self.phase_before_estop = DeliveryPhase.IDLE
        else:
            active = self.phase not in (
                DeliveryPhase.IDLE, DeliveryPhase.SAFE)
            self.phase = DeliveryPhase.IDLE
        self.requested_tower = None
        return f'mission_cancelled:{reason}' if active else None


@dataclass(frozen=True)
class ConsoleCommand:
    """A parsed interactive console action."""

    kind: str
    target: Optional[Tuple[float, float]] = None
    yaw: Optional[float] = None
    waypoint: Optional[str] = None


def parse_console_command(value, waypoints: Mapping[str, Tuple[float, float]]):
    """Parse a manual goal, disable, or quit command without ROS."""
    if not isinstance(value, str):
        return None
    text = value.strip()
    compact = text.lower()
    if compact in ('quit', 'exit'):
        return ConsoleCommand(kind='quit')
    if compact in ('disable', 'stop'):
        return ConsoleCommand(kind='disable')

    waypoint = parse_manual_goal(text)
    if waypoint is not None:
        if waypoint not in waypoints:
            return None
        try:
            target = tuple(float(item) for item in waypoints[waypoint])
        except (TypeError, ValueError):
            return None
        if len(target) != 2 or not all(math.isfinite(item) for item in target):
            return None
        return ConsoleCommand(
            kind='goal', target=target, yaw=None, waypoint=waypoint)

    fields = text.replace(',', ' ').split()
    if len(fields) not in (2, 3):
        return None
    try:
        numbers = [float(field) for field in fields]
    except ValueError:
        return None
    if not all(math.isfinite(number) for number in numbers):
        return None
    yaw = math.radians(numbers[2]) if len(numbers) == 3 else None
    return ConsoleCommand(
        kind='goal', target=(numbers[0], numbers[1]), yaw=yaw)
