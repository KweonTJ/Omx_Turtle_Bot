"""Stop-and-go UWB position controller with automatic and manual modes."""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import (
    ControlInterlock,
    DeliveryModeState,
    DeliveryPhase,
    normalize_angle,
    quaternion_to_yaw,
    target_error,
)


# Compatibility-friendly public name for users of the reference controller.
Phase = DeliveryPhase


class PositionControllerNode(Node):
    """Drive short pulses toward UWB goals while enforcing safety latches."""

    WAYPOINT_NAMES = ('pickup', 'tower1', 'tower2', 'tower3', 'safe')

    def __init__(self):
        super().__init__('position_controller_node')
        defaults = {
            'frame_id': 'uwb_map',
            'pose_topic': '/turtlebot3_position/pose',
            'odom_topic': '/odom',
            'goal_topic': '/turtlebot3_position/goal',
            'enable_topic': '/turtlebot3_position/enable',
            'safety_stop_topic': '/safety_stop',
            'status_topic': '/turtlebot3_position/status',
            'nav_cmd_vel_topic': '/turtlebot3_control/nav_cmd_vel',
            'base_arrived_topic': '/turtlebot3_control/base_arrived',
            'delivery_request_topic': '/delivery/request',
            'delivery_event_topic': '/delivery/event',
            'arrival_tolerance': 0.13,
            'arrival_confirmations': 3,
            'linear_speed': 0.055,
            'angular_speed': 0.28,
            'heading_tolerance': 0.30,
            'final_yaw_tolerance': 0.20,
            'drive_pulse_sec': 0.45,
            'turn_pulse_sec': 0.35,
            'settle_sec': 1.6,
            'uwb_timeout_sec': 2.0,
            'odom_timeout_sec': 1.0,
            'pickup_wait_sec': 2.0,
            'delivery_wait_sec': 2.0,
            'initial_yaw': 0.0,
            'use_odom_yaw': True,
            'use_goal_yaw': False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        for waypoint in self.WAYPOINT_NAMES:
            self.declare_parameter(f'{waypoint}_x', math.nan)
            self.declare_parameter(f'{waypoint}_y', math.nan)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._arrival_tolerance = max(
            0.0, float(self.get_parameter('arrival_tolerance').value))
        self._arrival_confirmations = max(
            1, int(self.get_parameter('arrival_confirmations').value))
        self._linear_speed = abs(
            float(self.get_parameter('linear_speed').value))
        self._angular_speed = abs(
            float(self.get_parameter('angular_speed').value))
        self._heading_tolerance = max(
            0.0, float(self.get_parameter('heading_tolerance').value))
        self._final_yaw_tolerance = max(
            0.0, float(self.get_parameter('final_yaw_tolerance').value))
        self._drive_pulse_sec = max(
            0.0, float(self.get_parameter('drive_pulse_sec').value))
        self._turn_pulse_sec = max(
            0.0, float(self.get_parameter('turn_pulse_sec').value))
        self._settle_sec = max(
            0.0, float(self.get_parameter('settle_sec').value))
        self._uwb_timeout_sec = max(
            0.0, float(self.get_parameter('uwb_timeout_sec').value))
        self._odom_timeout_sec = max(
            0.0, float(self.get_parameter('odom_timeout_sec').value))
        self._pickup_wait_sec = max(
            0.0, float(self.get_parameter('pickup_wait_sec').value))
        self._delivery_wait_sec = max(
            0.0, float(self.get_parameter('delivery_wait_sec').value))
        self._use_odom_yaw = bool(
            self.get_parameter('use_odom_yaw').value)
        self._use_goal_yaw = bool(
            self.get_parameter('use_goal_yaw').value)

        self.waypoints = {}
        for name in self.WAYPOINT_NAMES:
            point = (
                float(self.get_parameter(f'{name}_x').value),
                float(self.get_parameter(f'{name}_y').value),
            )
            if not all(math.isfinite(value) for value in point):
                raise RuntimeError(
                    f'Waypoint {name!r} is missing or non-finite; pass '
                    'config/position.yaml to position_controller_node')
            self.waypoints[name] = point

        self.cmd_pub = self.create_publisher(
            Twist, self._topic('nav_cmd_vel_topic'), 5)
        self.arrived_pub = self.create_publisher(
            Bool, self._topic('base_arrived_topic'), 5)
        self.status_pub = self.create_publisher(
            String, self._topic('status_topic'), 5)
        self.event_pub = self.create_publisher(
            String, self._topic('delivery_event_topic'), 5)

        self.create_subscription(
            PoseWithCovarianceStamped, self._topic('pose_topic'),
            self._on_position, 5)
        self.create_subscription(
            Odometry, self._topic('odom_topic'), self._on_odom, 5)
        self.create_subscription(
            PoseStamped, self._topic('goal_topic'), self._on_goal, 5)
        self.create_subscription(
            Bool, self._topic('enable_topic'), self._on_enable, 5)
        self.create_subscription(
            Bool, self._topic('safety_stop_topic'), self._on_safety, 5)
        self.create_subscription(
            String, self._topic('delivery_request_topic'),
            self._on_request, 5)

        self.mode = DeliveryModeState()
        self.interlock = ControlInterlock()
        self.position = None
        self.position_time = None
        self._position_sequence = 0
        self._last_arrival_sequence = -1
        self.odom_time = None
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.goal_target = None
        self.goal_yaw = None
        self.motion_until = None
        self.settle_until = None
        self.wait_until = None
        self.last_motion = (0.0, 0.0)
        self.arrival_count = 0
        self.last_tick = self.get_clock().now()
        self.control_state = 'DISABLED'
        self.detail = 'ENABLE_FALSE'

        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._publish_arrived)
        self.create_timer(0.5, self._publish_status)

    @property
    def phase(self):
        return self.mode.phase

    @property
    def requested_tower(self):
        return self.mode.requested_tower

    def _topic(self, name):
        return str(self.get_parameter(name).value)

    def _publish_event(self, event):
        if event:
            self.event_pub.publish(String(data=event))

    def _on_position(self, msg):
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if not all(math.isfinite(value) for value in point):
            self.get_logger().warning('Ignored non-finite UWB pose')
            return
        self.position = point
        self.position_time = self.get_clock().now()
        self._position_sequence += 1

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw, valid = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if not valid:
            self.get_logger().warning('Ignored invalid odometry quaternion')
            return
        self.odom_time = self.get_clock().now()
        if self._use_odom_yaw:
            self.yaw = yaw

    def _on_goal(self, msg):
        self.interlock.new_goal()
        self._publish_arrived()
        self._stop()

        if self.interlock.safety_stop or self.phase == DeliveryPhase.ESTOP:
            self._publish_event('manual_goal_rejected:safety_stop')
            self.get_logger().warning(
                'Manual goal ignored until safety is cleared and re-enabled')
            return

        target = (float(msg.pose.position.x), float(msg.pose.position.y))
        if not all(math.isfinite(value) for value in target):
            self._publish_event('manual_goal_rejected:invalid_coordinate')
            self.get_logger().warning('Manual goal has a non-finite coordinate')
            return

        expected_frame = self._frame_id
        if msg.header.frame_id and msg.header.frame_id != expected_frame:
            self.get_logger().warning(
                f'Goal frame {msg.header.frame_id!r} is interpreted as '
                f'{expected_frame!r}')

        goal_yaw = None
        if self._use_goal_yaw:
            q = msg.pose.orientation
            yaw, valid = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            if valid:
                goal_yaw = yaw
            else:
                self.get_logger().warning(
                    'Goal orientation is invalid; final alignment is skipped')

        cancellation_event = self.mode.start_manual()
        self._reset_motion_state(clear_wait=True)
        self._set_goal(target, goal_yaw)
        self._publish_event(cancellation_event)

    def _on_request(self, msg):
        # The delivery topic deliberately accepts canonical values only.
        request = msg.data.strip()
        if request not in ('tower1', 'tower2', 'tower3'):
            self._publish_event('request_rejected:invalid_tower')
            self.get_logger().warning(
                'Invalid delivery request; use tower1, tower2, or tower3')
            return
        if self.interlock.safety_stop or self.phase == DeliveryPhase.ESTOP:
            self._publish_event('request_rejected:safety_stop')
            self.get_logger().warning(
                'Delivery request ignored while safety stop is latched')
            return

        accepted, event = self.mode.request_delivery(request)
        if not accepted:
            self._publish_event(f'request_rejected:{event}')
            self.get_logger().warning(
                f'Delivery request ignored while {self.phase.value} is active')
            return

        self._stop()
        self._reset_motion_state(clear_wait=True)
        self._set_goal(self.waypoints['pickup'])
        self._publish_event(event)

    def _on_enable(self, msg):
        if not msg.data:
            self.interlock.set_enabled(False)
            event = self.mode.cancel('disable')
            self._clear_goal_and_timers()
            self._stop()
            self.control_state = (
                'SAFETY_STOP' if self.interlock.safety_stop else 'DISABLED')
            self.detail = (
                'LATCHED' if self.interlock.safety_stop else 'ENABLE_FALSE')
            self._publish_event(event)
            self._publish_arrived()
            return

        self.interlock.set_enabled(True)
        if not self.interlock.can_move:
            self._stop()
            self.control_state = 'SAFETY_STOP'
            self.detail = 'LATCHED'
            self._publish_arrived()
            return

        resumed = self.mode.resume_after_estop()
        if resumed == DeliveryPhase.WAIT_PICKUP:
            self.interlock.arrived = True
            self.wait_until = self.get_clock().now() + Duration(
                seconds=self._pickup_wait_sec)
        elif resumed == DeliveryPhase.WAIT_DELIVERY:
            self.interlock.arrived = True
            self.wait_until = self.get_clock().now() + Duration(
                seconds=self._delivery_wait_sec)
        self.control_state = 'IDLE'
        self.detail = 'ENABLED'
        self._publish_arrived()

    def _on_safety(self, msg):
        if msg.data:
            event = self.mode.enter_estop()
            self.interlock.set_safety_stop(True)
            self._reset_motion_state(clear_wait=True)
            self._stop()
            self.control_state = 'SAFETY_STOP'
            self.detail = 'ACTIVE'
            self._publish_event(event)
        else:
            if not self.interlock.safety_stop:
                # A level-triggered safety publisher may repeat false forever.
                # Only a real true-to-false edge may disturb motion/arrival.
                return
            self.interlock.set_safety_stop(False)
            self._stop()
            self.control_state = 'DISABLED'
            self.detail = 'SAFETY_CLEARED_REENABLE_REQUIRED'
        self._publish_arrived()

    def _set_goal(self, target, yaw=None):
        self.goal_target = (float(target[0]), float(target[1]))
        self.goal_yaw = yaw
        self._reset_motion_state(clear_wait=True)
        self.interlock.new_goal()
        self._publish_arrived()

    def _reset_motion_state(self, clear_wait=False):
        self.motion_until = None
        self.settle_until = None
        if clear_wait:
            self.wait_until = None
        self.arrival_count = 0
        self._last_arrival_sequence = -1
        self.last_motion = (0.0, 0.0)

    def _clear_goal_and_timers(self):
        self.goal_target = None
        self.goal_yaw = None
        self._reset_motion_state(clear_wait=True)
        self.interlock.new_goal()

    @staticmethod
    def _age(stamp, now):
        if stamp is None:
            return math.inf
        return (now - stamp).nanoseconds / 1e9

    def _sensor_fault(self, now):
        if self.position_time is None:
            return 'WAIT_SENSOR', 'NO_UWB'
        if self._age(self.position_time, now) > self._uwb_timeout_sec:
            return 'FAULT', 'UWB_TIMEOUT'
        if self._use_odom_yaw:
            if self.odom_time is None:
                return 'WAIT_SENSOR', 'NO_ODOM'
            if self._age(self.odom_time, now) > self._odom_timeout_sec:
                return 'FAULT', 'ODOM_TIMEOUT'
        return None

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        if not self._use_odom_yaw:
            self.yaw = normalize_angle(
                self.yaw + self.last_motion[1] * max(0.0, dt))

        if self.interlock.safety_stop:
            self.interlock.arrived = False
            self.control_state, self.detail = 'SAFETY_STOP', 'ACTIVE'
            self._stop()
            return
        if not self.interlock.enabled:
            self.interlock.arrived = False
            self.control_state, self.detail = 'DISABLED', 'ENABLE_FALSE'
            self._stop()
            return
        if self.goal_target is None:
            self.interlock.arrived = False
            self.control_state, self.detail = 'IDLE', 'NO_GOAL'
            self._stop()
            return

        fault = self._sensor_fault(now)
        if fault is not None:
            self.interlock.arrived = False
            self.control_state, self.detail = fault
            self._stop()
            return

        if self.phase in (
                DeliveryPhase.WAIT_PICKUP, DeliveryPhase.WAIT_DELIVERY):
            self._tick_delivery_wait(now)
            return
        if self.interlock.arrived:
            self.control_state, self.detail = 'ARRIVED', self.phase.value
            self._stop()
            return

        if self.motion_until is not None:
            if now < self.motion_until:
                self._publish_twist(*self.last_motion)
                return
            self.motion_until = None
            self._stop()
        if self.settle_until is not None:
            if now < self.settle_until:
                self._stop()
                return
            self.settle_until = None

        distance, angle = target_error(
            self.position, self.goal_target, self.yaw)
        if distance <= self._arrival_tolerance:
            self._confirm_arrival(now)
            return

        self.arrival_count = 0
        self._last_arrival_sequence = -1
        if abs(angle) > self._heading_tolerance:
            speed = math.copysign(self._angular_speed, angle)
            self.control_state = 'ROTATE_TO_GOAL'
            self.detail = f'ANGLE_ERROR={angle:.3f}'
            self._start_motion(now, 0.0, speed, self._turn_pulse_sec)
        else:
            self.control_state = 'DRIVE'
            self.detail = f'DISTANCE={distance:.3f}'
            self._start_motion(now, self._linear_speed, 0.0,
                               self._drive_pulse_sec)

    def _confirm_arrival(self, now):
        if self._position_sequence != self._last_arrival_sequence:
            self.arrival_count += 1
            self._last_arrival_sequence = self._position_sequence

        if self.arrival_count < self._arrival_confirmations:
            self.control_state = 'DRIVE'
            self.detail = (
                f'ARRIVAL_CONFIRM={self.arrival_count}/'
                f'{self._arrival_confirmations}')
            self._stop()
            self.settle_until = now + Duration(seconds=0.45)
            return

        if self.goal_yaw is not None:
            final_error = normalize_angle(self.goal_yaw - self.yaw)
            if abs(final_error) > self._final_yaw_tolerance:
                speed = math.copysign(self._angular_speed, final_error)
                self.control_state = 'FINAL_ALIGN'
                self.detail = f'YAW_ERROR={final_error:.3f}'
                self._start_motion(
                    now, 0.0, speed, self._turn_pulse_sec)
                return
        self._arrived(now)

    def _tick_delivery_wait(self, now):
        self._stop()
        self.control_state, self.detail = 'ARRIVED', self.phase.value
        if self.wait_until is None:
            delay = (
                self._pickup_wait_sec
                if self.phase == DeliveryPhase.WAIT_PICKUP
                else self._delivery_wait_sec)
            self.wait_until = now + Duration(seconds=delay)
        if now < self.wait_until:
            return

        next_waypoint, event = self.mode.complete_wait()
        if next_waypoint is None:
            self.control_state, self.detail = 'FAULT', 'INVALID_DELIVERY_STATE'
            self._clear_goal_and_timers()
            return
        self._set_goal(self.waypoints[next_waypoint])
        self._publish_event(event)

    def _start_motion(self, now, linear, angular, duration):
        self._publish_twist(linear, angular)
        self.motion_until = now + Duration(seconds=duration)
        self.settle_until = self.motion_until + Duration(
            seconds=self._settle_sec)

    def _arrived(self, now):
        self._stop()
        self.interlock.arrived = True
        _, event = self.mode.arrive()
        self.control_state, self.detail = 'ARRIVED', self.phase.value
        if self.phase == DeliveryPhase.WAIT_PICKUP:
            self.wait_until = now + Duration(seconds=self._pickup_wait_sec)
        elif self.phase == DeliveryPhase.WAIT_DELIVERY:
            self.wait_until = now + Duration(
                seconds=self._delivery_wait_sec)
        self._publish_event(event)
        self._publish_arrived()

    def _publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
        self.last_motion = (float(linear), float(angular))

    def _stop(self):
        self._publish_twist(0.0, 0.0)

    def _arrival_output(self):
        if not self.interlock.can_move or not self.interlock.arrived:
            return False
        if self.goal_target is None:
            return False
        return self._sensor_fault(self.get_clock().now()) is None

    def _publish_arrived(self):
        self.arrived_pub.publish(Bool(data=self._arrival_output()))

    def _publish_status(self):
        now = self.get_clock().now()
        status = self.control_state
        if self.detail:
            status += ':' + self.detail
        self.status_pub.publish(String(
            data=(
                f'{status} phase={self.phase.value} '
                f'uwb_age={self._age(self.position_time, now):.2f}s '
                f'odom_age={self._age(self.odom_time, now):.2f}s'
            )
        ))

    def destroy_node(self):
        self.interlock.set_enabled(False)
        self.interlock.arrived = False
        self._publish_arrived()
        self._stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
