#!/usr/bin/env python3
"""Coordinate one external-base PICK, transport, and PLACE cycle."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class OmxTurtleNode(Node):
    """Gate arm commands using fresh arrival and perception signals."""

    def __init__(self):
        super().__init__('omx_turtle_node')

        self.declare_parameter(
            'base_arrived_topic', '/turtlebot3_control/base_arrived')
        self.declare_parameter('aruco_visible_topic', '/target/aruco_visible')
        self.declare_parameter('mp_control_status_topic', '/mp_control/status')
        self.declare_parameter('mp_control_start_topic', '/mp_control/start')
        self.declare_parameter('rl_command_topic', '/rl_control/command')
        self.declare_parameter('mux_mode_topic', '/turtlebot3_control/mux_mode')
        self.declare_parameter(
            'status_topic', '/turtlebot3_control/coordinator_status')
        self.declare_parameter('wait_for_base_arrival', True)
        self.declare_parameter('require_aruco_visible', True)
        self.declare_parameter('require_transport_departure', True)
        self.declare_parameter('publish_compat_start', False)
        self.declare_parameter('command_publish_count', 3)
        self.declare_parameter('command_publish_period_s', 0.2)
        self.declare_parameter('aruco_visible_timeout_s', 0.8)
        self.declare_parameter('base_arrival_timeout_s', 2.0)
        self.declare_parameter('grasp_status_keywords', ['grasp complete'])
        self.declare_parameter(
            'place_status_keywords', ['pick place complete'])
        self.declare_parameter('error_status_keywords', [
            'abort',
            'ERROR',
            'failed',
            'FAULT',
            'E_STOP',
        ])

        self.wait_for_base_arrival = bool(
            self.get_parameter('wait_for_base_arrival').value)
        self.require_aruco_visible = bool(
            self.get_parameter('require_aruco_visible').value)
        self.require_transport_departure = bool(
            self.get_parameter('require_transport_departure').value)
        self.publish_compat_start = bool(
            self.get_parameter('publish_compat_start').value)
        self.command_publish_count = int(
            self.get_parameter('command_publish_count').value)
        self.command_publish_period_s = float(
            self.get_parameter('command_publish_period_s').value)
        self.aruco_visible_timeout_s = float(
            self.get_parameter('aruco_visible_timeout_s').value)
        self.base_arrival_timeout_s = float(
            self.get_parameter('base_arrival_timeout_s').value)
        self.grasp_status_keywords = self._string_list(
            'grasp_status_keywords')
        self.place_status_keywords = self._string_list(
            'place_status_keywords')
        self.error_status_keywords = self._string_list(
            'error_status_keywords')

        if self.command_publish_count == 0:
            raise ValueError('command_publish_count must be -1 or positive')
        if self.command_publish_period_s <= 0.0:
            raise ValueError('command_publish_period_s must be positive')

        self.base_arrived = False
        self.base_arrival_time = 0.0
        self.aruco_visible = False
        self.aruco_visible_time = 0.0
        self.mp_status = ''
        self.phase = 'NAVIGATING_PICK'
        self.phase_enter_time = time.monotonic()
        self.command_sent = 0
        self.last_command_time = 0.0
        self.last_status = ''

        self.create_subscription(
            Bool,
            str(self.get_parameter('base_arrived_topic').value),
            self._on_base_arrived,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('aruco_visible_topic').value),
            self._on_aruco_visible,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mp_control_status_topic').value),
            self._on_mp_status,
            10,
        )

        self.compat_start_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('mp_control_start_topic').value),
            10,
        )
        self.command_pub = self.create_publisher(
            String,
            str(self.get_parameter('rl_command_topic').value),
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter('mux_mode_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )

        self.create_timer(0.05, self._on_timer)

    def _string_list(self, parameter):
        return [
            str(word).lower()
            for word in self.get_parameter(parameter).value
        ]

    def _on_base_arrived(self, msg):
        self.base_arrived = bool(msg.data)
        self.base_arrival_time = time.monotonic()

    def _on_aruco_visible(self, msg):
        self.aruco_visible = bool(msg.data)
        if self.aruco_visible:
            self.aruco_visible_time = time.monotonic()

    def _on_mp_status(self, msg):
        self.mp_status = msg.data

    def _on_timer(self):
        now = time.monotonic()
        base_signal_fresh = (
            self.base_arrival_time > 0.0
            and now - self.base_arrival_time <= self.base_arrival_timeout_s
        )
        base_arrival_fresh = self.base_arrived and base_signal_fresh
        aruco_fresh = (
            self.aruco_visible
            and self.aruco_visible_time > 0.0
            and now - self.aruco_visible_time <= self.aruco_visible_timeout_s
        )
        status_lower = self.mp_status.lower()

        if self.phase == 'NAVIGATING_PICK':
            self._publish_mode('NAV')
            arrived = not self.wait_for_base_arrival or base_arrival_fresh
            visible_ok = not self.require_aruco_visible or aruco_fresh
            if arrived and visible_ok:
                self._set_phase('PICKING')

        elif self.phase == 'PICKING':
            self._publish_mode('PICK')
            self._maybe_publish_command('PICK', now)
            if self._matches(status_lower, self.error_status_keywords):
                self._set_phase('ERROR')
            elif self._matches(status_lower, self.grasp_status_keywords):
                next_phase = (
                    'WAIT_TRANSPORT_DEPARTURE'
                    if self.require_transport_departure
                    else 'TRANSPORTING'
                )
                self._set_phase(next_phase)

        elif self.phase == 'WAIT_TRANSPORT_DEPARTURE':
            self._publish_mode('NAV')
            departed_after_pick = (
                base_signal_fresh
                and not self.base_arrived
                and self.base_arrival_time >= self.phase_enter_time
            )
            if departed_after_pick:
                self._set_phase('TRANSPORTING')

        elif self.phase == 'TRANSPORTING':
            self._publish_mode('NAV')
            arrived_after_transport = (
                base_arrival_fresh
                and self.base_arrival_time >= self.phase_enter_time
            )
            if arrived_after_transport:
                self._set_phase('PLACING')

        elif self.phase == 'PLACING':
            self._publish_mode('PICK')
            self._maybe_publish_command('PLACE', now)
            if self._matches(status_lower, self.error_status_keywords):
                self._set_phase('ERROR')
            elif self._matches(status_lower, self.place_status_keywords):
                self._set_phase('DONE')

        elif self.phase in ('DONE', 'ERROR'):
            self._publish_mode('HOLD')

        self._publish_status(
            base_signal_fresh=base_signal_fresh,
            base_arrival_fresh=base_arrival_fresh,
            aruco_fresh=aruco_fresh,
        )

    @staticmethod
    def _matches(status, keywords):
        return any(word in status for word in keywords)

    def _set_phase(self, phase):
        if self.phase == phase:
            return
        self.phase = phase
        self.phase_enter_time = time.monotonic()
        self.command_sent = 0
        self.last_command_time = 0.0
        self._publish_status(force=True)

    def _maybe_publish_command(self, command, now):
        if (
            self.command_publish_count >= 0
            and self.command_sent >= self.command_publish_count
        ):
            return
        if (
            self.command_sent > 0
            and now - self.last_command_time < self.command_publish_period_s
        ):
            return
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        if command == 'PICK' and self.publish_compat_start:
            compat = Bool()
            compat.data = True
            self.compat_start_pub.publish(compat)
        self.command_sent += 1
        self.last_command_time = now

    def _publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def _publish_status(
            self, base_signal_fresh=None, base_arrival_fresh=None,
            aruco_fresh=None, force=False):
        text = (
            f'phase={self.phase} base_arrived={self.base_arrived} '
            f'base_signal_fresh={base_signal_fresh} '
            f'base_arrival_fresh={base_arrival_fresh} '
            f'aruco_visible={self.aruco_visible} '
            f'aruco_fresh={aruco_fresh} command_sent={self.command_sent}'
        )
        if not force and text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmxTurtleNode()
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
