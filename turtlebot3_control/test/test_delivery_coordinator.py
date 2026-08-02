"""State-transition tests for the delivery coordinator."""

import time
import unittest

from turtlebot3_control.omx_turtle_node import OmxTurtleNode


class CoordinatorHarness(OmxTurtleNode):
    """Use coordinator logic without creating ROS publishers or timers."""

    def __init__(self):
        self.wait_for_base_arrival = True
        self.require_aruco_visible = True
        self.require_transport_departure = True
        self.base_arrival_timeout_s = 2.0
        self.aruco_visible_timeout_s = 0.8
        self.grasp_status_keywords = ['grasp complete']
        self.place_status_keywords = ['pick place complete']
        self.error_status_keywords = ['error', 'fault', 'e_stop']
        self.base_arrived = True
        self.base_arrival_time = time.monotonic()
        self.aruco_visible = True
        self.aruco_visible_time = time.monotonic()
        self.mp_status = ''
        self.phase = 'NAVIGATING_PICK'
        self.phase_enter_time = time.monotonic()
        self.command_sent = 0
        self.last_command_time = 0.0
        self.last_status = ''
        self.commands = []
        self.modes = []

    def _publish_mode(self, mode):
        self.modes.append(mode)

    def _maybe_publish_command(self, command, now):
        self.commands.append(command)

    def _publish_status(self, **kwargs):
        del kwargs


class TestDeliveryCoordinator(unittest.TestCase):

    def test_requires_departure_before_delivery_arrival(self):
        node = CoordinatorHarness()

        node._on_timer()
        self.assertEqual(node.phase, 'PICKING')
        node.mp_status = 'grasp complete'
        node._on_timer()
        self.assertEqual(node.phase, 'WAIT_TRANSPORT_DEPARTURE')

        node.base_arrived = False
        node.base_arrival_time = time.monotonic()
        node._on_timer()
        self.assertEqual(node.phase, 'TRANSPORTING')

        node.base_arrived = True
        node.base_arrival_time = time.monotonic()
        node._on_timer()
        self.assertEqual(node.phase, 'PLACING')

        node.mp_status = 'pick place complete'
        node._on_timer()
        self.assertEqual(node.phase, 'DONE')
        self.assertIn('PICK', node.commands)
        self.assertIn('PLACE', node.commands)


if __name__ == '__main__':
    unittest.main()
