"""Small unittest contract suite discoverable by ``colcon test``."""

import math
from pathlib import Path
import unittest

from turtlebot3_position.core import (
    ControlInterlock,
    DeliveryModeState,
    DeliveryPhase,
    parse_console_command,
    parse_position,
)
import yaml


ROOT = Path(__file__).resolve().parents[1]


class ColconContractTest(unittest.TestCase):

    def test_uwb_parser_extracts_final_xy_and_rejects_nonfinite(self):
        line = (
            '01=1.2, 02=1.4, 03=1.0, 04=1.6, '
            'X=0.875, Y=0.565')
        self.assertEqual(parse_position(line), (0.875, 0.565))
        for invalid in ('X=?, Y=?', 'X=nan, Y=0.5', 'X=inf, Y=0.5'):
            with self.subTest(invalid=invalid):
                self.assertIsNone(parse_position(invalid))

    def test_disable_and_safety_latch_clear_arrival(self):
        interlock = ControlInterlock(enabled=True, arrived=True)
        interlock.set_enabled(False)
        self.assertFalse(interlock.can_move)
        self.assertFalse(interlock.arrived)

        interlock.set_enabled(True)
        interlock.arrived = True
        interlock.set_safety_stop(True)
        self.assertFalse(interlock.can_move)
        self.assertFalse(interlock.arrived)
        interlock.set_safety_stop(False)
        self.assertFalse(interlock.can_move)
        interlock.set_enabled(True)
        self.assertTrue(interlock.can_move)

    def test_new_goal_clears_arrival(self):
        interlock = ControlInterlock(enabled=True, arrived=True)
        interlock.new_goal()
        self.assertFalse(interlock.arrived)

    def test_automatic_delivery_sequence(self):
        mode = DeliveryModeState()
        self.assertEqual(
            mode.request_delivery('tower2'), (True, 'accepted:tower2'))
        self.assertIs(mode.phase, DeliveryPhase.TO_PICKUP)
        self.assertEqual(mode.arrive(), (None, 'arrived:pickup'))
        self.assertIs(mode.phase, DeliveryPhase.WAIT_PICKUP)
        self.assertEqual(
            mode.complete_wait(), ('tower2', 'pickup_wait_complete'))
        self.assertEqual(mode.arrive(), (None, 'arrived:tower2'))
        self.assertEqual(
            mode.complete_wait(), ('safe', 'delivery_wait_complete'))
        self.assertEqual(mode.arrive(), (None, 'arrived:safe'))
        self.assertIs(mode.phase, DeliveryPhase.SAFE)

    def test_manual_goal_cancels_delivery_without_follow_on_target(self):
        mode = DeliveryModeState()
        mode.request_delivery('tower1')
        self.assertEqual(
            mode.start_manual(), 'mission_cancelled:manual_goal')
        self.assertIs(mode.phase, DeliveryPhase.MANUAL)
        self.assertIsNone(mode.requested_tower)
        self.assertEqual(mode.arrive(), (None, 'manual_arrived:goal'))
        self.assertEqual(mode.complete_wait(), (None, None))

    def test_console_named_and_direct_goals(self):
        waypoints = {
            'pickup': (0.80, 0.93),
            'tower1': (0.20, 0.20),
            'tower2': (1.55, 0.93),
            'tower3': (1.55, 0.90),
            'safe': (0.80, 0.20),
        }
        self.assertEqual(
            parse_console_command('safezone', waypoints).target,
            waypoints['safe'],
        )
        direct = parse_console_command('1.0 2.0 90', waypoints)
        self.assertEqual(direct.target, (1.0, 2.0))
        self.assertTrue(math.isclose(direct.yaw, math.pi / 2.0))
        self.assertEqual(
            parse_console_command('stop', waypoints).kind, 'disable')

    def test_project_topic_parameters_and_waypoint_consistency(self):
        config = yaml.safe_load(
            (ROOT / 'config' / 'position.yaml').read_text(encoding='utf-8'))
        controller = config['position_controller_node']['ros__parameters']
        console = config['goal_console']['ros__parameters']
        self.assertEqual(
            controller['nav_cmd_vel_topic'],
            '/turtlebot3_control/nav_cmd_vel',
        )
        self.assertEqual(
            controller['base_arrived_topic'],
            '/turtlebot3_control/base_arrived',
        )
        self.assertEqual(controller['goal_topic'], '/turtlebot3_position/goal')
        for waypoint in ('pickup', 'tower1', 'tower2', 'tower3', 'safe'):
            for axis in ('x', 'y'):
                name = f'{waypoint}_{axis}'
                self.assertEqual(controller[name], console[name])

    def test_no_direct_cmd_vel_literal_in_nodes_or_launch(self):
        paths = list((ROOT / 'turtlebot3_position').glob('*.py'))
        paths.extend((ROOT / 'launch').glob('*.py'))
        for path in paths:
            source = path.read_text(encoding='utf-8')
            with self.subTest(path=path.name):
                self.assertNotIn("'/cmd_vel'", source)
                self.assertNotIn('"/cmd_vel"', source)


if __name__ == '__main__':
    unittest.main()
