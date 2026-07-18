"""Interactive console for one-shot manual UWB goals."""

import math
from pathlib import Path
import threading

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

from .core import parse_console_command, yaw_to_quaternion


def _position_config_path():
    """Find the installed YAML, with a source-tree fallback for development."""
    try:
        share = Path(get_package_share_directory('turtlebot3_position'))
        return share / 'config' / 'position.yaml'
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / 'config' / 'position.yaml'


def load_console_parameter_defaults(path=None):
    """Load the console's shared waypoint defaults from position.yaml."""
    config_path = Path(path) if path is not None else _position_config_path()
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    try:
        parameters = config['goal_console']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f'goal_console parameters are missing from {config_path}') from exc
    return dict(parameters)


class GoalConsole(Node):
    """Convert YAML waypoint names or x/y/yaw input to PoseStamped."""

    WAYPOINT_NAMES = ('pickup', 'tower1', 'tower2', 'tower3', 'safe')

    def __init__(self):
        super().__init__('goal_console')
        defaults = load_console_parameter_defaults()
        required = {
            'goal_topic', 'enable_topic', 'frame_id', 'auto_enable',
            *(f'{name}_{axis}' for name in self.WAYPOINT_NAMES
              for axis in ('x', 'y')),
        }
        missing = sorted(required.difference(defaults))
        if missing:
            raise RuntimeError(
                'position.yaml is missing goal_console parameters: '
                + ', '.join(missing))
        for name in sorted(required):
            self.declare_parameter(name, defaults[name])

        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), 5)
        self.enable_pub = self.create_publisher(
            Bool, str(self.get_parameter('enable_topic').value), 5)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._auto_enable = bool(self.get_parameter('auto_enable').value)
        self._waypoints = {
            name: (
                float(self.get_parameter(f'{name}_x').value),
                float(self.get_parameter(f'{name}_y').value),
            )
            for name in self.WAYPOINT_NAMES
        }

        self._thread = threading.Thread(
            target=self._input_loop, name='uwb_goal_console', daemon=True)
        self._thread.start()

    def _publish_goal(self, target, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        if yaw is not None:
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        # The ROS default all-zero quaternion means "no final yaw requested".
        self.goal_pub.publish(msg)
        if self._auto_enable:
            self.enable_pub.publish(Bool(data=True))

    def _input_loop(self):
        print('\n목적지: tower1/2/3, pickup, safe 또는 safezone')
        print('직접 좌표: x y [yaw_deg]   정지: disable/stop   종료: quit/exit\n')
        while rclpy.ok():
            try:
                value = input('UWB goal> ').strip()
            except (EOFError, KeyboardInterrupt):
                rclpy.try_shutdown()
                return

            command = parse_console_command(value, self._waypoints)
            if command is None:
                print('잘못된 입력입니다. waypoint 또는 x y [yaw_deg]를 입력하세요.')
                continue
            if command.kind == 'quit':
                rclpy.try_shutdown()
                return
            if command.kind == 'disable':
                self.enable_pub.publish(Bool(data=False))
                print('이동 비활성화 전송')
                continue

            self._publish_goal(command.target, command.yaw)
            suffix = (
                '' if command.yaw is None
                else f', yaw={math.degrees(command.yaw):.1f}deg')
            print(
                f'목적지 전송: x={command.target[0]:.3f}, '
                f'y={command.target[1]:.3f}{suffix}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalConsole()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
