#!/usr/bin/env python3
# flake8: noqa
"""Run repeatable ROS 2 Pick-Place cycles against the Gazebo RL scene."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import re
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


STATE_PATTERN = re.compile(r'^state=([A-Z_]+)\b')
TERMINAL_FAILURES = {'FAULT', 'HOLD', 'E_STOP'}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', required=True)
    parser.add_argument('--trials', type=int, default=50)
    parser.add_argument('--seed', type=int, default=20260717)
    parser.add_argument('--sample-offset', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trial-timeout', type=float, default=45.0)
    parser.add_argument('--ready-timeout', type=float, default=25.0)
    parser.add_argument('--object-x', type=float, default=0.27)
    parser.add_argument('--object-y', type=float, default=0.0)
    parser.add_argument('--object-z', type=float, default=0.1975)
    parser.add_argument('--target-z', type=float, default=0.1815)
    parser.add_argument('--position-range', type=float, default=0.01)
    parser.add_argument('--yaw-range', type=float, default=0.3)
    # Conservative center margin for a 6 x 5.5 cm box on a 13 cm tower.
    parser.add_argument('--final-xy-tolerance', type=float, default=0.025)
    parser.add_argument('--final-z-tolerance', type=float, default=0.012)
    return parser.parse_args()


class GazeboCycleRunner(Node):
    """Publish commands and collect state/physics results for each cycle."""

    def __init__(self, args: argparse.Namespace):
        super().__init__('gazebo_cycle_runner')
        self.args = args
        self.state = ''
        self.status_text = ''
        self.box_position = None
        self.target_xyz = np.array(
            [args.object_x, args.object_y, args.target_z], dtype=np.float64)
        self.target_yaw = 0.0
        self.command_pub = self.create_publisher(
            String, '/rl_control/command', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/target/object_pose', 10)
        self.valid_pub = self.create_publisher(Bool, '/target/valid', 10)
        self.create_subscription(
            String, '/rl_control/status', self._on_status, 20)
        self.create_subscription(
            TFMessage, '/world/default/pose/info', self._on_world_pose, 20)
        self.pose_client = self.create_client(
            SetEntityPose, '/world/default/set_pose')
        self.target_timer = self.create_timer(0.05, self._publish_target)

    def _on_status(self, message: String) -> None:
        self.status_text = message.data
        match = STATE_PATTERN.match(message.data)
        if match:
            self.state = match.group(1)

    def _on_world_pose(self, message: TFMessage) -> None:
        for transform in message.transforms:
            name = transform.child_frame_id
            if name == 'delivery_box' or name.endswith('/delivery_box'):
                translation = transform.transform.translation
                self.box_position = np.array(
                    [translation.x, translation.y, translation.z],
                    dtype=np.float64,
                )

    def _publish_target(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(self.target_xyz[0])
        pose.pose.position.y = float(self.target_xyz[1])
        pose.pose.position.z = float(self.target_xyz[2])
        pose.pose.orientation.z = math.sin(0.5 * self.target_yaw)
        pose.pose.orientation.w = math.cos(0.5 * self.target_yaw)
        self.pose_pub.publish(pose)
        valid = Bool()
        valid.data = True
        self.valid_pub.publish(valid)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, states: set[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state in states:
                return self.state
        return self.state or 'NO_STATUS'

    def command(self, value: str) -> None:
        message = String()
        message.data = value
        self.command_pub.publish(message)
        self.spin_for(0.15)

    def set_box_pose(self, x: float, y: float, yaw: float) -> bool:
        if not self.pose_client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetEntityPose.Request()
        request.entity.name = 'delivery_box'
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = self.args.object_z
        request.pose.orientation.z = math.sin(0.5 * yaw)
        request.pose.orientation.w = math.cos(0.5 * yaw)
        future = self.pose_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(future.done() and future.result().success)

    def physics_success(self) -> tuple[bool | None, float, float]:
        if self.box_position is None:
            return None, math.nan, math.nan
        xy_error = float(np.linalg.norm(
            self.box_position[:2]
            - np.array([self.args.object_x, self.args.object_y])
        ))
        z_error = abs(float(self.box_position[2]) - self.args.object_z)
        success = bool(
            xy_error <= self.args.final_xy_tolerance
            and z_error <= self.args.final_z_tolerance
        )
        return success, xy_error, z_error


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def recover(runner: GazeboCycleRunner, timeout: float) -> bool:
    runner.command('RESET')
    state = runner.wait_for_state({'STAY_EMPTY', 'WAIT_DELIVERY'}, timeout)
    if state == 'WAIT_DELIVERY':
        runner.command('PLACE')
        runner.wait_for_state({'COMPLETE'} | TERMINAL_FAILURES, timeout)
        runner.command('RESET')
        state = runner.wait_for_state({'STAY_EMPTY'}, timeout)
    return state == 'STAY_EMPTY'


def main() -> None:
    args = parse_args()
    if args.trials < 1:
        raise ValueError('trials must be positive')
    if args.sample_offset < 0:
        raise ValueError('sample-offset must be non-negative')
    rclpy.init()
    runner = GazeboCycleRunner(args)
    rows: list[dict[str, object]] = []
    rng = np.random.default_rng(args.seed)
    samples = [
        (
            int(rng.integers(0, 2**31 - 1)),
            float(rng.uniform(-args.position_range, args.position_range)),
            float(rng.uniform(-args.position_range, args.position_range)),
            float(rng.uniform(-args.yaw_range, args.yaw_range)),
        )
        for _ in range(args.trials + args.sample_offset)
    ][args.sample_offset:]

    try:
        initial = runner.wait_for_state(
            {'STAY_EMPTY'} | TERMINAL_FAILURES, args.ready_timeout)
        if initial != 'STAY_EMPTY':
            raise RuntimeError(
                f'controller did not become ready: state={initial} '
                f'status={runner.status_text}')

        for trial, (seed, dx, dy, yaw) in enumerate(
            samples,
            start=args.sample_offset + 1,
        ):
            x = args.object_x + dx
            y = args.object_y + dy
            runner.target_xyz = np.array(
                [x, y, args.target_z], dtype=np.float64)
            runner.target_yaw = yaw
            set_pose_success = runner.set_box_pose(x, y, yaw)
            runner.spin_for(1.2)

            started = time.monotonic()
            runner.command('PICK')
            pick_state = runner.wait_for_state(
                {'WAIT_DELIVERY'} | TERMINAL_FAILURES,
                args.trial_timeout,
            )
            pick_finished = time.monotonic()
            place_state = 'NOT_STARTED'
            if pick_state == 'WAIT_DELIVERY':
                runner.command('PLACE')
                place_state = runner.wait_for_state(
                    {'COMPLETE'} | TERMINAL_FAILURES,
                    args.trial_timeout,
                )
            finished = time.monotonic()
            runner.spin_for(1.0)
            physical, xy_error, z_error = runner.physics_success()
            state_success = place_state == 'COMPLETE'
            cycle_success = bool(state_success and physical is not False)
            row = {
                'controller': args.controller,
                'trial': trial,
                'seed': seed,
                'object_x_m': x,
                'object_y_m': y,
                'object_yaw_rad': yaw,
                'set_pose_success': set_pose_success,
                'pick_state': pick_state,
                'place_state': place_state,
                'pick_time_s': pick_finished - started,
                'place_time_s': (
                    finished - pick_finished
                    if pick_state == 'WAIT_DELIVERY'
                    else math.nan
                ),
                'cycle_time_s': finished - started,
                'state_success': state_success,
                'physics_success': physical,
                'cycle_success': cycle_success,
                'final_box_x_m': (
                    float(runner.box_position[0])
                    if runner.box_position is not None else math.nan
                ),
                'final_box_y_m': (
                    float(runner.box_position[1])
                    if runner.box_position is not None else math.nan
                ),
                'final_box_z_m': (
                    float(runner.box_position[2])
                    if runner.box_position is not None else math.nan
                ),
                'final_xy_error_m': xy_error,
                'final_z_error_m': z_error,
                'status': runner.status_text,
            }
            rows.append(row)
            write_rows(args.output, rows)
            print(
                f"controller={args.controller} trial={trial}/{args.trials} "
                f"success={cycle_success} state={place_state} "
                f"physics={physical} time={row['cycle_time_s']:.3f}s "
                f"xy_error={xy_error:.4f} z_error={z_error:.4f}",
                flush=True,
            )

            if trial < args.trials:
                if not recover(runner, args.ready_timeout):
                    raise RuntimeError(
                        f'failed to recover after trial {trial}: '
                        f'state={runner.state} status={runner.status_text}')
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
