#!/usr/bin/env python3
"""Explicit, localhost-only UI and one-way bridge checks; no hardware launch.

After sourcing the build, run: python3 test/runtime_smoke.py
Uses domains 224, 225, 230, 231 and publishes only private demo topics except a
synthetic /joint_states on the isolated localhost source domain 231.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
BINARY = Path(get_package_prefix('omx_leader_monitor')) / 'lib/omx_leader_monitor/leader_monitor'
ARTIFACT = Path(tempfile.mkdtemp(prefix='leader_monitor_test.'))


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def wait_for(predicate, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.1)
    raise AssertionError(label)


def ui_test(transport, domain):
    report = ARTIFACT / f'{transport}.json'
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               QT_QPA_PLATFORM='offscreen', ROS_LOG_DIR=str(ARTIFACT / 'ros'))
    with (ARTIFACT / f'{transport}.log').open('w') as log:
        viewer = subprocess.Popen([str(BINARY), '--ros-args', '-p', 'enable_3d:=false',
            '-p', 'demo_mode:=true', '-p', 'input_prefix:=/leader_monitor/demo',
            '-p', 'host_request_topic:=/leader_monitor/demo/host_request',
            '-p', f'image_transport:={transport}', '-p', f'diagnostics_path:={report}',
            '-p', 'auto_close_s:=40.0'], env=env, stdout=log, stderr=subprocess.STDOUT)
        demo = subprocess.Popen(['python3', str(ROOT / 'scripts/demo_telemetry.py'),
                                 '--transport', transport], env=env, stdout=log, stderr=subprocess.STDOUT)
        def state():
            assert viewer.poll() is None, 'Viewer exited prematurely'
            try:
                return json.loads(report.read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                return {}
        try:
            wait_for(lambda: all(state().get(k) for k in ('joint_live', 'camera_live', 'status_live', 'target_live')),
                     12, 'Telemetry never became live')
            wait_for(lambda: state().get('near'), 18, 'Near view label did not switch')
            live = state()
            assert live['requests'] == 1 and live['object_id'] == live['detected_id'] == 3
            stop(demo)
            wait_for(lambda: all(state().get(k) is False for k in ('joint_live', 'camera_live', 'status_live', 'target_live', 'near')),
                     6, 'Stopped telemetry still reported live')
            assert state()['requests'] == 1
            print(f'PASS {transport}: image/joints/status/ID, proximity, dedupe, dropout', flush=True)
        finally:
            stop(demo)
            stop(viewer)
        assert viewer.returncode == 0, f'Viewer exit: {viewer.returncode}'


def bridge_test():
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(ARTIFACT / 'ros')
    source_context, host_context = Context(), Context()
    source_context.init(args=[], domain_id=231)
    host_context.init(args=[], domain_id=230)
    source = Node('monitor_test_source', context=source_context)
    host = Node('monitor_test_host', context=host_context)
    pub = source.create_publisher(JointState, '/joint_states', 10)
    received = []
    sub = host.create_subscription(JointState, '/leader_monitor/input/joint_states',
                                   received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=host_context)
    executor.add_node(host)
    with (ARTIFACT / 'bridge.log').open('w') as log:
        process = subprocess.Popen(['python3', str(ROOT / 'scripts/topic_bridge.py'),
            '--source-domain', '231', '--host-domain', '230', '--config', str(ROOT / 'config/bridge_topics.yaml')],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 12
            while not received and time.monotonic() < deadline:
                assert process.poll() is None, 'Bridge exited prematurely'
                pub.publish(JointState(name=['joint1'], position=[.123]))
                executor.spin_once(timeout_sec=.1)
            assert received and list(received[-1].position) == [.123]
            command_topics = {'/cmd_vel', '/rl_control/command', '/arm_controller/joint_trajectory'}
            assert not command_topics.intersection(name for name, _ in host.get_topic_names_and_types())
            print('PASS bridge: domain 231 -> 230 payload; no control topics', flush=True)
        finally:
            stop(process)
            executor.shutdown()
            source.destroy_node()
            host.destroy_node()
            source_context.shutdown()
            host_context.shutdown()
        assert process.returncode == 0


if __name__ == '__main__':
    print(f'Artifacts: {ARTIFACT}', flush=True)
    ui_test('raw', 224)
    ui_test('compressed', 225)
    bridge_test()
