#!/usr/bin/env python3
"""One-way, allowlisted leader TELEMETRY bridge. No commands or reverse relay."""
import argparse
from pathlib import Path
import signal
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
import yaml

ALLOWED = {
    '/leader_monitor/input/joint_states': 'sensor_msgs/msg/JointState',
    '/leader_monitor/input/eef/image_raw': 'sensor_msgs/msg/Image',
    '/leader_monitor/input/eef/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/leader_monitor/input/status': 'std_msgs/msg/String',
    '/leader_monitor/input/aruco_pose': 'geometry_msgs/msg/PoseStamped',
    '/leader_monitor/input/target_valid': 'std_msgs/msg/Bool',
    '/leader_monitor/input/target_info': 'std_msgs/msg/String',
}


def load_routes(path, transport):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict) or transport not in ('raw', 'compressed'):
        raise ValueError('Expected a mapping and raw/compressed transport')
    routes = config.get('topics', [])
    if not isinstance(routes, list) or not routes:
        raise ValueError('topics must be a non-empty list')
    result, destinations = [], set()
    for item in routes:
        if not isinstance(item, dict) or not all(k in item for k in ('source', 'destination', 'type')):
            raise ValueError('Every route needs source, destination, type')
        source, destination, kind = item['source'], item['destination'], item['type']
        if not isinstance(destination, str) or not isinstance(kind, str):
            raise ValueError('Destination and type must be strings')
        if ALLOWED.get(destination) != kind or destination in destinations:
            raise ValueError('Only unique, predefined telemetry destinations/types are allowed')
        if not isinstance(source, str) or not source.startswith('/') or source.startswith('/leader_monitor/'):
            raise ValueError('Source must be an external absolute telemetry topic')
        destinations.add(destination)
        if item.get('transport', transport) not in ('raw', 'compressed'):
            raise ValueError('Invalid image transport')
        if item.get('transport', transport) == transport:
            result.append((source, destination, kind))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--source-domain', required=True, type=int)
    parser.add_argument('--host-domain', required=True, type=int)
    parser.add_argument('--transport', choices=['raw', 'compressed'], default='raw')
    args, _ = parser.parse_known_args()
    if not (0 <= args.source_domain <= 232 and 0 <= args.host_domain <= 232):
        raise ValueError('Invalid domain')
    if args.source_domain == args.host_domain:
        raise ValueError('Source and host domains must differ; use topic remapping on the same domain')
    routes = load_routes(args.config, args.transport)
    source_context, host_context = Context(), Context()
    source_context.init(args=[], domain_id=args.source_domain)
    host_context.init(args=[], domain_id=args.host_domain)
    source = Node('leader_monitor_bridge_source', context=source_context)
    host = Node('leader_monitor_bridge_host', context=host_context)
    executor = SingleThreadedExecutor(context=source_context)
    executor.add_node(source)
    endpoints = []
    for source_topic, destination, type_name in routes:
        kind = get_message(type_name)
        qos = QoSProfile(depth=2 if 'Image' in type_name else 10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        publisher = host.create_publisher(kind, destination, qos)
        subscription = source.create_subscription(kind, source_topic, publisher.publish, qos)
        endpoints.extend((publisher, subscription))
        print(f'TELEMETRY {args.source_domain}:{source_topic} -> {args.host_domain}:{destination}', flush=True)
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    try:
        while not stopped.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        source.destroy_node()
        host.destroy_node()
        source_context.shutdown()
        host_context.shutdown()


if __name__ == '__main__':
    main()
