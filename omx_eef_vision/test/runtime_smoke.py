# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Explicit localhost-only ROS smoke test; never connect to robot domains."""

import json
import os
import time

from geometry_msgs.msg import TransformStamped
from omx_eef_vision.multi_object_node import MultiObjectNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from test_multi_object import calibration, scene
from tf2_ros import StaticTransformBroadcaster


def main():
    """Verify real ROS serialization, callbacks, TF, selection and expiry."""
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or os.environ.get('ROS_DOMAIN_ID') != '228'):
        raise RuntimeError('Test requires ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=228')
    rclpy.init(args=[
        '--ros-args', '-p', 'marker_size_confirmed:=true',
        '-p', 'extrinsics_confirmed:=true'])
    vision = MultiObjectNode()
    driver = Node('perception_synthetic_test', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(vision)
    executor.add_node(driver)
    images = driver.create_publisher(Image, '/eef_camera/image_raw',
                                     qos_profile_sensor_data)
    infos = driver.create_publisher(CameraInfo, '/eef_camera/camera_info',
                                    qos_profile_sensor_data)
    requests = driver.create_publisher(String, '/perception/target_request', 10)
    results, all_objects = [], []
    driver.create_subscription(String, '/perception/selected_info',
                               lambda msg: results.append(json.loads(msg.data)), 10)
    driver.create_subscription(String, '/perception/objects',
                               lambda msg: all_objects.append(json.loads(msg.data)), 10)
    broadcaster = StaticTransformBroadcaster(driver)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'test_camera'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    def spin(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)

    def frames(ids=(1, 2, 3, 4), count=8, info=None, stamp_delta=0):
        array = scene(ids)
        for _ in range(count):
            camera = info if info is not None else calibration()
            camera.header.stamp = driver.get_clock().now().to_msg()
            infos.publish(camera)
            spin(.01)
            message = Image(height=480, width=640, encoding='bgr8', step=1920,
                            data=array.tobytes())
            message.header.frame_id = 'test_camera'
            message.header.stamp = driver.get_clock().now().to_msg()
            message.header.stamp.sec += stamp_delta
            images.publish(message)
            spin(.05)

    def select(marker_id, request_id):
        requests.publish(String(data=json.dumps({
            'request_id': request_id, 'object_id': marker_id, 'command': 'SELECT'})))
        spin(.04)

    try:
        spin(.5)
        frames()
        assert all_objects[-1]['overview_ready']
        assert len(all_objects[-1]['objects']) == 4
        assert not results[-1]['valid']
        select(3, 'smoke-3')
        assert not results[-1]['valid']
        frames()
        assert results[-1]['valid'], results[-1]
        assert results[-1]['marker_id'] == 3
        first_position = results[-1]['position']
        vision.cfg['marker_size_confirmed'] = False
        frames()
        assert results[-1]['reason'] == 'MARKER_SIZE_UNCONFIRMED'
        vision.cfg['marker_size_confirmed'] = True
        vision.cfg['extrinsics_confirmed'] = False
        frames()
        assert results[-1]['reason'] == 'EXTRINSICS_UNCONFIRMED'
        vision.cfg['extrinsics_confirmed'] = True
        vision.cfg['target_frame'] = 'missing_base'
        frames()
        assert results[-1]['reason'] == 'WAIT_TIMESTAMPED_TF'
        vision.cfg['target_frame'] = 'base_link'
        frames()
        assert results[-1]['valid']
        frames((3,))
        assert results[-1]['valid']  # Other markers may leave the close-up view.
        frames((1, 2, 4))
        assert not results[-1]['valid']
        assert results[-1]['reason'] == 'TARGET_NOT_VISIBLE'
        select(1, 'smoke-1')
        frames()
        assert results[-1]['valid'] and results[-1]['marker_id'] == 1
        assert results[-1]['position'] != first_position
        select(3, 'smoke-duplicate')
        frames((1, 2, 3, 3))
        assert results[-1]['reason'] == 'DUPLICATE_ID'
        bad_info = calibration()
        bad_info.width = 320
        frames(info=bad_info)
        assert results[-1]['reason'] == 'CALIBRATION_SIZE_MISMATCH'
        assert len(all_objects[-1]['objects']) == 4
        frames()
        assert results[-1]['valid']
        frames(count=1, stamp_delta=-2)
        assert not results[-1]['valid']
        frames()
        assert results[-1]['valid']
        spin(.65)
        assert results[-1]['reason'] == 'IMAGE_TIMEOUT'
        assert not all_objects[-1]['fresh'] and not all_objects[-1]['objects']
        published = driver.get_publisher_names_and_types_by_node(
            'multi_object_vision', '/')
        assert not any(name.startswith(('/target/', '/arm_controller/',
                                        '/gripper_controller/', '/cmd_vel'))
                       for name, _ in published)
        print('PASS: four IDs, selection, loss, duplicate, calibration, stale '
              'image, timeout, no motion publishers')
    finally:
        executor.shutdown()
        driver.destroy_node()
        vision.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
