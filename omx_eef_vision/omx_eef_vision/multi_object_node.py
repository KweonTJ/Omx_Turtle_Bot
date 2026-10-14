# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Camera-only multi-object perception; never publish robot control targets."""

import json
import time

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import tf2_geometry_msgs  # noqa: F401
import tf2_ros

from .aruco_geometry import estimate_square_pose, rotation_matrix_to_quaternion
from .eef_vision_node import image_to_bgr
from .multi_object_core import camera_model, MultiMarkerDetector, SelectionSession


class MultiObjectNode(Node):
    """Publish current-frame detections and a separately selected marker pose."""

    def __init__(self):
        super().__init__('multi_object_vision')
        defaults = {
            'image_topic': '/eef_camera/image_raw',
            'camera_info_topic': '/eef_camera/camera_info',
            'dictionary': 'DICT_4X4_50', 'expected_ids': [1, 2, 3, 4],
            'marker_size_m': 0.04, 'marker_size_confirmed': False,
            'extrinsics_confirmed': False, 'target_frame': 'base_link',
            'stable_frames': 3, 'min_perimeter_px': 48.0,
            'max_reprojection_error_px': 3.0,
            'depth_min_m': 0.05, 'depth_max_m': 1.2,
            'max_position_jump_m': 0.08, 'max_angle_jump_rad': 0.8,
            'max_image_age_s': 0.4, 'image_timeout_s': 0.5,
            'future_tolerance_s': 0.05, 'publish_debug_image': True,
        }
        self.cfg = {key: self.declare_parameter(
            key, value, descriptor=ParameterDescriptor(read_only=True)).value
                    for key, value in defaults.items()}
        positive = ('marker_size_m', 'min_perimeter_px',
                    'max_reprojection_error_px', 'depth_min_m', 'depth_max_m',
                    'max_position_jump_m', 'max_angle_jump_rad',
                    'max_image_age_s', 'image_timeout_s', 'future_tolerance_s')
        if any(not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0
               for key in positive):
            raise ValueError('Perception limits must be finite and positive')
        if (self.cfg['depth_min_m'] >= self.cfg['depth_max_m']
                or not self.cfg['target_frame']):
            raise ValueError('Invalid depth range or target frame')
        self.detector = MultiMarkerDetector(
            self.cfg['dictionary'], self.cfg['min_perimeter_px'])
        if max(self.cfg['expected_ids']) >= len(self.detector.dictionary.bytesList):
            raise ValueError('Configured ID is outside the dictionary')
        self.session = SelectionSession(
            self.cfg['expected_ids'], self.cfg['stable_frames'],
            self.cfg['max_position_jump_m'], self.cfg['max_angle_jump_rad'])
        self.camera_info = None
        self.calibration_key = None
        self.last_stamp_ns = -1
        self.last_receive = None
        self.timed_out = True
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Relative output names allow namespaces/remaps; defaults are perception/*.
        self.objects_pub = self.create_publisher(String, 'perception/objects', 1)
        self.selected_pub = self.create_publisher(
            String, 'perception/selected_info', 1)
        self.pose_pub = self.create_publisher(
            PoseStamped, 'perception/selected_pose', 1)
        self.valid_pub = self.create_publisher(Bool, 'perception/selected_valid', 1)
        self.status_pub = self.create_publisher(String, 'perception/status', 1)
        self.debug_pub = self.create_publisher(
            Image, 'perception/debug_image', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'],
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.cfg['image_topic'],
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, 'perception/target_request',
                                 self.on_request, 10)
        self.create_timer(0.1, self.on_watchdog)
        self.publish_invalid('WAIT_IMAGE')
        self.get_logger().info(
            'Perception only: IDs 1-4 profile; no arm, gripper or base commands.')

    def on_camera_info(self, message):
        """Keep the latest calibration, including invalid replacements."""
        key = (message.width, message.height, message.header.frame_id,
               message.distortion_model, tuple(message.k), tuple(message.d),
               message.binning_x, message.binning_y, message.roi.x_offset,
               message.roi.y_offset, message.roi.width, message.roi.height)
        if key != self.calibration_key:
            self.session.reset_observations()
            self.publish_invalid('CALIBRATION_CHANGED')
            self.calibration_key = key
        self.camera_info = message

    def emit(self, publisher, payload):
        """Serialize strict JSON without non-finite floats."""
        publisher.publish(String(data=json.dumps(payload, allow_nan=False)))

    def selection_info(self, reason, stamp_ns=0, record=None):
        """Keep ID, request and pose validity in one atomic message."""
        result = {
            'schema_version': 1, 'request_id': self.session.request_id,
            'marker_id': self.session.selected_id, 'valid': reason == 'TRACKING',
            'reason': reason, 'stamp_ns': stamp_ns,
            'frame_id': self.cfg['target_frame'], 'shelf_level': None,
            'overview_ready': self.session.overview_ready,
            'position': None, 'orientation_xyzw': None,
        }
        if result['valid'] and record is not None:
            result['position'] = record['position']
            result['orientation_xyzw'] = record['orientation_xyzw']
        return result

    def publish_invalid(self, reason):
        """Withdraw selection without republishing an old pose."""
        self.valid_pub.publish(Bool(data=False))
        self.emit(self.selected_pub, self.selection_info(reason))
        self.emit(self.status_pub, {
            'state': reason, 'valid': False,
            'overview_ready': self.session.overview_ready})

    def on_request(self, message):
        """Change the requested ID, not the robot state or its motion."""
        try:
            changed = self.session.request(
                message.data, self.get_clock().now().nanoseconds)
        except (ValueError, TypeError) as error:
            self.emit(self.status_pub, {
                'state': 'REQUEST_REJECTED', 'detail': str(error)})
            return
        if changed:
            self.publish_invalid('WAIT_NEW_FRAME' if self.session.selected_id
                                 is not None else 'WAIT_REQUEST')

    def on_watchdog(self):
        """Invalidate on receipt timeout or ROS timestamp expiry."""
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - self.last_stamp_ns) / 1e9
        if (self.last_receive is None
                or time.monotonic() - self.last_receive
                > self.cfg['image_timeout_s']
                or age > self.cfg['max_image_age_s']
                or age < -self.cfg['future_tolerance_s']):
            self.session.reset_observations()
            self.timed_out = True
            self.publish_invalid('WAIT_IMAGE' if self.last_receive is None
                                 else 'IMAGE_TIMEOUT')
            self.emit(self.objects_pub, {
                'schema_version': 1, 'fresh': False, 'stamp_ns': 0,
                'objects': [], 'overview_ready': False,
            })

    def pose_for(self, record, message, model, transform):
        """Estimate the marker centre; it is not a grasp or reachable target."""
        try:
            estimate = estimate_square_pose(
                record['corners_px'], self.cfg['marker_size_m'], *model)
        except (cv2.error, ValueError):
            estimate = None
        if estimate is None:
            record['pose_reason'] = 'PNP_FAILED'
            return
        position = estimate.tvec.reshape(3)
        if (not np.isfinite(position).all()
                or not np.isfinite(estimate.rvec).all()
                or not np.isfinite(estimate.reprojection_error_px)
                or estimate.reprojection_error_px
                > self.cfg['max_reprojection_error_px']
                or not self.cfg['depth_min_m'] <= position[2]
                <= self.cfg['depth_max_m']):
            record['pose_reason'] = 'POSE_QUALITY_REJECTED'
            return
        rotation, _ = cv2.Rodrigues(estimate.rvec)
        quaternion = rotation_matrix_to_quaternion(rotation)
        record.update(camera_position=position.tolist(),
                      camera_orientation_xyzw=quaternion.tolist(),
                      reprojection_error_px=estimate.reprojection_error_px)
        if not self.cfg['marker_size_confirmed']:
            record['pose_reason'] = 'MARKER_SIZE_UNCONFIRMED'
            return
        if not self.cfg['extrinsics_confirmed']:
            record['pose_reason'] = 'EXTRINSICS_UNCONFIRMED'
            return
        if transform is None:
            record['pose_reason'] = 'WAIT_TIMESTAMPED_TF'
            return
        pose = PoseStamped(header=message.header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        converted = tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
        p, q = converted.pose.position, converted.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1) > .01:
            record['pose_reason'] = 'INVALID_TRANSFORM_RESULT'
            return
        record.update(position=values[:3], orientation_xyzw=values[3:],
                      frame_id=self.cfg['target_frame'], pose_valid=True,
                      pose_reason='POSE_AVAILABLE')

    def on_image(self, message):
        """Process one acquisition timestamp; reject stale and repeated frames."""
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if (stamp_ns <= 0 or age > self.cfg['max_image_age_s']
                or age < -self.cfg['future_tolerance_s']
                or (not self.timed_out and stamp_ns <= self.last_stamp_ns)):
            self.session.reset_observations()
            self.publish_invalid('INVALID_IMAGE_TIMESTAMP')
            return
        try:
            image = image_to_bgr(message)
            records = self.detector.detect(image)
        except (ValueError, cv2.error) as error:
            self.session.reset_observations()
            self.publish_invalid('INVALID_IMAGE')
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.last_receive = time.monotonic()
        self.last_stamp_ns = stamp_ns
        self.timed_out = False
        model = None
        calibration_reason = 'WAIT_CALIBRATION'
        try:
            model = camera_model(self.camera_info, message.width,
                                 message.height, message.header.frame_id)
        except ValueError as error:
            calibration_reason = str(error)
        transform = None
        if model is not None and self.cfg['extrinsics_confirmed']:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.cfg['target_frame'], message.header.frame_id,
                    Time.from_msg(message.header.stamp), timeout=Duration())
            except tf2_ros.TransformException:
                pass
        for record in records:
            record['expected'] = record['marker_id'] in self.session.expected_ids
            if not record['usable_2d']:
                record['pose_reason'] = ('DUPLICATE_ID' if record['duplicate']
                                         else 'MARKER_TOO_SMALL')
            elif not record['expected']:
                record['pose_reason'] = 'UNCONFIGURED_ID'
            elif model is None:
                record['pose_reason'] = calibration_reason
            else:
                self.pose_for(record, message, model, transform)
        selected, reason = self.session.observe(records, stamp_ns)
        # Include processing time in the freshness gate as well.
        if ((self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                > self.cfg['max_image_age_s']):
            self.session.reset_observations()
            self.publish_invalid('PROCESSING_TIMEOUT')
            return
        self.emit(self.objects_pub, {
            'schema_version': 1, 'fresh': True, 'stamp_ns': stamp_ns,
            'camera_frame': message.header.frame_id,
            'dictionary': self.cfg['dictionary'],
            'marker_size_m': self.cfg['marker_size_m'],
            'marker_size_confirmed': self.cfg['marker_size_confirmed'],
            'image_size': [message.width, message.height],
            'overview_ready': self.session.overview_ready, 'objects': records,
        })
        info = self.selection_info(reason, stamp_ns, selected)
        self.valid_pub.publish(Bool(data=info['valid']))
        self.emit(self.selected_pub, info)
        self.emit(self.status_pub, {
            'state': reason, 'request_id': self.session.request_id,
            'selected_id': self.session.selected_id,
            'overview_ready': self.session.overview_ready,
            'detected_ids': [r['marker_id'] for r in records]})
        if info['valid']:
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = self.cfg['target_frame']
            p, q = pose.pose.position, pose.pose.orientation
            p.x, p.y, p.z = info['position']
            q.x, q.y, q.z, q.w = info['orientation_xyzw']
            self.pose_pub.publish(pose)
        if self.cfg['publish_debug_image']:
            self.publish_debug(image, records, message, reason)

    def publish_debug(self, image, records, message, reason):
        """Keep the full camera view and highlight the requested marker."""
        for record in records:
            selected = record['marker_id'] == self.session.selected_id
            color = (0, 200, 255) if selected else (50, 200, 50)
            if not record['usable_2d']:
                color = (0, 0, 255)
            points = np.asarray(record['corners_px'], dtype=np.int32)
            cv2.polylines(image, [points], True, color, 3 if selected else 2)
            x, y = points[0]
            label = f"ID {record['marker_id']}" + (' SELECTED' if selected else '')
            cv2.putText(image, label, (int(x), max(20, int(y)-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        cv2.putText(image, reason, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (0, 0, 220), 2)
        debug = Image(header=message.header, height=image.shape[0],
                      width=image.shape[1], encoding='bgr8',
                      step=image.shape[1]*3, data=image.tobytes())
        self.debug_pub.publish(debug)


def main(args=None):
    """Run recognition only."""
    rclpy.init(args=args)
    node = MultiObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
