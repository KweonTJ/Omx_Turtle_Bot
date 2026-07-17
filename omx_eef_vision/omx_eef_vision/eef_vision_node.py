#!/usr/bin/env python3
"""ArUco-only object pose estimation for the end-effector camera."""

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import tf2_geometry_msgs  # noqa: F401 - register geometry message transforms
import tf2_ros

from .aruco_geometry import (
    apply_marker_offset,
    blend_quaternions,
    estimate_square_pose,
    quaternion_angle,
    rotation_matrix_to_quaternion,
)


def get_aruco_dictionary(name: str):
    """Return an OpenCV predefined ArUco dictionary by name."""
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError(
            'OpenCV ArUco support is unavailable; install python3-opencv '
            'with the contrib modules.')
    if not hasattr(cv2.aruco, name):
        raise ValueError(f'Unknown ArUco dictionary: {name}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_detector_parameters():
    """Create detector parameters across supported OpenCV versions."""
    if hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 5
    parameters.cornerRefinementMaxIterations = 30
    parameters.cornerRefinementMinAccuracy = 0.01
    parameters.minMarkerPerimeterRate = 0.01
    parameters.polygonalApproxAccuracyRate = 0.03
    parameters.errorCorrectionRate = 0.6
    return parameters


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert common ROS image encodings without importing cv_bridge."""
    if message.height <= 0 or message.width <= 0 or message.step <= 0:
        raise ValueError('Image dimensions and step must be positive')

    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected_size = int(message.height) * int(message.step)
    if raw.size < expected_size:
        raise ValueError(
            f'Image data is short: received={raw.size}, expected={expected_size}')
    rows = raw[:expected_size].reshape((message.height, message.step))
    encoding = message.encoding.lower()

    if encoding in ('bgr8', 'rgb8'):
        packed = rows[:, :message.width * 3]
        image = packed.reshape((message.height, message.width, 3)).copy()
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    if encoding in ('bgra8', 'rgba8'):
        packed = rows[:, :message.width * 4]
        image = packed.reshape((message.height, message.width, 4)).copy()
        conversion = (
            cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(image, conversion)

    if encoding in ('mono8', '8uc1'):
        gray = rows[:, :message.width].copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if encoding in ('yuyv', 'yuy2', 'yuv422_yuy2'):
        packed = rows[:, :message.width * 2]
        image = packed.reshape((message.height, message.width, 2)).copy()
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)

    if encoding in ('uyvy', 'yuv422'):
        packed = rows[:, :message.width * 2]
        image = packed.reshape((message.height, message.width, 2)).copy()
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)

    raise ValueError(f'Unsupported image encoding: {message.encoding}')


class EefVisionNode(Node):
    """Detect an ArUco marker and publish a filtered object pose."""

    def __init__(self):
        super().__init__('eef_vision_node')

        self.image_topic = self._parameter(
            'image_topic', '/eef_camera/image_raw')
        self.camera_info_topic = self._parameter(
            'camera_info_topic', '/eef_camera/camera_info')
        self.marker_pose_topic = self._parameter(
            'marker_pose_topic', '/target/aruco_pose')
        self.object_pose_topic = self._parameter(
            'object_pose_topic', '/target/object_pose')
        self.visible_topic = self._parameter(
            'visible_topic', '/target/aruco_visible')
        self.valid_topic = self._parameter('valid_topic', '/target/valid')
        self.status_topic = self._parameter(
            'status_topic', '/target/aruco_status')
        self.debug_image_topic = self._parameter(
            'debug_image_topic', '/target/aruco_debug_image')

        self.camera_frame = self._parameter(
            'camera_frame', 'eef_usb_camera_optical_frame')
        self.target_frame = self._parameter('target_frame', 'base_link')
        self.use_latest_tf = bool(self._parameter('use_latest_tf', True))
        self.transform_timeout_s = float(
            self._parameter('transform_timeout_s', 0.08))

        self.marker_id = int(self._parameter('marker_id', 0))
        self.accept_any_marker = bool(
            self._parameter('accept_any_marker', True))
        self.marker_size_m = float(self._parameter('marker_size_m', 0.05))
        self.dictionary_name = str(
            self._parameter('dictionary', 'DICT_APRILTAG_36h11'))
        self.object_offset_xyz = self._vector_parameter(
            'object_offset_xyz', [0.0, 0.0, 0.0])
        self.object_offset_rpy = self._vector_parameter(
            'object_offset_rpy', [0.0, 0.0, 0.0])

        self.min_marker_perimeter_px = float(
            self._parameter('min_marker_perimeter_px', 12.0))
        self.max_reprojection_error_px = float(
            self._parameter('max_reprojection_error_px', 8.0))
        self.pose_z_min_m = float(self._parameter('pose_z_min_m', 0.04))
        self.pose_z_max_m = float(self._parameter('pose_z_max_m', 1.0))
        self.target_workspace_min = self._vector_parameter(
            'target_workspace_min', [-0.5, -0.5, 0.0])
        self.target_workspace_max = self._vector_parameter(
            'target_workspace_max', [0.8, 0.5, 0.8])

        self.stable_detection_count = int(
            self._parameter('stable_detection_count', 3))
        self.position_filter_alpha = float(
            self._parameter('position_filter_alpha', 0.35))
        self.orientation_filter_alpha = float(
            self._parameter('orientation_filter_alpha', 0.35))
        self.max_position_jump_m = float(
            self._parameter('max_position_jump_m', 0.10))
        self.max_orientation_jump_rad = float(
            self._parameter('max_orientation_jump_rad', 0.90))
        self.detection_timeout_s = float(
            self._parameter('detection_timeout_s', 0.35))
        self.image_timeout_s = float(
            self._parameter('image_timeout_s', 1.0))
        self.filter_reset_timeout_s = float(
            self._parameter('filter_reset_timeout_s', 0.6))

        self.publish_debug_image = bool(
            self._parameter('publish_debug_image', True))
        self.draw_axes = bool(self._parameter('draw_axes', True))
        self.axis_length_m = float(
            self._parameter('axis_length_m', 0.03))
        self.use_clahe = bool(self._parameter('use_clahe', False))
        self.status_period_s = float(
            self._parameter('status_period_s', 0.5))

        self._validate_parameters()

        self.aruco_dictionary = get_aruco_dictionary(self.dictionary_name)
        self.detector_parameters = make_detector_parameters()
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dictionary,
                self.detector_parameters,
            )
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.camera_matrix = None
        self.distortion = None
        self.camera_info_size = None
        self.camera_info_frame = self.camera_frame

        self.filtered_position = None
        self.filtered_orientation = None
        self.stable_count = 0
        self.last_image_time = None
        self.last_detection_time = None
        self.last_status = None
        self.last_status_time = self.get_clock().now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pose_pub = self.create_publisher(
            PoseStamped, self.marker_pose_topic, 10)
        self.object_pose_pub = self.create_publisher(
            PoseStamped, self.object_pose_topic, 10)
        self.visible_pub = self.create_publisher(Bool, self.visible_topic, 10)
        self.valid_pub = self.create_publisher(Bool, self.valid_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.debug_pub = self.create_publisher(
            Image, self.debug_image_topic, qos_profile_sensor_data)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.timeout_timer = self.create_timer(0.1, self.on_timeout_timer)

        self.publish_status(
            'ArUco-only EEF vision ready: '
            f'image={self.image_topic}, info={self.camera_info_topic}, '
            f'dictionary={self.dictionary_name}, marker_id={self.marker_id}, '
            f'marker_size={self.marker_size_m:.4f}m, '
            f'target_frame={self.target_frame}')

    def _parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def _vector_parameter(self, name, default) -> np.ndarray:
        value = self._parameter(name, default)
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (3,) or not np.isfinite(array).all():
            raise ValueError(f'{name} must contain three finite values')
        return array

    def _validate_parameters(self):
        if self.marker_size_m <= 0.0:
            raise ValueError('marker_size_m must be positive')
        if self.pose_z_min_m < 0.0 or self.pose_z_max_m <= self.pose_z_min_m:
            raise ValueError('pose_z_min_m/pose_z_max_m are invalid')
        if np.any(self.target_workspace_max <= self.target_workspace_min):
            raise ValueError('target workspace max must be greater than min')
        if self.stable_detection_count < 1:
            raise ValueError('stable_detection_count must be at least one')
        for name, value in (
            ('position_filter_alpha', self.position_filter_alpha),
            ('orientation_filter_alpha', self.orientation_filter_alpha),
        ):
            if not 0.0 < value <= 1.0:
                raise ValueError(f'{name} must be in (0, 1]')
        if self.max_position_jump_m <= 0.0:
            raise ValueError('max_position_jump_m must be positive')
        if self.max_orientation_jump_rad <= 0.0:
            raise ValueError('max_orientation_jump_rad must be positive')

    def publish_status(self, text: str):
        """Publish a throttled human-readable detector status."""
        now = self.get_clock().now()
        elapsed = (now - self.last_status_time).nanoseconds * 1.0e-9
        if text == self.last_status and elapsed < self.status_period_s:
            return
        self.last_status = text
        self.last_status_time = now
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def publish_flags(self, visible: bool, valid: bool):
        """Publish raw marker visibility and control-safe pose validity."""
        visible_message = Bool()
        visible_message.data = bool(visible)
        self.visible_pub.publish(visible_message)

        valid_message = Bool()
        valid_message.data = bool(valid)
        self.valid_pub.publish(valid_message)

    def on_camera_info(self, message: CameraInfo):
        """Store calibrated camera intrinsics."""
        matrix = np.asarray(message.k, dtype=np.float64).reshape((3, 3))
        if (
            not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0.0
            or matrix[1, 1] <= 0.0
        ):
            self.publish_status(
                'Invalid camera_info; waiting for calibrated intrinsics')
            return

        distortion = np.asarray(message.d, dtype=np.float64)
        if distortion.size == 0:
            distortion = np.zeros((5,), dtype=np.float64)
        if not np.isfinite(distortion).all():
            self.publish_status(
                'Invalid distortion coefficients in camera_info')
            return

        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_info_size = (int(message.width), int(message.height))
        self.camera_info_frame = message.header.frame_id or self.camera_frame

    def camera_model_for_image(self, message: Image):
        """Scale CameraInfo intrinsics to the incoming image dimensions."""
        if self.camera_matrix is None or self.distortion is None:
            return None, None
        matrix = self.camera_matrix.copy()
        info_width, info_height = self.camera_info_size
        if info_width > 0 and info_height > 0:
            scale_x = float(message.width) / float(info_width)
            scale_y = float(message.height) / float(info_height)
            matrix[0, :] *= scale_x
            matrix[1, :] *= scale_y
            matrix[2, :] = self.camera_matrix[2, :]
        return matrix, self.distortion

    def detect_markers(self, gray: np.ndarray):
        """Detect marker corners with the available OpenCV API."""
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray,
            self.aruco_dictionary,
            parameters=self.detector_parameters,
        )

    @staticmethod
    def marker_perimeter(corner: np.ndarray) -> float:
        """Return the marker perimeter in pixels."""
        points = np.asarray(corner, dtype=np.float64).reshape((4, 2))
        return float(sum(
            np.linalg.norm(points[(index + 1) % 4] - points[index])
            for index in range(4)
        ))

    def select_marker(self, corners, ids):
        """Select the largest configured marker that passes size gating."""
        candidates = []
        detected_ids = ids.flatten().astype(int).tolist()
        for index, detected_id in enumerate(detected_ids):
            if not self.accept_any_marker and detected_id != self.marker_id:
                continue
            perimeter = self.marker_perimeter(corners[index])
            if perimeter < self.min_marker_perimeter_px:
                continue
            candidates.append((perimeter, index, detected_id))
        if not candidates:
            return None, detected_ids
        _, index, detected_id = max(candidates)
        return (index, detected_id), detected_ids

    @staticmethod
    def make_pose(stamp, frame_id, position, orientation) -> PoseStamped:
        """Build a PoseStamped from NumPy vectors."""
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(orientation[0])
        message.pose.orientation.y = float(orientation[1])
        message.pose.orientation.z = float(orientation[2])
        message.pose.orientation.w = float(orientation[3])
        return message

    def transform_object_pose(self, pose: PoseStamped):
        """Transform an object pose into the configured target frame."""
        if not self.target_frame or pose.header.frame_id == self.target_frame:
            return pose

        original_stamp = pose.header.stamp
        transform_input = PoseStamped()
        transform_input.header = pose.header
        transform_input.pose = pose.pose
        if self.use_latest_tf:
            transform_input.header.stamp = Time().to_msg()

        transformed = self.tf_buffer.transform(
            transform_input,
            self.target_frame,
            timeout=Duration(seconds=self.transform_timeout_s),
        )
        transformed.header.stamp = original_stamp
        return transformed

    def filter_pose(self, pose: PoseStamped):
        """Reject jumps and low-pass filter a transformed object pose."""
        position = np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        orientation = np.array(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )

        if not np.isfinite(position).all() or not np.isfinite(orientation).all():
            return None, 'non-finite transformed pose'
        orientation_norm = float(np.linalg.norm(orientation))
        if orientation_norm <= 0.0:
            return None, 'zero-length transformed quaternion'
        orientation /= orientation_norm

        if np.any(position < self.target_workspace_min) or np.any(
            position > self.target_workspace_max
        ):
            return None, (
                'target pose outside workspace: '
                f'xyz=({position[0]:.3f}, {position[1]:.3f}, '
                f'{position[2]:.3f})')

        if self.filtered_position is not None:
            position_jump = float(np.linalg.norm(
                position - self.filtered_position))
            orientation_jump = quaternion_angle(
                orientation, self.filtered_orientation)
            if position_jump > self.max_position_jump_m:
                return None, (
                    f'position jump rejected: {position_jump:.3f}m > '
                    f'{self.max_position_jump_m:.3f}m')
            if orientation_jump > self.max_orientation_jump_rad:
                return None, (
                    f'orientation jump rejected: {orientation_jump:.3f}rad > '
                    f'{self.max_orientation_jump_rad:.3f}rad')

            self.filtered_position = (
                (1.0 - self.position_filter_alpha) * self.filtered_position
                + self.position_filter_alpha * position)
            self.filtered_orientation = blend_quaternions(
                self.filtered_orientation,
                orientation,
                self.orientation_filter_alpha,
            )
        else:
            self.filtered_position = position
            self.filtered_orientation = orientation

        self.stable_count = min(
            self.stable_count + 1, self.stable_detection_count)
        filtered_pose = self.make_pose(
            pose.header.stamp,
            pose.header.frame_id,
            self.filtered_position,
            self.filtered_orientation,
        )
        return filtered_pose, None

    def publish_debug(
        self,
        frame: np.ndarray,
        source_message: Image,
        text: str,
        corners=None,
        ids=None,
        estimate=None,
        camera_matrix=None,
        distortion=None,
    ):
        """Publish an annotated BGR debug image."""
        if not self.publish_debug_image:
            return
        debug_frame = frame.copy()
        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
        if (
            self.draw_axes
            and estimate is not None
            and camera_matrix is not None
            and distortion is not None
        ):
            cv2.drawFrameAxes(
                debug_frame,
                camera_matrix,
                distortion,
                estimate.rvec,
                estimate.tvec,
                self.axis_length_m,
            )
        cv2.putText(
            debug_frame,
            text,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        debug_message = Image()
        debug_message.header = source_message.header
        debug_message.height = int(debug_frame.shape[0])
        debug_message.width = int(debug_frame.shape[1])
        debug_message.encoding = 'bgr8'
        debug_message.is_bigendian = 0
        debug_message.step = int(debug_frame.shape[1] * 3)
        debug_message.data = debug_frame.tobytes()
        self.debug_pub.publish(debug_message)

    def reject_frame(
        self,
        reason: str,
        frame=None,
        source_message=None,
        corners=None,
        ids=None,
        visible=False,
    ):
        """Invalidate the current frame and publish a diagnostic reason."""
        self.stable_count = 0
        self.publish_flags(visible, False)
        self.publish_status(reason)
        if frame is not None and source_message is not None:
            self.publish_debug(
                frame,
                source_message,
                reason,
                corners=corners,
                ids=ids,
            )

    def on_image(self, message: Image):
        """Process an EEF camera image and publish accepted target poses."""
        self.last_image_time = self.get_clock().now()
        camera_matrix, distortion = self.camera_model_for_image(message)
        if camera_matrix is None:
            self.reject_frame('Waiting for calibrated EEF camera_info')
            return

        try:
            frame = image_to_bgr(message)
        except (ValueError, cv2.error) as error:
            self.reject_frame(f'Image conversion failed: {error}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_clahe:
            gray = self.clahe.apply(gray)
        corners, ids, _ = self.detect_markers(gray)

        if ids is None or len(ids) == 0:
            self.reject_frame(
                'ArUco marker not visible',
                frame,
                message,
            )
            return

        selected, detected_ids = self.select_marker(corners, ids)
        if selected is None:
            self.reject_frame(
                f'ArUco marker rejected: detected_ids={detected_ids}',
                frame,
                message,
                corners,
                ids,
            )
            return

        marker_index, detected_id = selected
        image_points = np.asarray(
            corners[marker_index], dtype=np.float64).reshape((4, 2))
        estimate = estimate_square_pose(
            image_points,
            self.marker_size_m,
            camera_matrix,
            distortion,
        )
        if estimate is None:
            self.reject_frame(
                'ArUco pose estimation failed',
                frame,
                message,
                corners,
                ids,
            )
            return

        marker_z = float(estimate.tvec[2, 0])
        if marker_z < self.pose_z_min_m or marker_z > self.pose_z_max_m:
            self.reject_frame(
                f'ArUco depth rejected: z={marker_z:.3f}m',
                frame,
                message,
                corners,
                ids,
            )
            return
        if estimate.reprojection_error_px > self.max_reprojection_error_px:
            self.reject_frame(
                'ArUco reprojection rejected: '
                f'{estimate.reprojection_error_px:.2f}px',
                frame,
                message,
                corners,
                ids,
            )
            return

        source_frame = message.header.frame_id or self.camera_info_frame
        marker_rotation, _ = cv2.Rodrigues(estimate.rvec)
        marker_orientation = rotation_matrix_to_quaternion(marker_rotation)
        marker_position = estimate.tvec.reshape((3,))
        marker_pose = self.make_pose(
            message.header.stamp,
            source_frame,
            marker_position,
            marker_orientation,
        )
        self.marker_pose_pub.publish(marker_pose)

        object_position, object_orientation = apply_marker_offset(
            estimate.rvec,
            estimate.tvec,
            self.object_offset_xyz,
            self.object_offset_rpy,
        )
        object_pose_camera = self.make_pose(
            message.header.stamp,
            source_frame,
            object_position,
            object_orientation,
        )

        self.last_detection_time = self.get_clock().now()
        try:
            object_pose_target = self.transform_object_pose(object_pose_camera)
        except Exception as error:
            self.reject_frame(
                f'TF {source_frame}->{self.target_frame} failed: {error}',
                frame,
                message,
                corners,
                ids,
                visible=True,
            )
            return

        filtered_pose, filter_error = self.filter_pose(object_pose_target)
        if filtered_pose is None:
            self.reject_frame(
                filter_error,
                frame,
                message,
                corners,
                ids,
                visible=True,
            )
            return

        valid = self.stable_count >= self.stable_detection_count
        self.publish_flags(True, valid)
        if valid:
            self.object_pose_pub.publish(filtered_pose)

        status = (
            f'ArUco id={detected_id} z={marker_z:.3f}m '
            f'error={estimate.reprojection_error_px:.2f}px '
            f'stable={self.stable_count}/{self.stable_detection_count} '
            f'valid={valid}')
        self.publish_status(status)
        self.publish_debug(
            frame,
            message,
            status,
            corners,
            ids,
            estimate,
            camera_matrix,
            distortion,
        )

    def reset_filter_if_stale(self, now):
        """Clear the temporal filter after a prolonged detection loss."""
        if self.last_detection_time is None:
            return
        age = (now - self.last_detection_time).nanoseconds * 1.0e-9
        if age > self.filter_reset_timeout_s:
            self.filtered_position = None
            self.filtered_orientation = None
            self.stable_count = 0

    def on_timeout_timer(self):
        """Invalidate output when camera or marker updates become stale."""
        now = self.get_clock().now()
        self.reset_filter_if_stale(now)

        if self.last_image_time is None:
            self.publish_flags(False, False)
            self.publish_status('Waiting for EEF camera image')
            return

        image_age = (now - self.last_image_time).nanoseconds * 1.0e-9
        if image_age > self.image_timeout_s:
            self.publish_flags(False, False)
            self.publish_status(
                f'EEF camera image timeout: age={image_age:.2f}s')
            return

        if self.last_detection_time is None:
            return
        detection_age = (
            now - self.last_detection_time).nanoseconds * 1.0e-9
        if detection_age > self.detection_timeout_s:
            self.publish_flags(False, False)


def main(args=None):
    """Run the ArUco-only EEF vision node."""
    rclpy.init(args=args)
    node = EefVisionNode()
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
