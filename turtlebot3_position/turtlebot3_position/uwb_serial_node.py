"""Publish ESP32 UWB coordinates with a fresh odometry yaw."""

import math
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import parse_position, quaternion_to_yaw, yaw_to_quaternion

try:
    import serial
except ImportError:  # The node stays alive and reports the missing dependency.
    serial = None


class UwbSerialNode(Node):
    """Read calculated X/Y coordinates from an ESP32 over serial."""

    def __init__(self):
        super().__init__('uwb_serial_node')

        defaults = {
            'port': '/dev/ttyUSB0',
            'baud': 115200,
            'reconnect_sec': 2.0,
            'frame_id': 'uwb_map',
            'pose_topic': '/turtlebot3_position/pose',
            'uwb_valid_topic': '/turtlebot3_position/uwb/valid',
            'uwb_raw_topic': '/turtlebot3_position/uwb/raw',
            'odom_topic': '/odom',
            'include_odom_yaw': True,
            'odom_timeout_sec': 1.0,
            'position_variance_x': 0.0121,
            'position_variance_y': 0.0121,
            'yaw_variance': 0.04,
            'yaw_unavailable_variance': 9.8696,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self._port = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._reconnect_sec = max(
            0.0, float(self.get_parameter('reconnect_sec').value))
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._include_odom_yaw = bool(
            self.get_parameter('include_odom_yaw').value)
        self._odom_timeout_sec = max(
            0.0, float(self.get_parameter('odom_timeout_sec').value))
        self._position_variance_x = float(
            self.get_parameter('position_variance_x').value)
        self._position_variance_y = float(
            self.get_parameter('position_variance_y').value)
        self._yaw_variance = float(
            self.get_parameter('yaw_variance').value)
        self._yaw_unavailable_variance = float(
            self.get_parameter('yaw_unavailable_variance').value)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value), 5)
        self.valid_pub = self.create_publisher(
            Bool, str(self.get_parameter('uwb_valid_topic').value), 5)
        self.raw_pub = self.create_publisher(
            String, str(self.get_parameter('uwb_raw_topic').value), 5)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, 5)

        self._yaw = 0.0
        self._odom_time = None
        self._odom_lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._serial_device = None
        self._stop_event = threading.Event()

        self.valid_pub.publish(Bool(data=False))
        self._thread = threading.Thread(
            target=self._read_loop, name='uwb_serial_reader', daemon=True)
        self._thread.start()

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw, valid = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if not valid:
            return
        with self._odom_lock:
            self._yaw = yaw
            self._odom_time = self.get_clock().now()

    def _pose_message(self, x, y):
        """Build a pose in metres; yaw is used only while odometry is fresh."""
        msg = PoseWithCovarianceStamped()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        with self._odom_lock:
            odom_time = self._odom_time
            odom_yaw = self._yaw
        yaw_fresh = odom_time is not None and (
            (now - odom_time).nanoseconds / 1e9 <= self._odom_timeout_sec)
        use_fresh_yaw = self._include_odom_yaw and yaw_fresh
        yaw = odom_yaw if use_fresh_yaw else 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        orientation = msg.pose.pose.orientation
        orientation.x = qx
        orientation.y = qy
        orientation.z = qz
        orientation.w = qw

        covariance = [0.0] * 36
        covariance[0] = self._position_variance_x
        covariance[7] = self._position_variance_y
        covariance[14] = math.pi ** 2
        covariance[21] = math.pi ** 2
        covariance[28] = math.pi ** 2
        covariance[35] = (
            self._yaw_variance if use_fresh_yaw
            else self._yaw_unavailable_variance)
        msg.pose.covariance = covariance
        return msg

    def _read_loop(self):
        if serial is None:
            self.get_logger().error(
                'pyserial is unavailable; install the python3-serial package')
            self.valid_pub.publish(Bool(data=False))
            return

        while rclpy.ok() and not self._stop_event.is_set():
            device = None
            try:
                device = serial.Serial(
                    port=self._port, baudrate=self._baud, timeout=0.5)
                with self._serial_lock:
                    self._serial_device = device
                self.get_logger().info(
                    f'UWB serial connected: {self._port} @ {self._baud}')

                while rclpy.ok() and not self._stop_event.is_set():
                    data = device.readline()
                    if not data:
                        continue
                    line = data.decode('utf-8', errors='replace').strip()
                    self.raw_pub.publish(String(data=line))
                    point = parse_position(line)
                    if point is None:
                        self.valid_pub.publish(Bool(data=False))
                        continue
                    self.pose_pub.publish(self._pose_message(*point))
                    self.valid_pub.publish(Bool(data=True))
            except (serial.SerialException, OSError, ValueError) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning(
                        f'UWB serial unavailable: {exc}; retrying in '
                        f'{self._reconnect_sec:.1f}s')
                    self.valid_pub.publish(Bool(data=False))
                    self._stop_event.wait(self._reconnect_sec)
            finally:
                with self._serial_lock:
                    if self._serial_device is device:
                        self._serial_device = None
                if device is not None and device.is_open:
                    try:
                        device.close()
                    except (serial.SerialException, OSError):
                        pass

    def destroy_node(self):
        self._stop_event.set()
        with self._serial_lock:
            device = self._serial_device
        if device is not None and device.is_open:
            try:
                device.close()
            except (serial.SerialException, OSError):
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbSerialNode()
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
