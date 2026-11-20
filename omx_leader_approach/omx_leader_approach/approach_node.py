"""Host handoff -> shelf alignment -> IMU-held straight approach -> arm-ready.

Publishes a proposed Twist on a dedicated topic. The existing cmd_vel mux owns
the robot output. Creating or launching this node alone does not select it.
"""
import json
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .alignment_core import AlignmentGate, HeadingObservation, imu_reference, wrap


def yaw(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(v).all() or abs(np.linalg.norm(v)-1) > .02:
        raise ValueError('Invalid orientation')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class ApproachNode(Node):
    def __init__(self):
        super().__init__('leader_approach')
        defaults = {'imu_topic': '/imu_broadcaster/imu',
                    'odom_topic': '/diff_drive_controller/odom', 'objects_topic': '/perception/objects',
                    'command_topic': '/leader_approach/cmd_vel', 'odom_frame': 'odom',
                    'distance_m': .382, 'speed_mps': .055, 'sensor_timeout_s': .3,
                    'overview_timeout_s': 10., 'alignment_timeout_s': 16., 'drive_timeout_s': 18.}
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not 0 < self.cfg['distance_m'] <= .6 or not 0 < self.cfg['speed_mps'] <= .06:
            raise ValueError('Expected staging distance <=0.6m and speed <=0.06m/s')
        if not 0 < self.cfg['sensor_timeout_s'] <= .5:
            raise ValueError('Expected sensor timeout <=0.5s')
        self.gate = AlignmentGate(max_frame_age=self.cfg['sensor_timeout_s'])
        self.command = self.create_publisher(Twist, self.cfg['command_topic'], 1)
        self.status = self.create_publisher(String, '/leader_approach/status', 1)
        self.select = self.create_publisher(String, '/perception/target_request', 10)
        self.create_subscription(Imu, self.cfg['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, 10)
        self.create_subscription(String, self.cfg['objects_topic'], self.on_objects, 10)
        self.create_subscription(String, '/leader_approach/request', self.on_request, 10)
        self.state, self.request = 'IDLE', None
        self.used_requests = set()
        self.imu_stamp = self.odom_stamp = self.overview_stamp = -math.inf
        self.imu_angle = self.omega = self.odom_angle = self.speed = 0.
        self.position = np.zeros(2)
        self.ready_ticks = 0
        self.target_imu_yaw = None
        self.heading_error = None
        self.state_since = self.get_clock().now().nanoseconds/1e9
        self.create_timer(.05, self.tick)

    def seconds(self, header):
        return Time.from_msg(header.stamp).nanoseconds/1e9

    def on_imu(self, message):
        try:
            if message.orientation_covariance[0] < 0 or message.angular_velocity_covariance[0] < 0:
                raise ValueError('IMU orientation or gyro unavailable')
            angle = yaw(message.orientation)
            if not np.isfinite(message.angular_velocity.z):
                raise ValueError('Invalid gyro')
            stamp = self.seconds(message.header)
            if stamp <= self.imu_stamp:
                return
            self.imu_angle, self.omega = angle, message.angular_velocity.z
            self.imu_stamp = stamp
        except ValueError:
            self.imu_stamp = -math.inf

    def on_odom(self, message):
        try:
            if message.header.frame_id != self.cfg['odom_frame']:
                raise ValueError('Host target and wheel odometry must share odom_frame')
            stamp = self.seconds(message.header)
            if stamp <= self.odom_stamp:
                return
            self.odom_angle = yaw(message.pose.pose.orientation)
            p = message.pose.pose.position
            values = np.array([p.x, p.y, message.twist.twist.linear.x])
            if not np.isfinite(values).all():
                raise ValueError('Invalid odometry')
            self.position, self.speed = values[:2], float(values[2])
            self.odom_stamp = stamp
        except ValueError:
            self.odom_stamp = -math.inf

    def on_objects(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Expected an object list envelope')
            records = value.get('objects', [])
            if not isinstance(records, list) or not all(isinstance(r, dict) for r in records):
                raise ValueError('Expected object records')
            ids = [r['marker_id'] for r in records if r.get('pose_valid') and r.get('usable_2d') and not r.get('duplicate', False)]
            complete = all(ids.count(i) == 1 for i in (1, 2, 3, 4))
            stamp = value['stamp_ns']/1e9
            self.overview_stamp = stamp if (value.get('fresh') and value.get('overview_ready') and complete and math.isfinite(stamp)) else -math.inf
        except (ValueError, KeyError, TypeError):
            self.overview_stamp = -math.inf

    def on_request(self, message):
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                return
            if request.get('command') == 'STOP':
                self.state, self.request = 'IDLE', None
                self.command.publish(Twist())
                return
            if request.get('command') != 'START' or type(request.get('object_id')) is not int or request['object_id'] not in (1, 2, 3, 4):
                return
            request_id = request.get('request_id')
            if not isinstance(request_id, str) or not request_id or request_id in self.used_requests:
                return
            # No silent zero or marker-plane fallback: IMU alone cannot know
            # the shelf orientation. The host supplies a radian yaw in odom.
            target = request.get('target_yaw_odom')
            if type(target) not in (int, float) or not math.isfinite(target):
                self.get_logger().warning('START rejected: target_yaw_odom is required')
                return
            if request.get('odom_frame') != self.cfg['odom_frame']:
                self.get_logger().warning('START rejected: odom_frame mismatch')
                return
            if self.state not in ('IDLE', 'READY_FOR_ARM', 'FAULT'):
                return
            self.used_requests.add(request_id)
            self.request, self.state = request, 'WAIT_OVERVIEW'
            self.gate.reset()
            self.ready_ticks = 0
            self.overview_stamp = -math.inf
            self.last_ready_odom_stamp = -math.inf
            self.target_imu_yaw = self.heading_error = None
            self.state_since = self.get_clock().now().nanoseconds/1e9
            self.select.publish(String(data=json.dumps({'command': 'SELECT', 'request_id': request_id,
                                                       'object_id': request['object_id']})))
        except (ValueError, TypeError):
            return

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        cmd, reason, travelled = Twist(), self.state, 0.
        imu_ok = 0 <= now-self.imu_stamp <= self.cfg['sensor_timeout_s']
        odom_ok = 0 <= now-self.odom_stamp <= self.cfg['sensor_timeout_s']
        if self.state not in ('IDLE', 'FAULT') and (not odom_ok or not imu_ok):
            reason = 'WAIT_FRESH_ODOM_IMU'
            self.ready_ticks = 0
            self.gate.reset()
        elif self.state == 'WAIT_OVERVIEW':
            if 0 <= now-self.overview_stamp <= .4 and self.overview_stamp > self.state_since:
                # Require near-synchronous, stopped host handoff before fixing
                # the offset between the two yaw frames.
                if abs(self.imu_stamp-self.odom_stamp) > .1 or abs(self.speed) > .008 or abs(self.omega) > .025:
                    reason = 'WAIT_STOPPED_HANDOFF'
                else:
                    self.target_imu_yaw = imu_reference(self.request['target_yaw_odom'], self.odom_angle, self.imu_angle)
                    self.state, self.state_since = 'ALIGN_PARALLEL', now
            elif now-self.state_since > self.cfg['overview_timeout_s']:
                self.state, reason = 'FAULT', 'OVERVIEW_TIMEOUT'
        elif self.state == 'ALIGN_PARALLEL':
            self.heading_error = wrap(self.target_imu_yaw-self.imu_angle)
            estimate = HeadingObservation(True, self.heading_error, 'HOST_YAW_IMU_ODOM')
            cmd.angular.z, ready, reference = self.gate.update(estimate, self.imu_stamp, now, self.imu_angle, self.omega)
            reason = estimate.reason
            if ready:
                self.yaw_reference = reference
                self.start = self.position.copy()
                self.axis = np.array([math.cos(self.request['target_yaw_odom']), math.sin(self.request['target_yaw_odom'])])
                self.state, self.state_since = 'DRIVE_STRAIGHT', now
                cmd = Twist()
            elif now-self.state_since > self.cfg['alignment_timeout_s']:
                self.state, cmd, reason = 'FAULT', Twist(), 'ALIGNMENT_TIMEOUT'
        elif self.state == 'DRIVE_STRAIGHT':
            error = wrap(self.yaw_reference-self.imu_angle)
            self.heading_error = error
            delta = self.position-self.start
            travelled = float(delta @ self.axis)
            lateral = float(delta @ np.array([-self.axis[1], self.axis[0]]))
            cmd.angular.z = float(np.clip(1.8*error, -.18, .18))
            remaining = self.cfg['distance_m']-travelled
            if remaining > .002 and abs(error) < math.radians(2.):
                cmd.linear.x = min(self.cfg['speed_mps'], max(.01, remaining*.8))
            stopped = remaining <= .002 and abs(self.speed) < .008 and abs(self.omega) < .025 and abs(error) < math.radians(.5)
            if not stopped:
                self.ready_ticks = 0
            elif self.odom_stamp > self.last_ready_odom_stamp:
                self.ready_ticks += 1
                self.last_ready_odom_stamp = self.odom_stamp
            if abs(lateral) > .02 or remaining < -.01:
                self.state, cmd, reason = 'FAULT', Twist(), 'ODOMETRY_PATH_DEVIATION'
            elif self.ready_ticks >= 10:
                self.state, cmd = 'READY_FOR_ARM', Twist()
            elif now-self.state_since > self.cfg['drive_timeout_s']:
                self.state, cmd, reason = 'FAULT', Twist(), 'DRIVE_TIMEOUT'
        self.command.publish(cmd)
        self.status.publish(String(data=json.dumps({'state': self.state, 'reason': reason,
                          'request_id': None if self.request is None else self.request['request_id'],
                          'ready_for_arm': self.state == 'READY_FOR_ARM' and imu_ok and odom_ok,
                          'travelled_m': travelled, 'heading_source': 'host_yaw_imu_odom', 'depth_used': False,
                          'heading_error_rad': self.heading_error, 'odom_frame': self.cfg['odom_frame'],
                          'target_yaw_odom': None if self.request is None else self.request['target_yaw_odom']}, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
