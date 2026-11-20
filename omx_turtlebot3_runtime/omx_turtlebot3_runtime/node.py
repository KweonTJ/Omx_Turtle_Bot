"""ROS supervisor: real encoders + synchronized RGB -> PPO -> guarded OpenCR."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import time
import math

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from ament_index_python.packages import get_package_share_directory

from .core import Fault, PoseCache, ObjectPose, Phase, observation, commissioning_issues
from .opencr import OpenCR
from .planning import Planner
from .policy import Actor
from .mission import Mission
from .vision import Detector, synchronized_joints, paired_camera_info


def json_value(value):
    return value.tolist() if hasattr(value, 'tolist') else str(value)


class Runtime(Node):
    def __init__(self, args, transport_factory=OpenCR):
        super().__init__('omx_v6_runtime')
        self.args = args
        self.cfg = yaml.safe_load(args.config.read_text())
        validate_config(self.cfg)
        self.training = yaml.safe_load((args.model_dir/'training_config.yaml').read_text())
        if args.execute and commissioning_issues(self.cfg):
            raise Fault('Execute blocked: '+', '.join(commissioning_issues(self.cfg)))
        self.actor = Actor(args.model_dir, args.backend, args.device)
        self.planner = Planner(args.scene, self.training, self.cfg)
        self.mission = Mission(self.cfg, self.training, self.planner, self.actor)
        self.detector, self.bridge = Detector(self.cfg['vision']), CvBridge()
        self.cache, self.history = PoseCache(), deque(maxlen=200)
        self.camera_info = None
        self.info_frame_binding_reported = False
        self.camera_counts = {'info_received': 0, 'image_received': 0,
                              'accepted_frames': 0, 'pose_0': 0, 'pose_1': 0}
        self.last_image_stamp = -math.inf
        self.deadman, self.deadman_time = False, -math.inf
        self.seen_requests = set()
        self.fault = None
        self.plan_checked = False
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.started = time.monotonic()
        args.log_dir.mkdir(parents=True, exist_ok=False)
        self.log = (args.log_dir/'runtime.jsonl').open('x', buffering=1)
        self.status_pub = self.create_publisher(String, '/omx_v6/status', 10)
        self.joint_pub = self.create_publisher(JointState, '/omx_v6/joint_states', 10)
        self.image_sub = Subscriber(self, Image, self.cfg['vision']['image_topic'], qos_profile=qos_profile_sensor_data)
        self.info_sub = Subscriber(self, CameraInfo, self.cfg['vision']['camera_info_topic'], qos_profile=qos_profile_sensor_data)
        self.camera_sync = TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.camera_sync.registerCallback(self.on_camera_pair)
        self.create_subscription(String, '/omx_v6/request', self.on_request, 10)
        self.create_subscription(Bool, '/omx_v6/deadman', self.on_deadman, 10)
        self.create_subscription(Bool, '/omx_v6/cancel', self.on_cancel, 10)
        self.transport = transport_factory(self.cfg, execute=args.execute)
        self.history.append(self.transport.last)
        self.timer = self.create_timer(self.cfg['control']['period_s'], self.tick)
        self.emit('READY', execute=args.execute, commissioning_issues=commissioning_issues(self.cfg),
                  estimator_semantics='physical_proxy_not_mujoco_contact', motor_writes=0,
                  command_route=self.cfg['control'].get('command_route', 'direct'),
                  robot_urdf_sha256=self.cfg['vision'].get('robot_urdf_sha256'),
                  policy_from_base=self.planner.policy_from_base)

    def emit(self, event, **payload):
        record = dict(event=event, monotonic=time.monotonic(), **payload)
        text = json.dumps(record, default=json_value, allow_nan=False)
        self.log.write(text+'\n')
        if event != 'SAMPLE':
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def stop(self, reason):
        self.fault = str(reason)
        self.mission.abort(self.fault)
        try:
            self.transport.hold()
        except Exception as exc:
            self.fault += '; HOLD_FAILED: '+str(exc)
        self.emit('STOPPED', reason=self.fault, motor_writes=self.transport.writes)

    def on_deadman(self, message):
        self.deadman, self.deadman_time = bool(message.data), time.monotonic()
        if not self.deadman and self.mission.active:
            self.stop('Operator deadman released')

    def on_cancel(self, message):
        if message.data:
            self.stop('Operator cancel; gripper not released')

    def on_camera_pair(self, image, info):
        try:
            bound = paired_camera_info(image, info, self.cfg['vision']['optical_frame'])
            if not info.header.frame_id and not self.info_frame_binding_reported:
                self.emit('CAMERA_INFO_FRAME_BOUND', frame=bound.header.frame_id,
                          source='exact_timestamp_image_pair', extrinsics_verified=False)
                self.info_frame_binding_reported = True
            self.on_info(bound)
            self.on_image(image)
        except Exception as exc:
            self.camera_info = None
            if self.mission.active:
                self.stop('Camera pair: '+str(exc))
            else:
                self.emit('VISION_REJECTED', reason=str(exc))

    def on_info(self, info):
        self.camera_counts['info_received'] += 1
        key = (info.width, info.height, info.header.frame_id, tuple(info.k), tuple(info.d), info.distortion_model)
        old = getattr(self, 'calibration_key', None)
        self.camera_info = None
        if old is not None and key != old:
            self.cache = PoseCache()
            if self.mission.active:
                self.stop('Camera calibration changed during mission')
        self.calibration_key = key
        matrix = np.asarray(info.k).reshape(3, 3)
        if ((info.width, info.height) != (320, 240)
                or info.header.frame_id != self.cfg['vision']['optical_frame']
                or info.distortion_model not in ('plumb_bob', 'rational_polynomial')
                or len(info.d) not in (4, 5, 8, 12, 14)
                or not np.isfinite(matrix).all() or not np.isfinite(info.d).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])
                or info.binning_x > 1 or info.binning_y > 1
                or any((info.roi.width, info.roi.height, info.roi.x_offset, info.roi.y_offset))):
            if self.camera_counts['info_received'] == 1 or key != old:
                self.emit('CAMERA_INFO_REJECTED', frame=info.header.frame_id,
                          size=[info.width, info.height], reason='Calibration contract mismatch')
            return
        self.camera_info = info
        if self.camera_counts['info_received'] == 1 or key != old:
            self.emit('CAMERA_INFO_ACCEPTED', frame=info.header.frame_id, size=[info.width, info.height])

    def on_image(self, message):
        self.camera_counts['image_received'] += 1
        try:
            if self.camera_info is None:
                return
            raw_stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            ros_now = self.get_clock().now().nanoseconds/1e9
            now = time.monotonic()
            age = ros_now-raw_stamp
            if raw_stamp <= self.last_image_stamp or not 0 <= age <= self.cfg['vision']['max_image_age_s']:
                return
            self.last_image_stamp = raw_stamp
            if message.header.frame_id != self.camera_info.header.frame_id or (message.width, message.height) != (320, 240):
                raise Fault('Camera image/CameraInfo contract mismatch')
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            q = synchronized_joints(self.history, now-age, self.cfg['vision']['joint_sync_tolerance_s'])
            matrix = np.asarray(self.camera_info.k).reshape(3, 3)
            poses = self.detector.detect(image, matrix, np.asarray(self.camera_info.d))
            self.cache.begin_frame(now-age)
            self.camera_counts['accepted_frames'] += 1
            for marker, position, rotation, error in poses:
                center, quaternion = self.planner.object_from_marker(q, position, rotation)
                self.cache.update(ObjectPose(marker, now-age, center, quaternion, error), now)
                self.camera_counts[f'pose_{marker}'] += 1
            self.emit('VISION', ids=[p[0] for p in poses], stamp=now-age,
                      poses=[None if p is None else vars(p) for p in self.cache.items],
                      extrinsics_verified=self.cfg['commissioning']['camera_extrinsics_verified'])
        except Exception as exc:
            if self.mission.active:
                self.stop('Vision: '+str(exc))
            else:
                self.emit('VISION_REJECTED', reason=str(exc))

    def on_request(self, message):
        try:
            if self.fault or self.pending is not None or self.mission.active or self.mission.done:
                raise Fault('Runtime is latched/active; explicit restart required')
            if len(message.data) > 2048:
                raise Fault('Request too long')
            request = json.loads(message.data)
            if not isinstance(request, dict) or set(request) != {'request_id', 'object_id'}:
                raise Fault('Expected request_id and physical object_id only')
            rid = request['request_id']
            if not isinstance(rid, str) or not rid.strip() or len(rid) > 128 or rid in self.seen_requests:
                raise Fault('Invalid/replayed request ID')
            if len(self.seen_requests) >= 4096:
                raise Fault('Request history full')
            if commissioning_issues(self.cfg):
                raise Fault('Commissioning incomplete: '+', '.join(commissioning_issues(self.cfg)))
            now = time.monotonic()
            if not self.deadman or now-self.deadman_time > self.cfg['control']['deadman_timeout_s']:
                raise Fault('Fresh operator deadman required')
            self.seen_requests.add(rid)
            # Separate model/data in the worker: never race mj_forward with image FK.
            snapshot, sample = deepcopy(self.cache), deepcopy(self.transport.last)
            def prepare():
                planner = Planner(self.args.scene, self.training, self.cfg)
                mission = Mission(self.cfg, self.training, planner, self.actor)
                mission.start(request['object_id'], sample, snapshot, now)
                return mission
            self.pending = (self.pool.submit(prepare), request, snapshot)
            self.emit('PLANNING', request_id=rid, motor_writes=0)
        except Exception as exc:
            if self.mission.active:
                self.stop('Request preflight: '+str(exc))
            else:
                self.emit('REQUEST_REJECTED', reason=str(exc))

    def tick(self):
        if self.fault:
            return
        try:
            sample = self.transport.read()
            self.history.append(sample)
            joints = JointState()
            joints.header.stamp = self.get_clock().now().to_msg()
            joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint', 'gripper_right_joint']
            joints.position = [*sample.q, sample.finger, sample.finger]
            joints.velocity = [*sample.dq, sample.finger_velocity, sample.finger_velocity]
            # Raw current is not JointState effort (N or Nm); report it only in telemetry.
            self.joint_pub.publish(joints)
            now = time.monotonic()
            if (getattr(self.args, 'check_plan_id', None) is not None and not self.plan_checked
                    and all(self.cache.fresh(i, now, .2) for i in (0, 1))
                    and all(self.camera_counts[f'pose_{i}'] >= 3 for i in (0, 1))):
                self.check_plan_only(sample, now)
                return
            if self.pending is not None and self.pending[0].done():
                future, request, snapshot = self.pending
                self.pending = None
                candidate_mission = future.result()
                if (np.max(np.abs(sample.q-candidate_mission.target)) > .01
                        or not all(self.cache.fresh(i, now) for i in (0, 1))
                        or not self.deadman or now-self.deadman_time > self.cfg['control']['deadman_timeout_s']
                        or any(np.linalg.norm(self.cache.items[i].position-snapshot.items[i].position) > .005 for i in (0, 1))):
                    raise Fault('Planning inputs changed/expired; no motor command sent')
                self.mission = candidate_mission
                self.mission.start_time = self.mission.phase_time = self.mission.last_time = now
                if self.args.execute:
                    self.transport.arm()
                    self.mission.last_time = time.monotonic()
                    self.emit('MISSION_STARTED', **request)
                    return  # Acquire fresh telemetry on the next cycle.
                self.mission.active, self.mission.done = False, True
                self.emit('PLAN_ONLY', place=self.mission.place, motor_writes=0)
            if self.mission.active:
                if not self.deadman or now-self.deadman_time > self.cfg['control']['deadman_timeout_s']:
                    raise Fault('Operator deadman expired')
                q, g, obs = self.mission.step(sample, self.cache, now)
                self.transport.command(q, g)
                self.emit('CONTROL', observation=obs, target_q=q, target_finger=g,
                          phase=self.mission.phase.name, motor_writes=self.transport.writes,
                          trajectory=self.transport.last_trajectory)
                if self.mission.done:
                    self.transport.hold()
                    self.emit('MISSION_DONE', result=self.mission.reason)
            self.emit('SAMPLE', **vars(sample), phase=self.mission.phase.name,
                      active=self.mission.active, grasp=self.mission.evidence.grasp_confirmed,
                      placed=self.mission.evidence.placed, evidence=self.mission.evidence.reason,
                      motor_writes=self.transport.writes)
        except Exception as exc:
            self.stop(exc)

    def check_plan_only(self, sample, now):
        """Fresh physical input -> actor + complete path check, without arming."""
        if self.args.execute or self.transport.execute:
            raise Fault('Plan inspection must never use an executing transport')
        self.plan_checked = True
        selected = self.args.check_plan_id
        planner = Planner(self.args.scene, self.training, self.cfg)
        cache = deepcopy(self.cache)
        camera, rotation = planner.camera_pose(sample.q)
        goal = np.asarray(self.training['motion']['scan_pose'])
        place = np.r_[self.cfg['estimation']['deck_center_xy'], self.cfg['estimation']['deck_top_z']]
        report = dict(selected_id=selected, q=sample.q, camera_center=camera,
                      optical_forward=rotation[:, 2], objects=[vars(p) for p in cache.items],
                      policy_from_base=planner.policy_from_base, motor_writes=self.transport.writes,
                      physical_commissioning_issues=commissioning_issues(self.cfg))
        try:
            mission = Mission(self.cfg, self.training, planner, self.actor)
            mission.start(selected, sample, cache, now)
            goal, place = mission.goal, mission.place
            report.update(plan_status='nominal_path_passed_not_execution_authorized',
                          waypoints={p.name: [q.tolist() for q in points] for p, points in mission.plans.items()})
        except Exception as exc:
            report.update(plan_status='rejected', reason=str(exc))
        eef = planner.forward(sample.q, sample.finger)
        obs = observation(sample, eef, goal, Phase.RECOGNIZE, 0, 1, cache, selected,
                          place, False, False, np.zeros(4), now, 0, now)
        report.update(observation=obs, actor_action=self.actor(obs),
                      inference_only=True, inference_not_a_joint_command=True)
        self.emit('MODEL_PLAN_CHECK', **report)

    def finish(self):
        try:
            self.transport.close()
        finally:
            self.emit('EXIT', motor_writes=self.transport.writes, camera_counts=self.camera_counts)
            self.log.close()
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.destroy_node()


def validate_config(cfg):
    if cfg['schema_version'] != 1 or cfg['model_version'] != 'omx_turtlebot3_v6':
        raise Fault('Runtime configuration version mismatch')
    v = cfg['vision']
    if (v['physical_ids'] != [0, 1] or v['policy_labels'] != [1, 2]
            or v['dictionary'] != 'DICT_4X4_50' or v['marker_size_m'] != .04):
        raise Fault('Physical marker/policy slot binding mismatch')
    for name in ('optical_rotation_in_link5', 'object_to_marker_rotation'):
        r = np.asarray(v[name], dtype=float)
        if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T@r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
            raise Fault('Invalid calibration rotation')
    c = cfg['control']
    if c.get('command_route', 'direct') not in ('direct', 'mp_control_equivalent'):
        raise Fault('Invalid command route')
    for key in ('period_s', 'max_cycle_s', 'joint_velocity_rad_s', 'joint_acceleration_rad_s2',
                'gripper_velocity_m_s', 'max_tracking_error_rad', 'pose_timeout_s', 'deadman_timeout_s'):
        if not isinstance(c[key], (float, int)) or not math.isfinite(c[key]) or c[key] <= 0:
            raise Fault('Invalid control limit: '+key)
    if c['joint_velocity_rad_s'] > .72 or c['gripper_velocity_m_s'] > .02 or c['max_cycle_s'] > .2:
        raise Fault('Control limit exceeds runtime envelope')


def main(argv=None):
    root = Path(get_package_share_directory('omx_turtlebot3_runtime'))
    models = Path(get_package_share_directory('omx_turtlebot3_policy'))/'models/omx_turtlebot3_v6'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root/'config/runtime.yaml')
    parser.add_argument('--scene', type=Path, default=root/'assets/scene.xml')
    parser.add_argument('--model-dir', type=Path, default=models)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--check-plan-id', type=int, choices=[0, 1],
                        help='Read-only actor and path check on fresh camera/encoder data; never arm')
    parser.add_argument('--backend', choices=['torchscript', 'onnx'], default='torchscript')
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cpu')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--log-dir', type=Path, default=Path('/tmp')/f'omx_v6_runtime_{time.time_ns()}')
    args, ros_args = parser.parse_known_args(argv)
    if args.execute and args.check_plan_id is not None:
        parser.error('--check-plan-id cannot be combined with --execute')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Runtime(args)
        while (rclpy.ok() and not node.plan_checked
               and (args.duration <= 0 or time.monotonic()-node.started < args.duration)):
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
