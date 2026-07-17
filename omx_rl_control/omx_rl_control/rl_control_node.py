#!/usr/bin/env python3
"""Run the trained arm-only residual PPO policy on ROS 2 hardware."""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .action_limiter import ActionLimitConfig
from .action_limiter import ActionLimiter
from .gripper_manager import GripperManager
from .kinematics import KinematicsError
from .kinematics import OpenManipulatorKinematics
from .model_contract import ContractError
from .model_contract import load_policy_contract
from .model_contract import validate_policy_spaces
from .observation_builder import ObservationBuilder
from .observation_builder import ObservationConfig
from .observation_builder import wrap_to_pi
from .reference_controller import ReferenceController
from .state_machine import BASE_HOLD_STATES
from .state_machine import BASE_INTERLOCK_STATES
from .state_machine import policy_phase
from .state_machine import RuntimeState


class RlControlNode(Node):
    """Coordinate perception, PPO arm motion, and deterministic gripper IO."""

    def __init__(self):
        super().__init__('rl_control_node')
        self._declare_parameters()

        self.joint_names = tuple(self._parameter('joint_names'))
        self.gripper_joint_name = str(
            self._parameter('gripper_joint_name'))
        self.target_frame_id = str(self._parameter('target_frame_id'))
        if not self.target_frame_id:
            raise ValueError('target_frame_id must not be empty')
        self.joint_low = self._vector('joint_low', 4)
        self.joint_high = self._vector('joint_high', 4)
        self.control_period_s = 1.0 / float(
            self._parameter('control_rate_hz'))
        self.trajectory_time_s = float(
            self._parameter('trajectory_time_s'))
        self.inference_deadline_s = float(
            self._parameter('inference_deadline_s'))
        self.inference_deadline_miss_limit = int(
            self._parameter('inference_deadline_miss_limit'))
        if self.inference_deadline_miss_limit <= 0:
            raise ValueError('inference_deadline_miss_limit must be positive')
        self.status_period_s = float(self._parameter('status_period_s'))
        self.release_settle_time_s = float(
            self._parameter('release_settle_time_s'))
        if self.release_settle_time_s < 0.0:
            raise ValueError('release_settle_time_s must be non-negative')

        self.contract = None
        self.policy = None
        self.startup_error = ''
        self._load_policy()

        action_scale = (
            np.asarray(self.contract.action_scale, dtype=np.float64)
            if self.contract is not None
            else self._vector('action_scale', 4)
        )
        filter_coefficient = (
            self.contract.action_filter_coefficient
            if self.contract is not None
            else float(self._parameter('action_filter_coefficient'))
        )
        residual_scale = (
            self.contract.residual_action_scale
            if self.contract is not None
            else float(self._parameter('residual_action_scale'))
        )
        stay_joints = (
            np.asarray(self.contract.stay_joint_positions, dtype=np.float64)
            if self.contract is not None
            else self._vector('stay_joint_positions', 4)
        )

        self.kinematics = OpenManipulatorKinematics(
            self.joint_low,
            self.joint_high,
            self._vector('ik_initial_guess', 4),
        )
        self.observation_builder = ObservationBuilder(ObservationConfig(
            joint_low=self.joint_low,
            joint_high=self.joint_high,
            joint_velocity_scale=self._vector(
                'joint_velocity_scale', 4),
            workspace_min=self._vector('workspace_min', 3),
            workspace_max=self._vector('workspace_max', 3),
            arm_base_xy=self._vector('arm_base_xy', 2),
            policy_frame_offset=self._vector(
                'policy_frame_offset', 3),
            gripper_close=float(self._parameter('gripper_close')),
            gripper_open=float(self._parameter('gripper_open')),
            gripper_velocity_scale=float(
                self._parameter('gripper_velocity_scale')),
        ))
        waypoints = np.asarray([
            self._vector('approach_waypoint_1', 4),
            self._vector('approach_waypoint_2', 4),
        ])
        self.reference = ReferenceController(
            self.kinematics,
            stay_joints,
            action_scale,
            waypoints,
            waypoint_tolerance=float(
                self._parameter('approach_waypoint_tolerance')),
            pregrasp_height_offset=float(
                self._parameter('pregrasp_height_offset')),
            action_limit=float(self._parameter('reference_action_limit')),
            final_action_limit=float(
                self._parameter('final_approach_action_limit')),
            target_update_min_m=float(
                self._parameter('target_update_min_m')),
        )
        self.action_limiter = ActionLimiter(ActionLimitConfig(
            joint_low=self.joint_low,
            joint_high=self.joint_high,
            action_scale=action_scale,
            filter_coefficient=filter_coefficient,
            residual_scale=residual_scale,
            control_period_s=self.control_period_s,
            max_velocity=self._vector('max_joint_velocity', 4),
            max_acceleration=self._vector('max_joint_acceleration', 4),
        ))

        self.state = RuntimeState.NOT_READY
        self.state_enter_time = time.monotonic()
        self.state_detail = 'waiting for policy and robot state'
        self.pending_command = ''
        self.grasped = False
        self.external_grasp_confirmed = False
        self.previous_action = np.zeros(4, dtype=np.float64)
        self.stable_gate_count = 0
        self.stay_stable_count = 0
        self.inference_deadline_misses = 0
        self.cycle_count = 0

        self.arm_qpos = np.zeros(4, dtype=np.float64)
        self.arm_qvel = np.zeros(4, dtype=np.float64)
        self.gripper_position = float(self._parameter('gripper_open'))
        self.gripper_velocity = 0.0
        self.joint_state_time = 0.0
        self.object_target = None
        self.object_yaw = 0.0
        self.object_pose_time = 0.0
        self.target_valid = False
        self.target_valid_time = 0.0
        self.delivery_target = self._vector(
            'fallback_delivery_position', 3)
        self.delivery_yaw = float(self._parameter('fallback_delivery_yaw'))
        self.delivery_pose_time = 0.0
        self.base_arrived = False
        self.base_arrived_time = 0.0
        self.base_linear_speed = math.inf
        self.base_angular_speed = math.inf
        self.odom_time = 0.0
        self.safety_stop = False
        self.last_rejected_pose_frame = ''

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            str(self._parameter('arm_trajectory_topic')),
            10,
        )
        self.base_hold_pub = self.create_publisher(
            Bool, str(self._parameter('base_hold_topic')), 10)
        self.status_pub = self.create_publisher(
            String, str(self._parameter('status_topic')), 10)
        self.compat_status_pub = self.create_publisher(
            String, str(self._parameter('compat_status_topic')), 10)
        self.gripper = GripperManager(
            self, str(self._parameter('gripper_action_name')))
        self._create_subscriptions()

        self.last_status_text = ''
        self.last_status_time = 0.0
        self.timer = self.create_timer(
            self.control_period_s, self._on_control_timer)
        self.get_logger().info(
            f'RL control initialized; policy_ready={self.policy is not None} '
            f'period={self.control_period_s:.3f}s')

    def _declare_parameters(self) -> None:
        defaults = {
            'artifact_dir': '',
            'policy_device': 'cpu',
            'policy_torch_threads': 1,
            'policy_warmup_runs': 3,
            'control_rate_hz': 50.0,
            'trajectory_time_s': 0.08,
            'inference_deadline_s': 0.018,
            'inference_deadline_miss_limit': 3,
            'status_period_s': 0.5,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
            'gripper_joint_name': 'gripper_left_joint',
            'target_frame_id': 'base_link',
            'joint_low': [-2.82743, -1.79071, -0.942478, -1.79071],
            'joint_high': [2.82743, 1.57080, 1.38230, 2.04204],
            'joint_velocity_scale': [2.0, 2.0, 2.0, 2.0],
            'workspace_min': [0.10, -0.18, 0.08],
            'workspace_max': [0.42, 0.18, 0.52],
            'arm_base_xy': [-0.08, 0.0],
            'policy_frame_offset': [0.0, 0.0, 0.016],
            'action_scale': [0.014, 0.014, 0.014, 0.014],
            'action_filter_coefficient': 0.18,
            'residual_action_scale': 0.10,
            'max_joint_velocity': [0.70, 0.70, 0.70, 0.70],
            'max_joint_acceleration': [8.0, 8.0, 8.0, 8.0],
            'stay_joint_positions': [0.0, 0.0, 1.38, -1.38],
            'initialize_to_policy_stay': True,
            'initialization_timeout_s': 8.0,
            'ik_initial_guess': [0.0, 1.15968, -0.48813, -0.67155],
            'approach_waypoint_1': [0.0, -0.5, 0.5, 0.0],
            'approach_waypoint_2': [0.0, 0.5, 0.2, -0.7],
            'approach_waypoint_tolerance': 0.08,
            'pregrasp_height_offset': 0.025,
            'reference_action_limit': 1.0,
            'final_approach_action_limit': 1.0,
            'target_update_min_m': 0.001,
            'gripper_open': 0.019,
            'gripper_close': -0.010,
            'gripper_grasp_position': -0.010,
            'gripper_velocity_scale': 0.25,
            'gripper_max_effort': 0.0,
            'gripper_timeout_s': 4.0,
            'release_settle_time_s': 0.24,
            'require_gripper_server': True,
            'require_external_grasp_confirmation': False,
            'grasp_confirmation_timeout_s': 2.0,
            'joint_state_timeout_s': 0.25,
            'target_pose_timeout_s': 0.50,
            'base_arrival_timeout_s': 2.0,
            'odom_timeout_s': 0.50,
            'require_base_arrived': True,
            'require_odom_stopped': True,
            'stopped_linear_speed_mps': 0.01,
            'stopped_angular_speed_radps': 0.05,
            'close_distance_m': 0.042,
            'close_xy_distance_m': 0.035,
            'close_z_tolerance_m': 0.030,
            'close_bearing_tolerance_rad': 0.35,
            'close_roll_tolerance_rad': 0.35,
            'gate_stable_count': 4,
            'stay_tolerance_rad': 0.050,
            'stay_stable_count': 8,
            'delivery_pose_required': False,
            'fallback_delivery_position': [0.27, 0.0, 0.1815],
            'fallback_delivery_yaw': 0.0,
            'publish_every_n_cycles': 1,
            'image_target_topic': '/target/object_pose',
            'target_valid_topic': '/target/valid',
            'delivery_pose_topic': '/target/delivery_pose',
            'joint_states_topic': '/joint_states',
            'odom_topic': '/odom',
            'base_arrived_topic': '/turtlebot3_control/base_arrived',
            'safety_stop_topic': '/safety_stop',
            'grasp_confirmed_topic': '/rl_control/grasp_confirmed',
            'command_topic': '/rl_control/command',
            'compat_start_topic': '/mp_control/start',
            'arm_trajectory_topic': '/arm_controller/joint_trajectory',
            'gripper_action_name': '/gripper_controller/gripper_cmd',
            'base_hold_topic': '/target/base_hold',
            'status_topic': '/rl_control/status',
            'compat_status_topic': '/mp_control/status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_subscriptions(self) -> None:
        subscriptions = (
            (PoseStamped, 'image_target_topic', self._on_object_pose),
            (Bool, 'target_valid_topic', self._on_target_valid),
            (PoseStamped, 'delivery_pose_topic', self._on_delivery_pose),
            (JointState, 'joint_states_topic', self._on_joint_states),
            (Odometry, 'odom_topic', self._on_odom),
            (Bool, 'base_arrived_topic', self._on_base_arrived),
            (Bool, 'safety_stop_topic', self._on_safety_stop),
            (Bool, 'grasp_confirmed_topic', self._on_grasp_confirmed),
            (String, 'command_topic', self._on_command),
            (Bool, 'compat_start_topic', self._on_compat_start),
        )
        for message_type, topic_parameter, callback in subscriptions:
            self.create_subscription(
                message_type,
                str(self._parameter(topic_parameter)),
                callback,
                10,
            )

    def _load_policy(self) -> None:
        try:
            artifact_value = str(self._parameter('artifact_dir')).strip()
            if artifact_value:
                artifact_dir = Path(artifact_value)
            else:
                artifact_dir = (
                    Path(get_package_share_directory('omx_rl_control'))
                    / 'models' / 'policies' / 'arm_delivery_residual_v2'
                )
            self.contract = load_policy_contract(artifact_dir)
            if tuple(self.joint_names) != self.contract.joint_names:
                raise ContractError(
                    'runtime joint_names mismatch: '
                    f'policy={self.contract.joint_names}, '
                    f'runtime={self.joint_names}'
                )
            if self.target_frame_id != self.contract.ros_target_frame:
                raise ContractError(
                    'target_frame_id mismatch: '
                    f'policy={self.contract.ros_target_frame}, '
                    f'runtime={self.target_frame_id}'
                )
            if abs(self.contract.control_period_s - self.control_period_s) > 1e-9:
                raise ContractError(
                    'control period mismatch: '
                    f'policy={self.contract.control_period_s}, '
                    f'runtime={self.control_period_s}')
            runtime_frame_offset = self._vector(
                'policy_frame_offset', 3
            )
            if not np.allclose(
                runtime_frame_offset,
                self.contract.ros_to_training_offset_xyz,
                rtol=0.0,
                atol=1.0e-12,
            ):
                raise ContractError(
                    'policy_frame_offset mismatch: '
                    f'policy={self.contract.ros_to_training_offset_xyz}, '
                    f'runtime={tuple(runtime_frame_offset)}'
                )
            import torch
            torch_threads = int(self._parameter('policy_torch_threads'))
            if torch_threads <= 0:
                raise ValueError('policy_torch_threads must be positive')
            torch.set_num_threads(torch_threads)
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                pass

            from stable_baselines3 import PPO
            self.policy = PPO.load(
                str(self.contract.policy_path),
                device=str(self._parameter('policy_device')),
            )
            validate_policy_spaces(self.policy, self.contract)
            warmup_runs = int(self._parameter('policy_warmup_runs'))
            if warmup_runs < 0:
                raise ValueError('policy_warmup_runs must be non-negative')
            warmup_observation = np.zeros(
                self.contract.observation_size,
                dtype=np.float32,
            )
            for _ in range(warmup_runs):
                self.policy.predict(
                    warmup_observation,
                    deterministic=True,
                )
        except Exception as error:
            self.contract = None
            self.policy = None
            self.startup_error = f'{type(error).__name__}: {error}'
            self.get_logger().error(
                f'Policy startup validation failed: {self.startup_error}')

    def _on_joint_states(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        try:
            self.arm_qpos = np.asarray(
                [positions[name] for name in self.joint_names],
                dtype=np.float64,
            )
        except KeyError:
            return
        self.arm_qvel = np.asarray(
            [velocities.get(name, 0.0) for name in self.joint_names],
            dtype=np.float64,
        )
        if self.gripper_joint_name in positions:
            self.gripper_position = float(
                positions[self.gripper_joint_name])
            self.gripper_velocity = float(
                velocities.get(self.gripper_joint_name, 0.0))
        if np.isfinite(self.arm_qpos).all() and np.isfinite(self.arm_qvel).all():
            self.joint_state_time = time.monotonic()

    def _on_object_pose(self, message: PoseStamped) -> None:
        if not self._pose_frame_valid(message):
            return
        position = np.asarray([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        if not np.isfinite(position).all():
            return
        yaw = self._yaw_from_pose(message)
        if not math.isfinite(yaw):
            return
        self.object_target = position
        self.object_yaw = yaw
        self.object_pose_time = time.monotonic()

    def _on_delivery_pose(self, message: PoseStamped) -> None:
        if not self._pose_frame_valid(message):
            return
        position = np.asarray([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        yaw = self._yaw_from_pose(message)
        if not np.isfinite(position).all() or not math.isfinite(yaw):
            return
        self.delivery_target = position
        self.delivery_yaw = yaw
        self.delivery_pose_time = time.monotonic()

    def _on_target_valid(self, message: Bool) -> None:
        self.target_valid = bool(message.data)
        if self.target_valid:
            self.target_valid_time = time.monotonic()

    def _on_base_arrived(self, message: Bool) -> None:
        self.base_arrived = bool(message.data)
        self.base_arrived_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        twist = message.twist.twist
        self.base_linear_speed = math.hypot(
            twist.linear.x, twist.linear.y)
        self.base_angular_speed = abs(float(twist.angular.z))
        self.odom_time = time.monotonic()

    def _on_safety_stop(self, message: Bool) -> None:
        self.safety_stop = bool(message.data)
        if self.safety_stop:
            self._enter_estop('safety_stop=true')

    def _on_grasp_confirmed(self, message: Bool) -> None:
        self.external_grasp_confirmed = bool(message.data)

    def _on_command(self, message: String) -> None:
        command = message.data.strip().upper()
        if command not in {'PICK', 'PLACE', 'HOLD', 'RESET', 'E_STOP'}:
            self.get_logger().warning(f'Ignoring unknown command: {command}')
            return
        if command == 'E_STOP':
            self.safety_stop = True
            self._enter_estop('E_STOP command')
        elif command == 'HOLD':
            self._enter_hold('HOLD command')
        elif command == 'RESET':
            self._reset_runtime()
        else:
            self.pending_command = command

    def _on_compat_start(self, message: Bool) -> None:
        if message.data:
            self.pending_command = 'PICK'

    def _on_control_timer(self) -> None:
        self.cycle_count += 1
        now = time.monotonic()
        self._publish_base_hold()
        try:
            if self.safety_stop:
                self._enter_estop('safety_stop=true')
            elif (
                self.state in BASE_INTERLOCK_STATES
                and not self._base_stopped(now)
            ):
                self._enter_hold(
                    'base stop gate lost during arm operation'
                )
            elif self.state == RuntimeState.NOT_READY:
                self._handle_not_ready(now)
            elif self.state == RuntimeState.ALIGN_STAY:
                self._handle_align_stay(now)
            elif self.state == RuntimeState.STAY_EMPTY:
                self._handle_stay_empty(now)
            elif self.state == RuntimeState.WAIT_PICK:
                self._handle_wait_pick(now)
            elif self.state == RuntimeState.OPEN_GRIPPER:
                self._handle_open_gripper(now)
            elif self.state == RuntimeState.PICK_REACH:
                self._handle_policy_motion(now)
            elif self.state == RuntimeState.CLOSE_GRIPPER:
                self._handle_close_gripper(now)
            elif self.state == RuntimeState.VERIFY_GRASP:
                self._handle_verify_grasp(now)
            elif self.state == RuntimeState.PICK_TO_STAY:
                self._handle_policy_motion(now)
            elif self.state == RuntimeState.WAIT_DELIVERY:
                self._handle_wait_delivery(now)
            elif self.state == RuntimeState.PLACE_REACH:
                self._handle_policy_motion(now)
            elif self.state == RuntimeState.OPEN_RELEASE:
                self._handle_open_release(now)
            elif self.state == RuntimeState.PLACE_TO_STAY:
                self._handle_policy_motion(now)
        except (ValueError, RuntimeError, KinematicsError) as error:
            self._enter_fault(f'{type(error).__name__}: {error}')
        except Exception as error:
            self._enter_fault(f'unexpected {type(error).__name__}: {error}')
        self._publish_status(now)

    def _handle_not_ready(self, now: float) -> None:
        reasons = self._readiness_reasons(now)
        if reasons:
            self.state_detail = ', '.join(reasons)
            return
        self.action_limiter.reset(self.arm_qpos)
        self.previous_action.fill(0.0)
        self.stay_stable_count = 0
        if bool(self._parameter('initialize_to_policy_stay')):
            self._set_state(
                RuntimeState.ALIGN_STAY,
                'aligning arm to policy Stay',
            )
        else:
            self._set_state(RuntimeState.STAY_EMPTY, 'runtime ready')

    def _handle_align_stay(self, now: float) -> None:
        if not self._joint_state_fresh(now):
            self._enter_fault('joint state timeout during Stay alignment')
            return
        if self._state_timed_out(now, 'initialization_timeout_s'):
            self._enter_fault('policy Stay alignment timeout')
            return

        reference_action = self.reference.action(
            self.action_limiter.arm_target,
            return_to_stay=True,
        )
        arm_target = self.action_limiter.step(
            np.zeros(4, dtype=np.float64),
            reference_action,
        )
        publish_every = int(self._parameter('publish_every_n_cycles'))
        if publish_every <= 0 or self.cycle_count % publish_every == 0:
            self._publish_arm_target(arm_target)

        stay_error = float(np.linalg.norm(
            self.arm_qpos - self.reference.stay_joints
        ))
        if stay_error <= float(self._parameter('stay_tolerance_rad')):
            self.stay_stable_count += 1
        else:
            self.stay_stable_count = 0
        if self.stay_stable_count >= int(
                self._parameter('stay_stable_count')):
            self.action_limiter.reset(self.arm_qpos)
            self.stay_stable_count = 0
            self._set_state(RuntimeState.STAY_EMPTY, 'runtime ready')

    def _handle_stay_empty(self, now: float) -> None:
        if not self._joint_state_fresh(now):
            self._enter_fault('joint state timeout while idle')
            return
        if self.pending_command == 'PICK':
            self.pending_command = ''
            self._set_state(RuntimeState.WAIT_PICK, 'waiting for pick gate')

    def _handle_wait_pick(self, now: float) -> None:
        if not self._base_stopped(now):
            self.state_detail = 'waiting for base arrival and zero velocity'
            return
        if not self._object_target_fresh(now):
            self.state_detail = 'waiting for valid object pose'
            return
        self.reference.set_target(self.object_target, force=True)
        self.action_limiter.reset(self.arm_qpos)
        self.previous_action.fill(0.0)
        self.gripper.command(
            float(self._parameter('gripper_open')),
            float(self._parameter('gripper_max_effort')),
            allow_stall=False,
        )
        self._set_state(RuntimeState.OPEN_GRIPPER, 'opening gripper')

    def _handle_open_gripper(self, now: float) -> None:
        if self.gripper.succeeded:
            self.gripper.reset()
            self._set_state(RuntimeState.PICK_REACH, 'gripper open')
        elif self.gripper.failed:
            self._enter_fault(self.gripper.message)
        elif self._state_timed_out(now, 'gripper_timeout_s'):
            self._enter_fault('gripper open timeout')

    def _handle_close_gripper(self, now: float) -> None:
        if self.gripper.succeeded:
            self.gripper.reset()
            self._set_state(RuntimeState.VERIFY_GRASP,
                            'waiting for grasp confirmation')
        elif self.gripper.failed:
            self._enter_fault(self.gripper.message)
        elif self._state_timed_out(now, 'gripper_timeout_s'):
            self._enter_fault('gripper close timeout')

    def _handle_verify_grasp(self, now: float) -> None:
        require_external = bool(
            self._parameter('require_external_grasp_confirmation'))
        if not require_external or self.external_grasp_confirmed:
            self.grasped = True
            self.stay_stable_count = 0
            self._set_state(RuntimeState.PICK_TO_STAY,
                            'grasp confirmed; returning to Stay')
        elif self._state_timed_out(now, 'grasp_confirmation_timeout_s'):
            self._enter_hold('grasp confirmation timeout')

    def _handle_wait_delivery(self, now: float) -> None:
        if not self._joint_state_fresh(now):
            self._enter_fault('joint state timeout during delivery')
            return
        if self.pending_command != 'PLACE':
            return
        if not self._base_stopped(now):
            self.state_detail = 'PLACE pending; waiting for base stop'
            return
        if (
            bool(self._parameter('delivery_pose_required'))
            and now - self.delivery_pose_time
            > float(self._parameter('target_pose_timeout_s'))
        ):
            self.state_detail = 'PLACE pending; waiting for delivery pose'
            return
        self.pending_command = ''
        self.reference.set_target(self.delivery_target, force=True)
        self.action_limiter.reset(self.arm_qpos)
        self.previous_action.fill(0.0)
        self.stable_gate_count = 0
        self._set_state(RuntimeState.PLACE_REACH, 'starting place reach')

    def _handle_open_release(self, now: float) -> None:
        if self.gripper.succeeded:
            if now - self.state_enter_time < self.release_settle_time_s:
                self.state_detail = 'gripper open; settling object'
                return
            self.gripper.reset()
            self.grasped = False
            self.stay_stable_count = 0
            self._set_state(RuntimeState.PLACE_TO_STAY,
                            'released; returning to Stay')
        elif self.gripper.failed:
            self._enter_fault(self.gripper.message)
        elif self._state_timed_out(now, 'gripper_timeout_s'):
            self._enter_fault('gripper release timeout')

    def _handle_policy_motion(self, now: float) -> None:
        if not self._joint_state_fresh(now):
            self._enter_hold('joint state timeout')
            return
        if self.state in (RuntimeState.PICK_REACH, RuntimeState.PLACE_REACH):
            if self.state == RuntimeState.PICK_REACH:
                if not self._object_target_fresh(now):
                    self._enter_hold('object pose timeout during reach')
                    return
                self.reference.set_target(self.object_target)
                target_yaw = self.object_yaw
            else:
                if (
                    bool(self._parameter('delivery_pose_required'))
                    and now - self.delivery_pose_time
                    > float(self._parameter('target_pose_timeout_s'))
                ):
                    self._enter_hold('delivery pose timeout during reach')
                    return
                self.reference.set_target(self.delivery_target)
                target_yaw = self.delivery_yaw
            self.reference.advance(self.arm_qpos)
            return_to_stay = False
            if not self.reference.final_stage:
                target_yaw = 0.0
        else:
            target_yaw = 0.0
            return_to_stay = True

        phase = policy_phase(self.state)
        eef_position = self.kinematics.forward(self.arm_qpos)
        active_target = self.reference.active_target(return_to_stay)
        observation = self.observation_builder.build(
            self.arm_qpos,
            self.arm_qvel,
            self.gripper_position,
            self.gripper_velocity,
            eef_position,
            active_target,
            target_yaw,
            self.grasped,
            int(phase),
            self.previous_action,
        )
        started = time.perf_counter()
        raw_action, _ = self.policy.predict(
            observation, deterministic=True)
        elapsed = time.perf_counter() - started
        raw_action = np.asarray(raw_action, dtype=np.float64).reshape((-1,))
        if raw_action.shape != (4,) or not np.isfinite(raw_action).all():
            raise RuntimeError(f'invalid policy action: {raw_action}')
        if elapsed > self.inference_deadline_s:
            self.inference_deadline_misses += 1
            if self.inference_deadline_misses == 1:
                self.get_logger().warning(
                    'Dropping late policy output: '
                    f'{elapsed:.4f}s > {self.inference_deadline_s:.4f}s'
                )
            if (
                self.inference_deadline_misses
                >= self.inference_deadline_miss_limit
            ):
                self._enter_hold(
                    'policy inference deadline exceeded '
                    f'{self.inference_deadline_misses} consecutive cycles; '
                    f'latest={elapsed:.4f}s'
                )
            return
        self.inference_deadline_misses = 0

        reference_action = self.reference.action(
            self.action_limiter.arm_target,
            return_to_stay=return_to_stay,
        )
        arm_target = self.action_limiter.step(
            raw_action, reference_action)
        self.previous_action = raw_action.copy()
        publish_every = int(self._parameter('publish_every_n_cycles'))
        if publish_every <= 0 or self.cycle_count % publish_every == 0:
            self._publish_arm_target(arm_target)

        if self.state in (RuntimeState.PICK_REACH, RuntimeState.PLACE_REACH):
            if self._reach_gate(eef_position):
                self.stable_gate_count += 1
            else:
                self.stable_gate_count = 0
            if self.stable_gate_count >= int(
                    self._parameter('gate_stable_count')):
                self.stable_gate_count = 0
                if self.state == RuntimeState.PICK_REACH:
                    self.gripper.command(
                        float(self._parameter('gripper_grasp_position')),
                        float(self._parameter('gripper_max_effort')),
                        allow_stall=True,
                    )
                    self._set_state(RuntimeState.CLOSE_GRIPPER,
                                    'grasp gate reached')
                else:
                    self.gripper.command(
                        float(self._parameter('gripper_open')),
                        float(self._parameter('gripper_max_effort')),
                        allow_stall=False,
                    )
                    self._set_state(RuntimeState.OPEN_RELEASE,
                                    'place gate reached')
        elif return_to_stay:
            stay_error = float(np.linalg.norm(
                self.arm_qpos - self.reference.stay_joints))
            if stay_error <= float(self._parameter('stay_tolerance_rad')):
                self.stay_stable_count += 1
            else:
                self.stay_stable_count = 0
            if self.stay_stable_count >= int(
                    self._parameter('stay_stable_count')):
                self.stay_stable_count = 0
                if self.state == RuntimeState.PICK_TO_STAY:
                    self._set_state(RuntimeState.WAIT_DELIVERY,
                                    'grasp complete')
                else:
                    self._set_state(RuntimeState.COMPLETE,
                                    'pick place complete')

    def _reach_gate(self, eef_position: np.ndarray) -> bool:
        if not self.reference.final_stage:
            return False
        delta = self.reference.pregrasp_target - eef_position
        target_bearing = self.kinematics.bearing(self.reference.target)
        bearing_error = abs(wrap_to_pi(
            float(self.arm_qpos[0]) - target_bearing))
        roll_error = abs(wrap_to_pi(float(np.sum(self.arm_qpos[1:4]))))
        return bool(
            np.linalg.norm(delta)
            <= float(self._parameter('close_distance_m'))
            and np.linalg.norm(delta[:2])
            <= float(self._parameter('close_xy_distance_m'))
            and abs(float(delta[2]))
            <= float(self._parameter('close_z_tolerance_m'))
            and bearing_error
            <= float(self._parameter('close_bearing_tolerance_rad'))
            and roll_error
            <= float(self._parameter('close_roll_tolerance_rad'))
        )

    def _readiness_reasons(self, now: float) -> list[str]:
        reasons = []
        if self.policy is None:
            reasons.append(self.startup_error or 'policy unavailable')
        if not self._joint_state_fresh(now):
            reasons.append('joint states unavailable')
        if (
            bool(self._parameter('require_gripper_server'))
            and not self.gripper.server_ready
        ):
            reasons.append('gripper action server unavailable')
        return reasons

    def _joint_state_fresh(self, now: float) -> bool:
        return bool(
            self.joint_state_time > 0.0
            and now - self.joint_state_time
            <= float(self._parameter('joint_state_timeout_s'))
        )

    def _object_target_fresh(self, now: float) -> bool:
        timeout = float(self._parameter('target_pose_timeout_s'))
        return bool(
            self.object_target is not None
            and self.target_valid
            and now - self.object_pose_time <= timeout
            and now - self.target_valid_time <= timeout
        )

    def _base_stopped(self, now: float) -> bool:
        if bool(self._parameter('require_base_arrived')):
            if not (
                self.base_arrived
                and now - self.base_arrived_time
                <= float(self._parameter('base_arrival_timeout_s'))
            ):
                return False
        if bool(self._parameter('require_odom_stopped')):
            if (
                self.odom_time <= 0.0
                or now - self.odom_time
                > float(self._parameter('odom_timeout_s'))
            ):
                return False
            if (
                self.base_linear_speed
                > float(self._parameter('stopped_linear_speed_mps'))
                or self.base_angular_speed
                > float(self._parameter('stopped_angular_speed_radps'))
            ):
                return False
        return True

    def _state_timed_out(self, now: float, parameter: str) -> bool:
        return now - self.state_enter_time > float(self._parameter(parameter))

    def _reset_runtime(self) -> None:
        if self.safety_stop:
            self.state_detail = 'RESET rejected while safety_stop=true'
            return
        self.pending_command = ''
        self.external_grasp_confirmed = False
        self.stable_gate_count = 0
        self.stay_stable_count = 0
        self.inference_deadline_misses = 0
        self.previous_action.fill(0.0)
        self.gripper.reset()
        if self._joint_state_fresh(time.monotonic()):
            self.action_limiter.reset(self.arm_qpos)
        target_state = (
            RuntimeState.WAIT_DELIVERY
            if self.grasped else RuntimeState.NOT_READY
        )
        self._set_state(target_state, 'manual reset')

    def _enter_hold(self, detail: str) -> None:
        if self.state in (RuntimeState.E_STOP, RuntimeState.FAULT):
            return
        if self._joint_state_fresh(time.monotonic()):
            self.action_limiter.reset(self.arm_qpos)
            self._publish_arm_target(self.arm_qpos)
        self._set_state(RuntimeState.HOLD, detail)

    def _enter_fault(self, detail: str) -> None:
        if self.state == RuntimeState.E_STOP:
            return
        self.gripper.cancel()
        if self._joint_state_fresh(time.monotonic()):
            self.action_limiter.reset(self.arm_qpos)
            self._publish_arm_target(self.arm_qpos)
        self._set_state(RuntimeState.FAULT, detail)
        self.get_logger().error(detail)

    def _enter_estop(self, detail: str) -> None:
        if self.state == RuntimeState.E_STOP:
            return
        self.gripper.cancel()
        if self._joint_state_fresh(time.monotonic()):
            self.action_limiter.reset(self.arm_qpos)
            self._publish_arm_target(self.arm_qpos)
        self._set_state(RuntimeState.E_STOP, detail)

    def _set_state(self, state: RuntimeState, detail: str) -> None:
        if self.state == state and self.state_detail == detail:
            return
        previous = self.state
        self.state = state
        self.state_detail = detail
        self.state_enter_time = time.monotonic()
        self.get_logger().info(
            f'RL state {previous.value} -> {state.value}: {detail}')

    def _publish_arm_target(self, positions: np.ndarray) -> None:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(
            seconds=self.trajectory_time_s).to_msg()
        message.points = [point]
        self.arm_pub.publish(message)

    def _publish_base_hold(self) -> None:
        message = Bool()
        message.data = self.state in BASE_HOLD_STATES
        self.base_hold_pub.publish(message)

    def _publish_status(self, now: float) -> None:
        model_version = (
            self.contract.version if self.contract is not None else 'none')
        text = (
            f'state={self.state.value} detail={self.state_detail} '
            f'model={model_version} grasped={self.grasped} '
            f'pending={self.pending_command or "none"}')
        if (
            text == self.last_status_text
            and now - self.last_status_time < self.status_period_s
        ):
            return
        self.last_status_text = text
        self.last_status_time = now
        message = String()
        message.data = text
        self.status_pub.publish(message)

        compatibility = String()
        if self.state == RuntimeState.WAIT_DELIVERY:
            compatibility.data = 'grasp complete'
        elif self.state == RuntimeState.COMPLETE:
            compatibility.data = 'pick place complete'
        elif self.state in (RuntimeState.FAULT, RuntimeState.E_STOP):
            compatibility.data = f'ERROR {self.state_detail}'
        else:
            compatibility.data = text
        self.compat_status_pub.publish(compatibility)

    def _parameter(self, name: str):
        return self.get_parameter(name).value

    def _pose_frame_valid(self, message: PoseStamped) -> bool:
        frame_id = message.header.frame_id.strip()
        if frame_id == self.target_frame_id:
            return True
        if frame_id != self.last_rejected_pose_frame:
            self.last_rejected_pose_frame = frame_id
            self.get_logger().warning(
                'Rejecting target pose frame '
                f'{frame_id or "<empty>"}; expected {self.target_frame_id}'
            )
        return False

    def _vector(self, name: str, size: int) -> np.ndarray:
        value = np.asarray(self._parameter(name), dtype=np.float64)
        if value.shape != (size,) or not np.isfinite(value).all():
            raise ValueError(f'{name} must be a finite {size}-vector')
        return value

    @staticmethod
    def _yaw_from_pose(message: PoseStamped) -> float:
        orientation = message.pose.orientation
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-12 or not math.isfinite(norm):
            return math.nan
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        sine = 2.0 * (w * z + x * y)
        cosine = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sine, cosine)


def main(args=None):
    """Run the RL control node."""
    rclpy.init(args=args)
    node = RlControlNode()
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
