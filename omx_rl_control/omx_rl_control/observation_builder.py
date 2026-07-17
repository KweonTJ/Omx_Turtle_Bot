"""Build the exact 33-dimensional observation used during PPO training."""

from dataclasses import dataclass
import math

import numpy as np


def wrap_to_pi(value: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return float((value + math.pi) % (2.0 * math.pi) - math.pi)


@dataclass(frozen=True)
class ObservationConfig:
    """Normalization constants copied from the training environment."""

    joint_low: np.ndarray
    joint_high: np.ndarray
    joint_velocity_scale: np.ndarray
    workspace_min: np.ndarray
    workspace_max: np.ndarray
    arm_base_xy: np.ndarray
    policy_frame_offset: np.ndarray
    gripper_close: float = -0.010
    gripper_open: float = 0.019
    gripper_velocity_scale: float = 0.25

    def __post_init__(self) -> None:
        vectors = {
            'joint_low': (self.joint_low, 4),
            'joint_high': (self.joint_high, 4),
            'joint_velocity_scale': (self.joint_velocity_scale, 4),
            'workspace_min': (self.workspace_min, 3),
            'workspace_max': (self.workspace_max, 3),
            'arm_base_xy': (self.arm_base_xy, 2),
            'policy_frame_offset': (self.policy_frame_offset, 3),
        }
        for name, (value, size) in vectors.items():
            array = np.asarray(value, dtype=np.float64)
            if array.shape != (size,) or not np.isfinite(array).all():
                raise ValueError(f'{name} must be a finite {size}-vector')
            object.__setattr__(self, name, array)
        if np.any(self.joint_high <= self.joint_low):
            raise ValueError('joint_high must be greater than joint_low')
        if np.any(self.workspace_max <= self.workspace_min):
            raise ValueError('workspace_max must exceed workspace_min')
        if self.gripper_open <= self.gripper_close:
            raise ValueError('gripper_open must exceed gripper_close')
        if self.gripper_velocity_scale <= 0.0:
            raise ValueError('gripper_velocity_scale must be positive')


class ObservationBuilder:
    """Convert ROS robot state into the policy observation schema."""

    SIZE = 33

    def __init__(self, config: ObservationConfig):
        self.config = config

    def build(
        self,
        arm_qpos: np.ndarray,
        arm_qvel: np.ndarray,
        gripper_position: float,
        gripper_velocity: float,
        eef_position: np.ndarray,
        active_target: np.ndarray,
        target_yaw: float,
        grasped: bool,
        phase_index: int,
        previous_action: np.ndarray,
    ) -> np.ndarray:
        """Return one clipped float32 observation in training order."""
        qpos = self._vector(arm_qpos, 4, 'arm_qpos')
        qvel = self._vector(arm_qvel, 4, 'arm_qvel')
        eef = self._vector(eef_position, 3, 'eef_position')
        target = self._vector(active_target, 3, 'active_target')
        previous = self._vector(previous_action, 4, 'previous_action')
        scalars = np.asarray(
            [gripper_position, gripper_velocity, target_yaw],
            dtype=np.float64,
        )
        if not np.isfinite(scalars).all():
            raise ValueError('observation scalar inputs must be finite')
        if phase_index not in range(4):
            raise ValueError('phase_index must be in [0, 3]')

        config = self.config
        qpos_normalized = 2.0 * (
            (qpos - config.joint_low)
            / (config.joint_high - config.joint_low)
        ) - 1.0
        qvel_normalized = qvel / config.joint_velocity_scale
        gripper_normalized = 2.0 * (
            (gripper_position - config.gripper_close)
            / (config.gripper_open - config.gripper_close)
        ) - 1.0
        gripper_velocity_normalized = (
            gripper_velocity / config.gripper_velocity_scale)

        policy_eef = eef + config.policy_frame_offset
        policy_target = target + config.policy_frame_offset
        relative = policy_target - policy_eef
        workspace_span = config.workspace_max - config.workspace_min
        eef_normalized = 2.0 * (
            (policy_eef - config.workspace_min) / workspace_span
        ) - 1.0
        target_normalized = 2.0 * (
            (policy_target - config.workspace_min) / workspace_span
        ) - 1.0
        relative_normalized = 2.0 * relative / workspace_span

        bearing_delta = target[:2] - config.arm_base_xy
        target_bearing = math.atan2(bearing_delta[1], bearing_delta[0])
        roll_error = wrap_to_pi(float(np.sum(qpos[1:4]))) / math.pi
        phase_one_hot = np.zeros(4, dtype=np.float64)
        phase_one_hot[phase_index] = 1.0

        observation = np.concatenate((
            qpos_normalized,
            qvel_normalized,
            [gripper_normalized],
            [gripper_velocity_normalized],
            eef_normalized,
            target_normalized,
            relative_normalized,
            [math.sin(target_bearing), math.cos(target_bearing)],
            [math.sin(target_yaw), math.cos(target_yaw)],
            [roll_error],
            [float(grasped)],
            phase_one_hot,
            previous,
        ))
        if observation.shape != (self.SIZE,):
            raise RuntimeError(
                f'Internal observation size error: {observation.shape}')
        if not np.isfinite(observation).all():
            raise ValueError('observation contains NaN or Inf')
        return np.clip(observation, -1.0, 1.0).astype(np.float32)

    @staticmethod
    def _vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (size,) or not np.isfinite(array).all():
            raise ValueError(f'{name} must be a finite {size}-vector')
        return array
