"""Residual-policy action filtering and hardware safety limits."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ActionLimitConfig:
    """Constants used to turn policy actions into joint targets."""

    joint_low: np.ndarray
    joint_high: np.ndarray
    action_scale: np.ndarray
    filter_coefficient: float
    residual_scale: float
    control_period_s: float
    max_velocity: np.ndarray
    max_acceleration: np.ndarray

    def __post_init__(self) -> None:
        for name in (
            'joint_low', 'joint_high', 'action_scale',
            'max_velocity', 'max_acceleration',
        ):
            array = np.asarray(getattr(self, name), dtype=np.float64)
            if array.shape != (4,) or not np.isfinite(array).all():
                raise ValueError(f'{name} must be a finite 4-vector')
            object.__setattr__(self, name, array)
        if np.any(self.joint_high <= self.joint_low):
            raise ValueError('joint_high must exceed joint_low')
        if np.any(self.action_scale <= 0.0):
            raise ValueError('action_scale values must be positive')
        if np.any(self.max_velocity <= 0.0):
            raise ValueError('max_velocity values must be positive')
        if np.any(self.max_acceleration <= 0.0):
            raise ValueError('max_acceleration values must be positive')
        if not 0.0 < self.filter_coefficient <= 1.0:
            raise ValueError('filter_coefficient must be in (0, 1]')
        if not 0.0 <= self.residual_scale <= 1.0:
            raise ValueError('residual_scale must be in [0, 1]')
        if self.control_period_s <= 0.0:
            raise ValueError('control_period_s must be positive')


class ActionLimiter:
    """Apply the training filter and conservative physical limits."""

    def __init__(self, config: ActionLimitConfig):
        self.config = config
        self.arm_target = np.zeros(4, dtype=np.float64)
        self.filtered_action = np.zeros(4, dtype=np.float64)
        self.previous_velocity = np.zeros(4, dtype=np.float64)
        self.initialized = False

    def reset(self, current_joints: np.ndarray) -> None:
        """Start integration from the measured robot state."""
        current = self._vector(current_joints, 'current_joints')
        self.arm_target = np.clip(
            current, self.config.joint_low, self.config.joint_high)
        self.filtered_action.fill(0.0)
        self.previous_velocity.fill(0.0)
        self.initialized = True

    def step(
        self,
        raw_action: np.ndarray,
        reference_action: np.ndarray,
        integrate: bool = True,
    ) -> np.ndarray:
        """Return the next bounded four-joint position target."""
        if not self.initialized:
            raise RuntimeError('ActionLimiter.reset must be called first')
        raw = self._vector(raw_action, 'raw_action')
        reference = self._vector(reference_action, 'reference_action')
        config = self.config
        control = np.clip(
            reference + config.residual_scale * raw,
            -1.0,
            1.0,
        )
        self.filtered_action += config.filter_coefficient * (
            control - self.filtered_action)
        if not integrate:
            return self.arm_target.copy()

        requested_delta = self.filtered_action * config.action_scale
        requested_velocity = requested_delta / config.control_period_s
        velocity = np.clip(
            requested_velocity,
            -config.max_velocity,
            config.max_velocity,
        )
        maximum_velocity_change = (
            config.max_acceleration * config.control_period_s)
        velocity = np.clip(
            velocity,
            self.previous_velocity - maximum_velocity_change,
            self.previous_velocity + maximum_velocity_change,
        )
        next_target = self.arm_target + velocity * config.control_period_s
        self.arm_target = np.clip(
            next_target, config.joint_low, config.joint_high)
        self.previous_velocity = velocity
        return self.arm_target.copy()

    def hold(self) -> np.ndarray:
        """Stop integration while retaining the last safe target."""
        if not self.initialized:
            raise RuntimeError('ActionLimiter.reset must be called first')
        self.previous_velocity.fill(0.0)
        return self.arm_target.copy()

    @staticmethod
    def _vector(value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (4,) or not np.isfinite(array).all():
            raise ValueError(f'{name} must be a finite 4-vector')
        return array
