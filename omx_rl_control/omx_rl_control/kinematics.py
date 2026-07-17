"""Lightweight OpenMANIPULATOR-X forward and inverse kinematics."""

from dataclasses import dataclass
import math

import numpy as np


class KinematicsError(RuntimeError):
    """Raised when a reference target is not safely reachable."""


@dataclass(frozen=True)
class IKResult:
    """Result of a bounded numerical inverse-kinematics solve."""

    joints: np.ndarray
    position_error_m: float
    iterations: int


def _translation(vector: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = vector
    return transform


def _rotation_y(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.array([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ])
    return transform


def _rotation_z(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return transform


class OpenManipulatorKinematics:
    """Kinematics matching the checked-in Waffle Pi manipulator URDF."""

    JOINT1_ORIGIN = np.array([-0.080, 0.0, 0.195], dtype=np.float64)
    JOINT2_ORIGIN = np.array([0.0, 0.0, 0.0595], dtype=np.float64)
    JOINT3_ORIGIN = np.array([0.024, 0.0, 0.128], dtype=np.float64)
    JOINT4_ORIGIN = np.array([0.124, 0.0, 0.0], dtype=np.float64)
    EEF_ORIGIN = np.array([0.126, 0.0, 0.0], dtype=np.float64)

    def __init__(
        self,
        joint_low: np.ndarray,
        joint_high: np.ndarray,
        initial_guess: np.ndarray | None = None,
    ):
        self.joint_low = self._vector(joint_low, 'joint_low')
        self.joint_high = self._vector(joint_high, 'joint_high')
        if np.any(self.joint_high <= self.joint_low):
            raise ValueError('joint_high must be greater than joint_low')
        if initial_guess is None:
            initial_guess = np.array(
                [0.0, 1.15968, -0.48813, -0.67155],
                dtype=np.float64,
            )
        self.initial_guess = np.clip(
            self._vector(initial_guess, 'initial_guess'),
            self.joint_low,
            self.joint_high,
        )

    def forward(self, joints: np.ndarray) -> np.ndarray:
        """Return the end-effector position in ROS base_link."""
        q = self._vector(joints, 'joints')
        transform = _translation(self.JOINT1_ORIGIN) @ _rotation_z(q[0])
        transform = (
            transform @ _translation(self.JOINT2_ORIGIN)
            @ _rotation_y(q[1])
        )
        transform = (
            transform @ _translation(self.JOINT3_ORIGIN)
            @ _rotation_y(q[2])
        )
        transform = (
            transform @ _translation(self.JOINT4_ORIGIN)
            @ _rotation_y(q[3])
        )
        transform = transform @ _translation(self.EEF_ORIGIN)
        return transform[:3, 3].copy()

    def bearing(self, target: np.ndarray) -> float:
        """Return target bearing about the arm base."""
        position = np.asarray(target, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('target must be a finite 3-vector')
        delta = position[:2] - self.JOINT1_ORIGIN[:2]
        return float(math.atan2(delta[1], delta[0]))

    def solve_position(
        self,
        target: np.ndarray,
        max_iterations: int = 80,
        tolerance_m: float = 0.001,
        max_position_error_m: float = 0.015,
    ) -> IKResult:
        """Solve the training reference IK without a SciPy dependency."""
        target_position = np.asarray(target, dtype=np.float64)
        if (
            target_position.shape != (3,)
            or not np.isfinite(target_position).all()
        ):
            raise ValueError('target must be a finite 3-vector')

        joints = self.initial_guess.copy()
        joints[0] = np.clip(
            self.bearing(target_position),
            self.joint_low[0],
            self.joint_high[0],
        )
        variables = joints[1:].copy()
        best_variables = variables.copy()
        best_position_error = math.inf
        damping = 1.0e-4

        for iteration in range(1, max_iterations + 1):
            residual = self._ik_residual(
                joints[0], variables, target_position)
            position_error = float(np.linalg.norm(residual[:3]) / 10.0)
            if position_error < best_position_error:
                best_position_error = position_error
                best_variables = variables.copy()
            if position_error <= tolerance_m:
                break

            jacobian = np.empty((4, 3), dtype=np.float64)
            epsilon = 1.0e-5
            for index in range(3):
                shifted = variables.copy()
                shifted[index] += epsilon
                shifted = np.clip(
                    shifted,
                    self.joint_low[1:],
                    self.joint_high[1:],
                )
                denominator = shifted[index] - variables[index]
                if abs(denominator) < 1.0e-12:
                    shifted[index] = max(
                        self.joint_low[index + 1],
                        variables[index] - epsilon,
                    )
                    denominator = shifted[index] - variables[index]
                jacobian[:, index] = (
                    self._ik_residual(
                        joints[0], shifted, target_position) - residual
                ) / denominator

            normal = jacobian.T @ jacobian + damping * np.eye(3)
            gradient = jacobian.T @ residual
            try:
                step = -np.linalg.solve(normal, gradient)
            except np.linalg.LinAlgError:
                step = -np.linalg.lstsq(normal, gradient, rcond=None)[0]

            accepted = False
            baseline = float(residual @ residual)
            for scale in (1.0, 0.5, 0.25, 0.1, 0.05):
                candidate = np.clip(
                    variables + scale * step,
                    self.joint_low[1:],
                    self.joint_high[1:],
                )
                candidate_residual = self._ik_residual(
                    joints[0], candidate, target_position)
                if float(candidate_residual @ candidate_residual) < baseline:
                    variables = candidate
                    accepted = True
                    damping = max(1.0e-8, damping * 0.5)
                    break
            if not accepted:
                damping = min(1.0e2, damping * 10.0)

        solved = np.concatenate(([joints[0]], best_variables))
        if best_position_error > max_position_error_m:
            raise KinematicsError(
                'Reference IK target is outside the safe workspace: '
                f'error={best_position_error:.4f}m target={target_position}')
        return IKResult(solved, best_position_error, iteration)

    def _ik_residual(
        self,
        joint1: float,
        joints_2_to_4: np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        joints = np.concatenate(([joint1], joints_2_to_4))
        position_error = 10.0 * (self.forward(joints) - target)
        return np.concatenate((position_error, [np.sum(joints_2_to_4)]))

    @staticmethod
    def _vector(value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (4,) or not np.isfinite(array).all():
            raise ValueError(f'{name} must be a finite 4-vector')
        return array
