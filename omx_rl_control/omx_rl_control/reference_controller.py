"""Deterministic reference path used by the residual PPO policy."""

import numpy as np

from .kinematics import OpenManipulatorKinematics


class ReferenceController:
    """Generate approach, pregrasp, and return-to-stay references."""

    def __init__(
        self,
        kinematics: OpenManipulatorKinematics,
        stay_joints: np.ndarray,
        action_scale: np.ndarray,
        approach_waypoints: np.ndarray,
        waypoint_tolerance: float = 0.08,
        pregrasp_height_offset: float = 0.0225,
        action_limit: float = 1.0,
        final_action_limit: float = 1.0,
        target_update_min_m: float = 0.001,
    ):
        self.kinematics = kinematics
        self.stay_joints = self._vector(stay_joints, 'stay_joints')
        self.action_scale = self._vector(action_scale, 'action_scale')
        waypoints = np.asarray(approach_waypoints, dtype=np.float64)
        if waypoints.ndim != 2 or waypoints.shape[1] != 4:
            raise ValueError('approach_waypoints must have shape (N, 4)')
        self.base_waypoints = waypoints
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.pregrasp_height_offset = float(pregrasp_height_offset)
        self.action_limit = float(action_limit)
        self.final_action_limit = float(final_action_limit)
        self.target_update_min_m = float(target_update_min_m)
        if np.any(self.action_scale <= 0.0):
            raise ValueError('action_scale values must be positive')
        if self.waypoint_tolerance <= 0.0:
            raise ValueError('waypoint_tolerance must be positive')

        self.target = None
        self.pregrasp_target = None
        self.pregrasp_joints = None
        self.waypoints = np.empty((0, 4), dtype=np.float64)
        self.waypoint_targets = np.empty((0, 3), dtype=np.float64)
        self.stage = 0

    @property
    def final_stage(self) -> bool:
        """Return true after all collision-avoidance waypoints."""
        return self.stage >= len(self.waypoints)

    def set_target(self, target: np.ndarray, force: bool = False) -> bool:
        """Update the object target and solve its pregrasp reference."""
        position = np.asarray(target, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('target must be a finite 3-vector')
        if (
            not force
            and self.target is not None
            and np.linalg.norm(position - self.target) < self.target_update_min_m
        ):
            return False

        initialize_reference = self.target is None or force
        bearing = self.kinematics.bearing(position)
        if initialize_reference:
            self.waypoints = self.base_waypoints.copy()
            if len(self.waypoints):
                self.waypoints[:, 0] = bearing
            self.waypoint_targets = np.asarray([
                self.kinematics.forward(waypoint)
                for waypoint in self.waypoints
            ], dtype=np.float64).reshape((-1, 3))
            self.stage = 0

        self.target = position.copy()
        self.pregrasp_target = position + np.array(
            [0.0, 0.0, self.pregrasp_height_offset],
            dtype=np.float64,
        )
        if initialize_reference:
            self.pregrasp_joints = self.kinematics.solve_position(
                self.pregrasp_target).joints
        return True

    def advance(self, current_joints: np.ndarray) -> bool:
        """Advance one waypoint when the current arm enters tolerance."""
        current = self._vector(current_joints, 'current_joints')
        if self.final_stage:
            return False
        error = float(np.linalg.norm(current - self.waypoints[self.stage]))
        if error > self.waypoint_tolerance:
            return False
        self.stage += 1
        if self.final_stage and self.pregrasp_target is not None:
            self.pregrasp_joints = self.kinematics.solve_position(
                self.pregrasp_target).joints
        return True

    def active_target(self, return_to_stay: bool = False) -> np.ndarray:
        """Return the EEF target used in the policy observation."""
        if return_to_stay:
            return self.kinematics.forward(self.stay_joints)
        if self.target is None or self.pregrasp_target is None:
            raise RuntimeError('reference target is not configured')
        if not self.final_stage:
            return self.waypoint_targets[self.stage].copy()
        return self.pregrasp_target.copy()

    def action(
        self,
        arm_target: np.ndarray,
        return_to_stay: bool = False,
        gate_active: bool = False,
    ) -> np.ndarray:
        """Return the normalized reference action used during training."""
        current_target = self._vector(arm_target, 'arm_target')
        if gate_active:
            return np.zeros(4, dtype=np.float64)
        if return_to_stay:
            joint_goal = self.stay_joints
            action_limit = self.action_limit
        elif not self.final_stage:
            joint_goal = self.waypoints[self.stage]
            action_limit = self.action_limit
        else:
            if self.pregrasp_joints is None:
                raise RuntimeError('reference target is not configured')
            joint_goal = self.pregrasp_joints
            action_limit = self.final_action_limit
        return np.clip(
            (joint_goal - current_target) / self.action_scale,
            -action_limit,
            action_limit,
        )

    @staticmethod
    def _vector(value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (4,) or not np.isfinite(array).all():
            raise ValueError(f'{name} must be a finite 4-vector')
        return array
