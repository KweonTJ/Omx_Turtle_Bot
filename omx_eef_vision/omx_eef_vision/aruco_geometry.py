"""Geometry helpers for ArUco marker pose estimation."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class PoseEstimate:
    """Pose of a square marker in the camera optical frame."""

    rvec: np.ndarray
    tvec: np.ndarray
    reprojection_error_px: float


def square_object_points(marker_size_m: float) -> np.ndarray:
    """Return IPPE_SQUARE corners in OpenCV's required order."""
    half_size = 0.5 * float(marker_size_m)
    return np.array(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float64,
    )


def reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> float:
    """Return mean corner reprojection error in pixels."""
    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        distortion,
    )
    projected = projected.reshape((-1, 2))
    observed = np.asarray(image_points, dtype=np.float64).reshape((-1, 2))
    return float(np.mean(np.linalg.norm(projected - observed, axis=1)))


def estimate_square_pose(
    image_points: np.ndarray,
    marker_size_m: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> PoseEstimate | None:
    """Estimate a square marker pose and select the best positive-Z solution."""
    object_points = square_object_points(marker_size_m)
    image_points = np.asarray(image_points, dtype=np.float64).reshape((4, 2))
    candidates: list[PoseEstimate] = []

    try:
        result = cv2.solvePnPGeneric(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        solved, rvecs, tvecs = result[:3]
        if solved:
            for rvec, tvec in zip(rvecs, tvecs):
                rvec = np.asarray(rvec, dtype=np.float64).reshape((3, 1))
                tvec = np.asarray(tvec, dtype=np.float64).reshape((3, 1))
                if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                    continue
                if float(tvec[2, 0]) <= 0.0:
                    continue
                error = reprojection_error(
                    object_points,
                    image_points,
                    rvec,
                    tvec,
                    camera_matrix,
                    distortion,
                )
                candidates.append(PoseEstimate(rvec, tvec, error))
    except cv2.error:
        candidates = []

    if not candidates:
        solved, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved or float(tvec[2, 0]) <= 0.0:
            return None
        error = reprojection_error(
            object_points,
            image_points,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        candidates.append(PoseEstimate(rvec, tvec, error))

    return min(candidates, key=lambda candidate: candidate.reprojection_error_px)


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to normalized [x, y, z, w]."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    quaternion = np.empty((4,), dtype=np.float64)
    trace = float(np.trace(rotation))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion[3] = 0.25 * scale
        quaternion[0] = (rotation[2, 1] - rotation[1, 2]) / scale
        quaternion[1] = (rotation[0, 2] - rotation[2, 0]) / scale
        quaternion[2] = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        quaternion[3] = (rotation[2, 1] - rotation[1, 2]) / scale
        quaternion[0] = 0.25 * scale
        quaternion[1] = (rotation[0, 1] + rotation[1, 0]) / scale
        quaternion[2] = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        ) * 2.0
        quaternion[3] = (rotation[0, 2] - rotation[2, 0]) / scale
        quaternion[0] = (rotation[0, 1] + rotation[1, 0]) / scale
        quaternion[1] = 0.25 * scale
        quaternion[2] = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
        ) * 2.0
        quaternion[3] = (rotation[1, 0] - rotation[0, 1]) / scale
        quaternion[0] = (rotation[0, 2] + rotation[2, 0]) / scale
        quaternion[1] = (rotation[1, 2] + rotation[2, 1]) / scale
        quaternion[2] = 0.25 * scale

    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('Rotation matrix produced an invalid quaternion')
    return quaternion / norm


def rpy_to_rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll)."""
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64).reshape((3,))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def apply_marker_offset(
    rvec: np.ndarray,
    tvec: np.ndarray,
    offset_xyz: np.ndarray,
    offset_rpy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Transform a marker pose into the configured object pose."""
    marker_rotation, _ = cv2.Rodrigues(
        np.asarray(rvec, dtype=np.float64).reshape((3, 1))
    )
    marker_position = np.asarray(tvec, dtype=np.float64).reshape((3,))
    offset_xyz = np.asarray(offset_xyz, dtype=np.float64).reshape((3,))
    object_rotation = marker_rotation @ rpy_to_rotation_matrix(offset_rpy)
    object_position = marker_position + marker_rotation @ offset_xyz
    return object_position, rotation_matrix_to_quaternion(object_rotation)


def quaternion_angle(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between two quaternions."""
    first = np.asarray(first, dtype=np.float64).reshape((4,))
    second = np.asarray(second, dtype=np.float64).reshape((4,))
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    dot = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def blend_quaternions(
    previous: np.ndarray,
    current: np.ndarray,
    alpha: float,
) -> np.ndarray:
    """Blend normalized quaternions on the same hemisphere."""
    previous = np.asarray(previous, dtype=np.float64).reshape((4,))
    current = np.asarray(current, dtype=np.float64).reshape((4,))
    previous /= np.linalg.norm(previous)
    current /= np.linalg.norm(current)
    if float(np.dot(previous, current)) < 0.0:
        current = -current
    blended = (1.0 - alpha) * previous + alpha * current
    return blended / np.linalg.norm(blended)
