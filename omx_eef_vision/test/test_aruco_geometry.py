"""Unit tests for ArUco geometry and ROS image conversion."""

import cv2
import numpy as np

from omx_eef_vision.aruco_geometry import (
    apply_marker_offset,
    estimate_square_pose,
    square_object_points,
)
from omx_eef_vision.eef_vision_node import image_to_bgr
from sensor_msgs.msg import Image


def test_estimate_square_pose_from_synthetic_corners():
    marker_size = 0.05
    camera_matrix = np.array(
        [
            [420.0, 0.0, 160.0],
            [0.0, 420.0, 120.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.zeros((5,), dtype=np.float64)
    expected_rvec = np.array([[0.12], [-0.08], [0.18]])
    expected_tvec = np.array([[0.015], [-0.01], [0.34]])
    image_points, _ = cv2.projectPoints(
        square_object_points(marker_size),
        expected_rvec,
        expected_tvec,
        camera_matrix,
        distortion,
    )

    estimate = estimate_square_pose(
        image_points,
        marker_size,
        camera_matrix,
        distortion,
    )

    assert estimate is not None
    np.testing.assert_allclose(estimate.tvec, expected_tvec, atol=1.0e-5)
    assert estimate.reprojection_error_px < 1.0e-4


def test_apply_marker_offset_uses_marker_axes():
    object_position, object_orientation = apply_marker_offset(
        np.zeros((3, 1)),
        np.array([[0.1], [0.2], [0.3]]),
        np.array([0.01, -0.02, 0.03]),
        np.zeros((3,)),
    )

    np.testing.assert_allclose(object_position, [0.11, 0.18, 0.33])
    np.testing.assert_allclose(object_orientation, [0.0, 0.0, 0.0, 1.0])


def test_rgb8_image_conversion_does_not_require_cv_bridge():
    message = Image()
    message.height = 1
    message.width = 2
    message.encoding = 'rgb8'
    message.step = 6
    message.data = bytes([255, 0, 0, 0, 255, 0])

    converted = image_to_bgr(message)

    assert converted.shape == (1, 2, 3)
    np.testing.assert_array_equal(converted[0, 0], [0, 0, 255])
    np.testing.assert_array_equal(converted[0, 1], [0, 255, 0])
