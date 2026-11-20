import numpy as np
import pytest
import yaml
from pathlib import Path
from omx_turtlebot3_runtime.core import Fault, RobotSample

pytest.importorskip('rclpy')
from omx_turtlebot3_runtime.vision import Detector, synchronized_joints, paired_camera_info
from sensor_msgs.msg import Image, CameraInfo
import cv2


def camera_pair():
    image, info = Image(), CameraInfo()
    for message in (image, info):
        message.width, message.height = 320, 240
        message.header.stamp.sec, message.header.stamp.nanosec = 123, 456
    image.header.frame_id = 'eef_usb_camera_optical_frame'
    return image, info


def test_bind_missing_info_frame_only_from_matching_image_without_mutation():
    image, info = camera_pair()
    bound = paired_camera_info(image, info, image.header.frame_id)
    assert bound.header.frame_id == image.header.frame_id
    assert info.header.frame_id == ''


@pytest.mark.parametrize('mismatch', ['timestamp', 'resolution', 'image_frame', 'info_frame'])
def test_camera_frame_binding_rejects_unrelated_messages(mismatch):
    image, info = camera_pair()
    if mismatch == 'timestamp':
        info.header.stamp.nanosec += 1
    elif mismatch == 'resolution':
        info.width = 640
    elif mismatch == 'image_frame':
        image.header.frame_id = ''
    else:
        info.header.frame_id = 'some_other_camera'
    with pytest.raises(Fault):
        paired_camera_info(image, info, 'eef_usb_camera_optical_frame')


def test_real_detector_isolates_ids_zero_and_one():
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/runtime.yaml').read_text())
    detector = Detector(cfg['vision'])
    image = np.full((240, 320, 3), 255, dtype=np.uint8)
    for i, x in enumerate((30, 190)):
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marker = cv2.aruco.generateImageMarker(detector.dictionary, i, 80)
        else:
            marker = cv2.aruco.drawMarker(detector.dictionary, i, 80)
        image[80:160, x:x+80] = marker[:, :, None]
    found = detector.detect(image, np.array([[300., 0, 160], [0, 300, 120], [0, 0, 1]]), np.zeros(5))
    assert sorted(v[0] for v in found) == [0, 1]


def test_interpolation_not_latest_tf():
    a = RobotSample(10., np.zeros(4), np.ones(4), 0., 0., 0., 12., (0, 0), 10000)
    b = RobotSample(10.1, np.ones(4)*.1, np.ones(4), 0., 0., 0., 12., (0, 0), 10100)
    np.testing.assert_allclose(synchronized_joints([a, b], 10.05, .06), np.ones(4)*.05)
    with pytest.raises(Fault):
        synchronized_joints([a, b], 10.12, .06)
    with pytest.raises(Fault):
        synchronized_joints([a, b], 9.8, .06)
