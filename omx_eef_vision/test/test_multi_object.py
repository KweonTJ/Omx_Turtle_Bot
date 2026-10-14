# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Exercise simultaneous detection and strictly ID-based perception selection."""

import json

import cv2
import numpy as np
from omx_eef_vision.multi_object_core import (
    camera_model, MultiMarkerDetector, SelectionSession,
)
from omx_eef_vision.print_markers import marker_bitmap, print_sheet
import pytest
from sensor_msgs.msg import CameraInfo


def scene(ids=(1, 2, 3, 4)):
    """Produce a single frame with four separated markers."""
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    positions = ((80, 80), (440, 80), (80, 300), (440, 300))
    for marker_id, (x, y) in zip(ids, positions):
        tag = marker_bitmap(dictionary, marker_id, 90)
        image[y:y+90, x:x+90] = tag[..., None]
    return image


def observations(ids=(1, 2, 3, 4), offset=0):
    """Supply synthetic valid poses to isolate selection state tests."""
    return [{'marker_id': i, 'usable_2d': True, 'duplicate': False,
             'pose_valid': True, 'position': [i*.02+offset, 0., .3],
             'orientation_xyzw': [0., 0., 0., 1.]} for i in ids]


def request(session, marker_id=3, request_id='r1', stamp=0):
    """Select an ID without publishing any robot action."""
    return session.request(json.dumps({
        'request_id': request_id, 'command': 'SELECT', 'object_id': marker_id}), stamp)


def test_four_ids_detected_in_one_frame_without_calibration():
    records = MultiMarkerDetector().detect(scene())
    assert [r['marker_id'] for r in records] == [1, 2, 3, 4]
    assert all(r['usable_2d'] and not r['pose_valid'] for r in records)


def test_duplicate_marker_never_becomes_a_unique_target():
    records = MultiMarkerDetector().detect(scene((1, 2, 3, 3)))
    duplicate = [r for r in records if r['marker_id'] == 3]
    assert len(duplicate) == 2
    assert all(r['duplicate'] and not r['usable_2d'] for r in duplicate)


def test_overview_then_selected_only_remains_trackable():
    session = SelectionSession()
    request(session)
    for stamp in (1, 2, 3):
        selected, reason = session.observe(observations(), stamp)
    assert reason == 'TRACKING' and selected['marker_id'] == 3
    assert session.observe(observations((3,)), 4)[1] == 'TRACKING'
    assert session.observe(observations((1, 2, 4)), 5)[1] == 'TARGET_NOT_VISIBLE'
    assert session.observe(observations((3,)), 6)[1] == 'WAIT_STABLE_POSE'


def test_missing_initial_id_does_not_complete_overview():
    session = SelectionSession()
    request(session)
    for stamp in range(1, 10):
        assert session.observe(observations((1, 2, 3)), stamp)[1] == (
            'WAIT_FOUR_OBJECT_OVERVIEW')


def test_new_request_requires_new_frame_and_stable_new_id():
    session = SelectionSession(stable_frames=1)
    request(session, stamp=10)
    assert session.observe(observations(), 9)[1] == 'WAIT_NEW_FRAME'
    assert session.observe(observations(), 11)[1] == 'TRACKING'
    request(session, 1, 'r2', 12)
    selected, reason = session.observe(observations(), 13)
    assert reason == 'TRACKING' and selected['marker_id'] == 1


def test_pose_jump_reacquires_current_position_not_cached_position():
    session = SelectionSession(stable_frames=2)
    request(session)
    session.observe(observations(), 1)
    assert session.observe(observations(), 2)[1] == 'TRACKING'
    assert session.observe(observations(offset=.12), 3)[1] == 'WAIT_STABLE_POSE'
    selected, reason = session.observe(observations(offset=.12), 4)
    assert reason == 'TRACKING'
    assert selected['position'][0] == pytest.approx(.18)


@pytest.mark.parametrize('payload', [
    '[]', '{}', '{', '{"request_id":"r","object_id":true}',
    '{"request_id":"r","object_id":5}',
    '{"request_id":"r","object_id":1,"command":"PICK"}',
    '{"request_id":"r","object_id":1,"unknown":2}',
])
def test_rejected_request_does_not_replace_selection(payload):
    session = SelectionSession()
    request(session)
    with pytest.raises((ValueError, TypeError)):
        session.request(payload, 5)
    assert session.selected_id == 3


def test_duplicate_request_does_not_rearm_or_revert_selection():
    session = SelectionSession()
    request(session)
    request(session, 2, 'r2')
    assert not request(session)
    assert session.selected_id == 2
    with pytest.raises(ValueError):
        request(session, 1, 'r1')


def test_clear_resets_selection_and_overview():
    session = SelectionSession(stable_frames=1)
    request(session)
    session.observe(observations(), 1)
    session.request('{"request_id":"clear1","command":"CLEAR"}', 2)
    assert session.selected_id is None and not session.overview_ready


def calibration():
    """Construct a calibration matching the synthetic image only."""
    info = CameraInfo(width=640, height=480, distortion_model='plumb_bob')
    info.header.frame_id = 'test_camera'
    info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    info.d = [0.] * 5
    return info


def test_calibration_exact_mode_and_frame_required():
    info = calibration()
    matrix, _ = camera_model(info, 640, 480, 'test_camera')
    assert matrix[0, 0] == 600
    with pytest.raises(ValueError, match='SIZE_MISMATCH'):
        camera_model(info, 320, 240, 'test_camera')
    with pytest.raises(ValueError, match='FRAME_MISMATCH'):
        camera_model(info, 640, 480, 'other_camera')
    info.k[0] = 0
    with pytest.raises(ValueError, match='INVALID_CALIBRATION'):
        camera_model(info, 640, 480, 'test_camera')


@pytest.mark.parametrize('change', ['nan', 'fisheye', 'roi', 'binning'])
def test_unsupported_calibration_rejected(change):
    info = calibration()
    if change == 'nan':
        info.d[0] = float('nan')
    elif change == 'fisheye':
        info.distortion_model = 'equidistant'
    elif change == 'roi':
        info.roi.width = 320
    else:
        info.binning_x = 2
    with pytest.raises(ValueError):
        camera_model(info, 640, 480, 'test_camera')


def test_print_sheet_has_physical_page_size_and_four_labels():
    svg = print_sheet()
    assert 'width="210mm" height="297mm"' in svg
    for marker_id in range(1, 5):
        assert f'>ID {marker_id}</text>' in svg
