# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Multi-marker perception and selection, without robot control dependencies."""

from collections import Counter, OrderedDict
import json

import cv2
import numpy as np

from .aruco_geometry import quaternion_angle
from .eef_vision_node import get_aruco_dictionary, make_detector_parameters


class MultiMarkerDetector:
    """Return every detected marker, including ambiguous duplicate IDs."""

    def __init__(self, dictionary='DICT_4X4_50', min_perimeter=48.0):
        self.dictionary = get_aruco_dictionary(dictionary)
        self.parameters = make_detector_parameters()
        self.min_perimeter = min_perimeter
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.parameters)

    def detect(self, image):
        """Detect IDs/corners independently of calibration or TF availability."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)
        if ids is None:
            return []
        counts = Counter(int(value) for value in ids.flatten())
        records = []
        for corner, marker_id in zip(corners, ids.flatten()):
            points = np.asarray(corner).reshape(4, 2)
            perimeter = float(np.linalg.norm(
                points - np.roll(points, 1, axis=0), axis=1).sum())
            records.append({
                'marker_id': int(marker_id),
                'corners_px': points.tolist(),
                'center_px': points.mean(axis=0).tolist(),
                'perimeter_px': perimeter,
                'duplicate': counts[int(marker_id)] > 1,
                'usable_2d': (counts[int(marker_id)] == 1
                              and perimeter >= self.min_perimeter),
                'pose_valid': False,
                'pose_reason': 'WAIT_CALIBRATION',
            })
        return sorted(records, key=lambda item: item['marker_id'])


def camera_model(info, width, height, frame_id):
    """Require an exact raw-image calibration; never resize intrinsics silently."""
    if info is None:
        raise ValueError('WAIT_CALIBRATION')
    if (info.width, info.height) != (width, height):
        raise ValueError('CALIBRATION_SIZE_MISMATCH')
    if not frame_id or info.header.frame_id != frame_id:
        raise ValueError('CALIBRATION_FRAME_MISMATCH')
    if info.binning_x > 1 or info.binning_y > 1:
        raise ValueError('UNSUPPORTED_BINNING')
    if any((info.roi.x_offset, info.roi.y_offset,
            info.roi.width, info.roi.height)):
        raise ValueError('UNSUPPORTED_ROI')
    matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=float)
    lengths = {'plumb_bob': (4, 5), 'rational_polynomial': (8, 12, 14)}
    if len(distortion) not in lengths.get(info.distortion_model, ()):
        raise ValueError('UNSUPPORTED_DISTORTION')
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])):
        raise ValueError('INVALID_CALIBRATION')
    return matrix, distortion


class SelectionSession:
    """Select only the requested ID and require fresh, stable pose observations."""

    def __init__(self, expected_ids=(1, 2, 3, 4), stable_frames=3,
                 max_position_jump=0.08, max_angle_jump=0.8):
        self.expected_ids = tuple(expected_ids)
        if (not self.expected_ids or len(set(expected_ids)) != len(expected_ids)
                or any(type(value) is not int or value < 0
                       for value in expected_ids) or stable_frames < 1):
            raise ValueError('Invalid expected IDs or stable frame count')
        self.stable_frames = stable_frames
        self.max_position_jump = max_position_jump
        self.max_angle_jump = max_angle_jump
        self.selected_id = None
        self.request_id = None
        self.request_stamp_ns = 0
        self.seen_requests = OrderedDict()
        self.reset_observations()

    def reset_observations(self):
        """Forget overview and pose stability after a stream interruption."""
        self.overview_count = 0
        self.overview_ready = False
        self.tracks = {}

    def request(self, text, stamp_ns):
        """Accept SELECT/CLEAR metadata only; no motion commands are supported."""
        if len(text.encode('utf-8')) > 2048:
            raise ValueError('Request exceeds 2048 bytes')
        request = json.loads(text)
        if not isinstance(request, dict):
            raise ValueError('Request must be an object')
        command = request.get('command', 'SELECT')
        fields = {'request_id', 'command', 'object_id'}
        if set(request) - fields or command not in ('SELECT', 'CLEAR'):
            raise ValueError('Only SELECT/CLEAR perception requests are allowed')
        request_id = request.get('request_id')
        if (not isinstance(request_id, str) or not request_id.strip()
                or len(request_id) > 128):
            raise ValueError('request_id must be a nonempty string, max 128 chars')
        marker_id = request.get('object_id')
        if command == 'SELECT':
            if type(marker_id) is not int or marker_id not in self.expected_ids:
                raise ValueError('object_id must be one of the configured IDs')
        elif marker_id is not None:
            raise ValueError('CLEAR must omit object_id')
        identity = (command, marker_id)
        if request_id in self.seen_requests:
            if self.seen_requests[request_id] != identity:
                raise ValueError('request_id reused with different content')
            return False
        # Do not evict IDs and accidentally re-accept an old replay.
        if len(self.seen_requests) >= 4096:
            raise ValueError('Session request capacity reached; restart node')
        self.seen_requests[request_id] = identity
        self.request_id = request_id
        self.selected_id = marker_id
        self.request_stamp_ns = stamp_ns
        self.tracks = {}
        if command == 'CLEAR':
            self.reset_observations()
        return True

    def observe(self, records, stamp_ns):
        """Latch a simultaneous overview, then follow each visible ID separately."""
        usable = {r['marker_id']: r for r in records if r['usable_2d']}
        if set(self.expected_ids).issubset(usable):
            self.overview_count += 1
        else:
            self.overview_count = 0
        if self.overview_count >= self.stable_frames:
            self.overview_ready = True
        next_tracks = {}
        for marker_id, record in usable.items():
            if marker_id not in self.expected_ids or not record['pose_valid']:
                continue
            position = np.asarray(record['position'], dtype=float)
            orientation = np.asarray(record['orientation_xyzw'], dtype=float)
            previous = self.tracks.get(marker_id)
            count = 1
            if previous is not None:
                old_position, old_orientation, old_count = previous
                if (np.linalg.norm(position - old_position)
                        <= self.max_position_jump
                        and quaternion_angle(orientation.copy(),
                                             old_orientation.copy())
                        <= self.max_angle_jump):
                    count = old_count + 1
            next_tracks[marker_id] = (position, orientation, count)
            record['stable_frames'] = count
        self.tracks = next_tracks
        selected = usable.get(self.selected_id)
        reason = 'WAIT_REQUEST'
        if self.selected_id is not None:
            if stamp_ns <= self.request_stamp_ns:
                reason = 'WAIT_NEW_FRAME'
            elif not self.overview_ready:
                reason = 'WAIT_FOUR_OBJECT_OVERVIEW'
            elif selected is None:
                duplicate = any(r['marker_id'] == self.selected_id
                                and r['duplicate'] for r in records)
                reason = 'DUPLICATE_ID' if duplicate else 'TARGET_NOT_VISIBLE'
            elif not selected['pose_valid']:
                reason = selected['pose_reason']
            elif selected.get('stable_frames', 0) < self.stable_frames:
                reason = 'WAIT_STABLE_POSE'
            else:
                reason = 'TRACKING'
        return selected, reason
