"""ROS-independent host yaw -> IMU reference and fresh-sample alignment gate.

No depth input or shelf-normal inference. The host must supply the heading of
the shelf approach in the same odom frame as the robot's wheel odometry.
"""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def imu_reference(target_yaw_odom, odom_yaw, imu_yaw):
    """Transfer a host heading into the IMU's possibly offset yaw frame once."""
    if not all(math.isfinite(a) for a in (target_yaw_odom, odom_yaw, imu_yaw)):
        raise ValueError('Host, odometry and IMU yaw must be finite')
    return wrap(imu_yaw+wrap(target_yaw_odom-odom_yaw))


@dataclass(frozen=True)
class HeadingObservation:
    valid: bool
    error: float = 0.
    reason: str = 'WAIT_TARGET_YAW'


class AlignmentGate:
    def __init__(self, tolerance=math.radians(.5), stable_frames=10, gain=1.8,
                 max_omega=.18, max_frame_age=.3, max_stopped_omega=.025):
        self.tolerance, self.stable_frames = tolerance, stable_frames
        self.gain, self.max_omega = gain, max_omega
        self.max_frame_age, self.max_stopped_omega = max_frame_age, max_stopped_omega
        self.reset()

    def reset(self):
        self.count, self.last_frame, self.references = 0, -math.inf, []

    def update(self, estimate, frame_stamp, now, imu_yaw, measured_omega):
        finite = all(math.isfinite(a) for a in (estimate.error, frame_stamp, now, imu_yaw, measured_omega))
        if not estimate.valid or not finite or not 0 <= now-frame_stamp <= self.max_frame_age:
            self.count, self.references = 0, []
            return 0., False, None
        if frame_stamp < self.last_frame:
            return 0., False, None
        command = float(np.clip(self.gain*estimate.error, -self.max_omega, self.max_omega))
        if frame_stamp > self.last_frame:
            good = abs(estimate.error) <= self.tolerance and abs(measured_omega) <= self.max_stopped_omega
            self.count = self.count+1 if good else 0
            self.references = (self.references+[wrap(imu_yaw+estimate.error)])[-self.stable_frames:] if good else []
            self.last_frame = frame_stamp
        ready = self.count >= self.stable_frames
        reference = math.atan2(sum(math.sin(a) for a in self.references),
                               sum(math.cos(a) for a in self.references)) if ready else None
        return command, ready, reference
