"""Pure validation for a one-shot joint3-only commissioning envelope."""
import math
import struct
import numpy as np
from .core import Fault

TICK = math.tau/4096
MAX_TICKS = 22  # 1.934 degrees, below the advertised 2-degree bound.


def command_interval(now, last_command, feedback_stamp):
    """Use the command clock, not the earlier callback/inference-start clock."""
    if not all(math.isfinite(v) for v in (now, last_command, feedback_stamp)):
        raise Fault('Invalid probe command timestamps')
    if not 0 <= now-feedback_stamp <= .10:
        raise Fault('Probe requires fresh feedback')
    dt = now-last_command
    if not 0 < dt <= .15:
        raise Fault(f'Probe command deadline: dt={dt:.6f}s')
    return dt


class ProbeEnvelope:
    def __init__(self, ticks, finger_tick, now):
        values = np.asarray(ticks)
        if (values.shape != (4,) or values.dtype.kind not in 'iu'
                or np.any(values < 0) or np.any(values > 4096-MAX_TICKS)
                or not math.isfinite(now)):
            raise Fault('Invalid probe initial encoders')
        self.start = values.astype(int).copy()
        self.finger_tick = int(finger_tick)
        self.last_tick = int(self.start[2])
        self.started = now

    def validate(self, address, payload, now, holding=False):
        if not math.isfinite(now) or not holding and not 0 <= now-self.started <= 4.:
            raise Fault('Probe hardware lease expired')
        if address == 19 and len(payload) == 1:
            return
        if address == 220 and payload == b'\x01':
            return
        if address != 200 or len(payload) != 16:
            raise Fault('Probe forbids gripper, base, torque and profile writes')
        ticks = np.asarray(struct.unpack('<4i', payload))
        if not np.array_equal(ticks[[0, 1, 3]], self.start[[0, 1, 3]]):
            raise Fault('Probe may move joint3 only')
        if not self.start[2] <= ticks[2] <= self.start[2]+MAX_TICKS:
            raise Fault('Probe total travel limit')
        if not holding and not 0 <= ticks[2]-self.last_tick <= 2:
            raise Fault('Probe must be monotonic with at most two ticks per command')
        self.last_tick = int(ticks[2])
