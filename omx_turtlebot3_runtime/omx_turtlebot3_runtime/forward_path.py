"""Short, coordinated Cartesian commissioning path; not a pick mission."""
import math
import struct
import numpy as np
from scipy.optimize import least_squares
from .core import Fault, Phase, finite

TICK = math.tau/4096


class ForwardPath:
    def __init__(self, planner, start, finger, distance=.010):
        if not 0 < distance <= .010:
            raise Fault('Forward test is limited to 10 mm')
        self.start = finite(start, (4,)).copy()
        self.finger, self.distance = float(finger), distance
        self.eef = planner.forward(self.start, finger).copy()
        self.pitch = float(sum(self.start[1:]))
        # Robot-forward X in the stationary base/policy frame, not camera Z.
        self.points = [self.start.copy()]
        for x in np.linspace(0, distance, 21)[1:]:
            position = self.eef+np.array([x, 0., 0.])
            yaw = math.atan2(position[1]-planner.origin[1], position[0]-planner.origin[0])
            if not planner.low[0] <= yaw <= planner.high[0]:
                raise Fault('Forward yaw outside joint limits')
            seed = self.points[-1][1:]
            result = least_squares(
                lambda a: np.r_[20*(planner.fk(np.r_[yaw,a])-position), sum(a)-self.pitch],
                seed, bounds=(planner.low[1:], planner.high[1:]), max_nfev=80)
            q = np.r_[yaw, result.x]
            if np.linalg.norm(planner.fk(q)-position) > .0005 or abs(sum(q[1:])-self.pitch) > .005:
                raise Fault('Coordinated forward IK failed')
            planner.path(self.points[-1], q, Phase.RECOGNIZE)
            for fraction in np.linspace(0, 1, 7):
                planner.check(self.points[-1]+fraction*(q-self.points[-1]), Phase.RECOGNIZE, finger)
            self.points.append(q)
        self.points = np.asarray(self.points)
        if np.max(np.abs(self.points-self.start)) > .12:
            raise Fault('Forward test joint travel exceeds 0.12 rad')

    def sample(self, progress):
        if not math.isfinite(progress) or not 0 <= progress <= 1:
            raise Fault('Invalid forward path progress')
        x = progress*(len(self.points)-1)
        i = min(int(x), len(self.points)-2)
        return self.points[i]+(x-i)*(self.points[i+1]-self.points[i])


class ForwardEnvelope:
    """Restrict writes to the checked four-joint path; never finger/base/torque."""
    def __init__(self, path, initial_ticks, finger_tick, now):
        self.path = path
        self.start = finite(initial_ticks, (4,)).astype(int)
        if np.max(np.abs((self.start-2048)*TICK-path.start)) > TICK/2:
            raise Fault('Path start differs from actual encoders')
        self.finger_tick = int(finger_tick)
        self.started = self.last_time = now
        self.last_ticks = self.start.copy()
        self.path_ticks = path.points/TICK+2048
        self.low = np.floor(self.path_ticks.min(axis=0)).astype(int)-1
        self.high = np.ceil(self.path_ticks.max(axis=0)).astype(int)+1
        if not math.isfinite(now) or np.any(self.low < 0) or np.any(self.high > 4096):
            raise Fault('Invalid forward encoder envelope')

    def validate(self, address, payload, now, holding=False):
        if not math.isfinite(now) or not holding and not 0 <= now-self.started <= 15.:
            raise Fault('Forward write lease expired')
        if address == 19 and len(payload) == 1:
            return
        if address == 220 and payload == b'\x01':
            return
        if address != 200 or len(payload) != 16:
            raise Fault('Forward test forbids gripper, wheels, torque and profile writes')
        ticks = np.array(struct.unpack('<4i', payload))
        if np.any(ticks < self.low) or np.any(ticks > self.high):
            raise Fault('Forward command outside total travel envelope')
        if not holding:
            a, d = self.path_ticks[:-1], np.diff(self.path_ticks, axis=0)
            t = np.clip(np.sum((ticks-a)*d, axis=1)/np.maximum(np.sum(d*d, axis=1), 1e-12), 0, 1)
            error = np.max(np.abs(ticks-(a+t[:,None]*d)), axis=1)
            if np.min(error) > 1.:
                raise Fault('Forward command left coordinated path')
            if not 0 <= now-self.last_time <= .15 or np.max(np.abs(ticks-self.last_ticks)) > 3:
                raise Fault('Forward command step/deadline exceeded')
        self.last_ticks, self.last_time = ticks, now
