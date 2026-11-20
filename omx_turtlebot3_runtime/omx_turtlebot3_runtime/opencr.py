"""Exclusive OpenCR transport. Read-only by default, no wheel/torque/EEPROM writes."""
import fcntl
import math
import struct
import subprocess
import termios
import time
import numpy as np

from .core import Fault, RobotSample, finite, commissioning_issues
from .robot_contract import mp_controller_trajectory


class OpenCR:
    def __init__(self, cfg, execute=False):
        self.cfg, self.execute = cfg, execute
        self.c = cfg['serial']
        self.writes = 0
        self.armed = False
        self.last = self.last_raw = self.command_q = None
        self.command_finger = None
        self.last_trajectory = None
        self.heartbeat = 0
        self.closed = False
        self.restore_values = None
        if execute and commissioning_issues(cfg):
            raise Fault('Execution calibration gates: '+', '.join(commissioning_issues(cfg)))
        if subprocess.run(['fuser', self.c['device']], capture_output=True).returncode == 0:
            raise Fault('Serial device already owned; stop existing hardware driver explicitly')
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        self.ok = COMM_SUCCESS
        self.port = PortHandler(self.c['device'])
        self.packet = PacketHandler(2.0)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(self.c['baudrate']):
                raise Fault('OpenCR serial initialization failed')
            fcntl.flock(self.port.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            model, comm, error = self.packet.ping(self.port, self.c['id'])
            if comm != self.ok or error or model != self.c['model_number']:
                raise Fault('Unexpected OpenCR identity or ping error')
            self.read()
            if self.last_raw['ros']:
                raise Fault('Existing OpenCR heartbeat active')
        except BaseException:
            self.port.closePort()
            raise

    def read(self):
        start = time.monotonic()
        raw, comm, error = self.packet.readTxRx(self.port, self.c['id'], 0, 344)
        now = time.monotonic()
        if comm != self.ok or error or len(raw) != 344 or now-start > .10:
            raise Fault('OpenCR telemetry failed or exceeded deadline')
        b = bytes(raw)
        millis = struct.unpack_from('<I', b, 10)[0]
        ticks = np.array(struct.unpack_from('<4i', b, 224))
        if np.any(ticks < 0) or np.any(ticks > 4096):
            raise Fault('Invalid joint encoder ticks')
        signs = finite(self.c['joint_signs'], (4,))
        if not np.all(np.isin(signs, [-1, 1])):
            raise Fault('Invalid calibrated joint signs')
        q = (ticks-finite(self.c['joint_zero_ticks'], (4,)))*math.tau/4096*signs
        finger_tick = struct.unpack_from('<i', b, 240)[0]
        finger = (finger_tick-self.c['gripper_zero_tick'])*math.tau/4096*self.c['gripper_m_per_rad']
        dq, finger_v = np.zeros(4), 0.
        if self.last is not None:
            if millis <= self.last.millis:
                raise Fault('OpenCR feedback counter stalled or rebooted')
            dt = (millis-self.last.millis)/1000
            # Encoder differences avoid assuming firmware raw-velocity scaling.
            dq = (q-self.last.q)/dt
            finger_v = (finger-self.last.finger)/dt
        voltage = struct.unpack_from('<I', b, 42)[0]/100.
        wheel_ticks = struct.unpack_from('<2i', b, 136)
        wheel_velocity = struct.unpack_from('<2i', b, 128)
        base_command = struct.unpack_from('<6i', b, 150)
        current = struct.unpack_from('<h', b, 272)[0]
        raw_info = dict(ros=b[15], connected=b[16], torque=b[199],
                        ticks=ticks.tolist(), finger_tick=finger_tick,
                        arm_currents_raw=list(struct.unpack_from('<4h', b, 264)),
                        profile_acc=list(struct.unpack_from('<5i', b, 284)),
                        profile_vel=list(struct.unpack_from('<5i', b, 308)),
                        gripper_goal_current=struct.unpack_from('<h', b, 340)[0])
        if b[16] != 1 or b[199] != 1:
            raise Fault('Manipulator disconnected or torque unavailable; never auto-enable')
        if self.armed and b[15] != 1:
            raise Fault('OpenCR command heartbeat lost')
        low, high = self.cfg['control']['voltage_range']
        if not low <= voltage <= high:
            raise Fault('OpenCR voltage outside operating envelope')
        if any(base_command) or any(abs(v) > 1 for v in wheel_velocity):
            raise Fault('Base motion is outside this stationary task')
        if self.last is not None and any(abs(a-b) > 3 for a, b in zip(wheel_ticks, self.last.wheel_ticks)):
            raise Fault('Base encoder movement detected')
        if abs(current) > self.cfg['control']['current_abort_raw']:
            raise Fault('Gripper current exceeded commissioned limit')
        self.last_raw = raw_info
        self.last = RobotSample(now, q, dq, finger, finger_v, current, voltage, wheel_ticks, millis)
        return self.last

    def _write(self, address, value):
        if not self.execute:
            raise Fault('Read-only transport cannot write')
        allowed = {19, 200, 216, 220, 221, 284, 300, 304, 305, 306,
                   308, 324, 328, 329, 330, 340, 343, 344}
        if address not in allowed:
            raise Fault('Register outside arm-only write allowlist')
        comm, error = self.packet.writeTxRx(self.port, self.c['id'], address, len(value), list(value))
        if comm != self.ok or error:
            raise Fault(f'OpenCR write failed at {address}')
        self.writes += 1

    def beat(self):
        self.heartbeat = (self.heartbeat+1) % 256
        self._write(19, bytes([self.heartbeat]))

    def arm(self):
        if not self.execute or self.last is None or self.last_raw['ros']:
            raise Fault('Cannot arm this transport')
        # Reject invalid settings BEFORE even a heartbeat or profile-refresh write.
        limit = self.c['gripper_goal_current_raw']
        if type(limit) is not int or not 1 <= limit <= 80:
            raise Fault('Unapproved gripper current limit')
        for key in ('arm_profile_acceleration', 'arm_profile_velocity',
                    'gripper_profile_acceleration', 'gripper_profile_velocity'):
            value = self.c[key]
            if type(value) is not int or not 1 <= value <= 20:
                raise Fault('Unapproved nonzero motor profile: '+key)
        self._write(306, b'\x01'); self._write(330, b'\x01'); self._write(344, b'\x01')
        self.read()
        self.restore_values = (self.last_raw['profile_acc'], self.last_raw['profile_vel'])
        self.beat()
        self.command_q, self.command_finger = self.last.q.copy(), self.last.finger
        self.last_command_time = time.monotonic()
        # Capture current goals before changing profiles. No home/Stay command here.
        self._goals(self.command_q, self.command_finger)
        for base, commit, values in ((284, 304, [self.c['arm_profile_acceleration']]*4),
                                     (308, 328, [self.c['arm_profile_velocity']]*4)):
            self._write(base, struct.pack('<4i', *values)); self._write(commit, b'\x01')
        self._write(300, struct.pack('<i', self.c['gripper_profile_acceleration'])); self._write(305, b'\x01')
        self._write(324, struct.pack('<i', self.c['gripper_profile_velocity'])); self._write(329, b'\x01')
        self._write(340, struct.pack('<h', limit)); self._write(343, b'\x01')
        self.armed = True

    def _goals(self, q, finger):
        signs = np.asarray(self.c['joint_signs'])
        ticks = np.rint(q/signs*4096/math.tau+self.c['joint_zero_ticks']).astype(int)
        grip = round(finger/self.c['gripper_m_per_rad']*4096/math.tau+self.c['gripper_zero_tick'])
        if np.any(ticks < 0) or np.any(ticks > 4096) or not 0 <= grip <= 4096:
            raise Fault('Command encoder range rejected')
        self._write(200, struct.pack('<4i', *ticks)); self._write(220, b'\x01')
        self._write(216, struct.pack('<i', grip)); self._write(221, b'\x01')

    def command(self, q, finger):
        now = time.monotonic()
        q = finite(q, (4,))
        c = self.cfg['control']
        if not self.armed or now-self.last.stamp > c['max_cycle_s']:
            raise Fault('Command requires fresh feedback and armed transport')
        if not math.isfinite(finger) or not -.010 <= finger <= .019:
            raise Fault('Gripper command range rejected')
        dt = min(now-self.last_command_time, c['max_cycle_s'])
        if (np.max(np.abs(q-self.command_q)) > c['joint_velocity_rad_s']*dt+.002
                or abs(finger-self.command_finger) > c['gripper_velocity_m_s']*dt+.0002):
            raise Fault('Transport rate limit exceeded')
        if np.max(np.abs(q-self.last.q)) > c['max_tracking_error_rad']:
            raise Fault('Motor is not tracking the command')
        route = c.get('command_route', 'direct')
        if route == 'mp_control_equivalent':
            q, self.last_trajectory = mp_controller_trajectory(self.last.q, q)
        elif route != 'direct':
            raise Fault('Unknown joint command route')
        self.beat(); self._goals(q, finger)
        self.command_q, self.command_finger, self.last_command_time = q.copy(), finger, now

    def hold(self):
        if self.armed:
            try:
                sample = self.read()
                self.beat()
                # Keep gripper goal: never open a payload because of a fault.
                self._goals(sample.q, self.command_finger)
            finally:
                self.armed = False

    def close(self):
        if not self.closed:
            try:
                self.hold()
            finally:
                # No torque-off: the arm may carry a payload. Do not restore a
                # potentially faster profile/current limit after an interrupted task.
                self.port.closePort()
                self.closed = True
