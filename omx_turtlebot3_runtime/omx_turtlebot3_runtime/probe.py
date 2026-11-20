"""Explicit, bounded joint3 commissioning trial. NOT an autonomous pick mode.

Full-mission commissioning gates remain unchanged. This separate physical test
allows only joint3 to move monotonically by at most 22 encoder ticks (1.934 deg),
at <=0.03 rad/s commanded speed, for <=4 seconds. It cannot command the gripper,
wheels, torque, profiles or arbitrary arm goals. No automatic repeat or return.
"""
import argparse
from pathlib import Path
import struct
import sys
import time
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from .core import Fault, Phase, SlewLimiter, observation, residual_step
from .node import Runtime
from .opencr import OpenCR
from .robot_contract import mp_controller_trajectory
from .probe_limits import TICK, MAX_TICKS, ProbeEnvelope, command_interval


class ProbeTransport(OpenCR):
    """Narrow commissioning capability; does not enable the mission writer."""
    def __init__(self, cfg, execute_probe=False):
        self.probe_permitted = execute_probe
        self.envelope = None
        self.holding_probe = False
        super().__init__(cfg, execute=False)

    def _write(self, address, payload):
        if not self.probe_permitted or self.envelope is None:
            raise Fault('Probe write requires explicit execution and bounded envelope')
        self.envelope.validate(address, payload, time.monotonic(), self.holding_probe)
        comm, error = self.packet.writeTxRx(self.port, self.c['id'], address, len(payload), list(payload))
        if comm != self.ok or error:
            raise Fault('Probe SDK write failed')
        self.writes += 1

    def read(self):
        sample = super().read()
        if max(abs(x) for x in self.last_raw['arm_currents_raw']) > 100:
            raise Fault('Probe arm current limit')
        if self.envelope is not None:
            if abs(sample.dq[2]) > .12 or np.max(np.abs(sample.dq[[0,1,3]])) > .08:
                raise Fault('Probe measured velocity limit')
            if abs(self.last_raw['finger_tick']-self.envelope.finger_tick) > 4:
                raise Fault('Gripper moved during no-contact probe')
            ticks = np.asarray(self.last_raw['ticks'])
            if np.max(np.abs((ticks-self.envelope.start)[[0, 1, 3]])) > 4:
                raise Fault('Non-tested joint drift')
            if not self.envelope.start[2]-2 <= ticks[2] <= self.envelope.start[2]+MAX_TICKS+2:
                raise Fault('Measured joint3 escaped probe envelope')
        return sample

    def check_probe_start(self):
        if self.envelope is not None or self.last_raw['ros']:
            raise Fault('Probe unavailable or already used')
        if self.c['joint_signs'] != [1, 1, 1, 1] or self.c['joint_zero_ticks'] != [2048]*4:
            raise Fault('Unexpected controller/encoder coordinate contract')
        if any(not 1 <= n <= 5 for key in ('profile_acc', 'profile_vel') for n in self.last_raw[key][:4]):
            raise Fault('Probe requires existing low motor profiles; never changes profiles')
        if self.last is None or np.max(np.abs(self.last.dq)) > .03:
            raise Fault('Probe requires a stationary starting arm')

    def arm_probe(self):
        if not self.probe_permitted:
            raise Fault('Probe execution not requested')
        self.check_probe_start()
        if self.last is None or time.monotonic()-self.last.stamp > .10:
            raise Fault('Probe start requires fresh feedback')
        self.envelope = ProbeEnvelope(self.last_raw['ticks'], self.last_raw['finger_tick'], time.monotonic())
        self.command_q, self.command_finger = self.last.q.copy(), self.last.finger
        self.last_command_time = time.monotonic()
        self.beat()
        self._probe_goals(self.last.q)
        self.armed = True

    def _probe_goals(self, q):
        ticks = self.envelope.start.copy()
        ticks[2] = round(float(q[2])/TICK+2048)
        self._write(200, struct.pack('<4i', *ticks))
        self._write(220, b'\x01')

    def command_probe(self, q):
        now = time.monotonic()
        if not self.armed:
            raise Fault('Probe not armed')
        dt = command_interval(now, self.last_command_time, self.last.stamp)
        if abs(q[2]-self.command_q[2]) > .03*dt+1e-6:
            raise Fault(f'Probe command speed limit: delta={q[2]-self.command_q[2]:.8f}, dt={dt:.6f}')
        if abs(q[2]-self.last.q[2]) > 6*TICK:
            raise Fault('Probe tracking error')
        q, self.last_trajectory = mp_controller_trajectory(self.last.q, q)
        self.beat()
        self._probe_goals(q)
        self.command_q, self.last_command_time = q.copy(), now

    def hold(self):
        if self.envelope is None or not self.armed:
            return
        self.holding_probe = True
        try:
            try:
                sample = self.read()
                target = sample.q.copy()
            except Exception:
                target = self.command_q.copy()  # Last bounded command if feedback failed.
            target[2] = np.clip(target[2], (self.envelope.start[2]-2048)*TICK,
                                (self.envelope.start[2]+MAX_TICKS-2048)*TICK)
            self.beat()
            self._probe_goals(target)
        finally:
            self.armed = False
            self.holding_probe = False


class ProbeRuntime(Runtime):
    def __init__(self, args):
        self.done = False
        self.probe_started = None
        self.warmed = False
        self.probe_args = args
        super().__init__(args, transport_factory=lambda cfg, execute: ProbeTransport(cfg, args.execute_probe))
        self.emit('PROBE_MODE', execute_probe=args.execute_probe, full_mission_enabled=False,
                  maximum_joint3_degrees=float(np.rad2deg(MAX_TICKS*TICK)), gripper_commands=False)

    def on_request(self, message):
        self.emit('REQUEST_REJECTED', reason='Probe process cannot accept pickup missions')

    def on_cancel(self, message):
        if message.data:
            self.stop('Operator cancelled bounded probe')
            self.done = True

    def tick(self):
        if self.done:
            return
        super().tick()  # Telemetry + timestamp history only: mission never becomes active.
        if self.fault:
            self.done = True
            return
        now, sample = time.monotonic(), self.transport.last
        try:
            if self.probe_started is None:
                if now-self.started > 10:
                    raise Fault('Probe requires three fresh observations of both objects')
                if (not all(self.cache.fresh(i, now, .2) for i in (0, 1))
                        or not all(self.camera_counts[f'pose_{i}'] >= 3 for i in (0, 1))):
                    return
                if self.planner.robot_urdf is None or np.max(np.abs(sample.q-self.planner.stay)) > .08:
                    raise Fault('Probe requires verified URDF binding and near-Stay start')
                self.start_q = sample.q.copy()
                self.goal_q = sample.q.copy(); self.goal_q[2] += MAX_TICKS*TICK
                if not self.warmed:
                    # First TorchScript call can compile/optimize. Do this with
                    # zero write capability, then reacquire fresh inputs.
                    eef = self.planner.forward(sample.q, sample.finger)
                    place = np.r_[self.cfg['estimation']['deck_center_xy'], self.cfg['estimation']['deck_top_z']]
                    obs = observation(sample, eef, self.goal_q, Phase.RECOGNIZE, 0, 1,
                        self.cache, 0, place, False, False, np.zeros(4), now, 0, now)
                    elapsed = []
                    for _ in range(3):
                        before = time.monotonic()
                        self.actor(obs)
                        elapsed.append(time.monotonic()-before)
                    self.warmed = True
                    self.emit('PROBE_WARMUP', inference_seconds=elapsed, motor_writes=self.transport.writes)
                    return
                self.planner.update_objects(self.cache, 0)
                self.planner.path(self.start_q, self.goal_q, Phase.RECOGNIZE)
                for fraction in np.linspace(0, 1, 7):
                    self.planner.check(self.start_q+(self.goal_q-self.start_q)*fraction,
                                       Phase.RECOGNIZE, sample.finger)
                self.initial_objects = [p.position.copy() for p in self.cache.items]
                eef = self.planner.forward(sample.q, sample.finger)
                distances = [float(np.linalg.norm(eef-p)) for p in self.initial_objects]
                if min(distances) < .12:
                    raise Fault('Probe too close to an object')
                self.transport.check_probe_start()
                self.emit('PROBE_PATH_CHECKED', start_q=self.start_q, goal_q=self.goal_q,
                          object_distance_m=distances, raw_feedback=self.transport.last_raw,
                          motor_writes=self.transport.writes)
                if not self.probe_args.execute_probe:
                    self.done = True
                    return
                self.transport.arm_probe()
                self.probe_started = time.monotonic()
                self.target_q, self.previous_action = sample.q.copy(), np.zeros(4)
                # Leave deadline/rate headroom for inference + serial scheduling.
                limits = dict(self.cfg['control'], joint_velocity_rad_s=.02, joint_acceleration_rad_s2=.1)
                self.probe_limiter = SlewLimiter(limits)
                self.emit('PROBE_ARMED', motor_writes=self.transport.writes)
                return
            if now-self.probe_started >= 3.:
                self.transport.hold()
                final = self.transport.read()
                self.emit('PROBE_DONE', measured_delta_rad=final.q-self.start_q,
                          reached_endpoint=abs(final.q[2]-self.goal_q[2]) <= 3*TICK,
                          motor_writes=self.transport.writes, not_a_pickup=True)
                self.done = True
                return
            if not all(self.cache.fresh(i, now, .5) for i in (0, 1)):
                raise Fault('Probe visual observation expired')
            if any(np.linalg.norm(p.position-start) > .01 for p, start in zip(self.cache.items, self.initial_objects)):
                raise Fault('Camera/FK consistency or scene changed during probe')
            self.planner.update_objects(self.cache, 0)
            eef = self.planner.forward(sample.q, sample.finger)
            if any(np.linalg.norm(eef-p.position) < .12 for p in self.cache.items):
                raise Fault('Probe too close to an object')
            place = np.r_[self.cfg['estimation']['deck_center_xy'], self.cfg['estimation']['deck_top_z']]
            obs = observation(sample, eef, self.goal_q, Phase.RECOGNIZE, 0, 1,
                self.cache, 0, place, False, False, self.previous_action,
                self.probe_started, 0, now)
            raw_action = self.actor(obs)
            dt = command_interval(time.monotonic(), self.transport.last_command_time, sample.stamp)
            delta, action = residual_step(self.target_q, self.goal_q, raw_action,
                Phase.RECOGNIZE, 0, 1, sample.q, self.training, dt)
            delta[[0,1,3]] = 0.
            action[[0,1,3]] = 0.
            delta[2] = max(0., delta[2])
            candidate = self.target_q+self.probe_limiter.step(delta, dt)
            candidate[[0,1,3]] = self.start_q[[0,1,3]]
            candidate[2] = min(self.goal_q[2], max(self.target_q[2], candidate[2]))
            self.planner.path(sample.q, candidate, Phase.RECOGNIZE)
            self.planner.check(candidate, Phase.RECOGNIZE, sample.finger)
            self.transport.command_probe(candidate)
            self.target_q, self.previous_action = candidate, action
            self.emit('PROBE_CONTROL', observation=obs, actor_action=raw_action, applied_action=action,
                      target_q=candidate, measured_q=sample.q, trajectory=self.transport.last_trajectory,
                      motor_writes=self.transport.writes)
        except Exception as exc:
            self.stop(str(exc))
            self.done = True


def main():
    root = Path(get_package_share_directory('omx_turtlebot3_runtime'))
    models = Path(get_package_share_directory('omx_turtlebot3_policy'))/'models/omx_turtlebot3_v6'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-probe', action='store_true', help='Explicit bounded joint3-only physical test; not a mission')
    parser.add_argument('--log-dir', type=Path, required=True)
    args = parser.parse_args()
    args.config, args.scene, args.model_dir = root/'config/runtime.yaml', root/'assets/scene.xml', models
    args.execute, args.check_plan_id, args.backend, args.device = False, None, 'torchscript', 'cpu'
    rclpy.init()
    node = None
    try:
        node = ProbeRuntime(args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.025)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
        rclpy.shutdown()
    return 1 if node is not None and node.fault else 0


if __name__ == '__main__':
    sys.exit(main())
