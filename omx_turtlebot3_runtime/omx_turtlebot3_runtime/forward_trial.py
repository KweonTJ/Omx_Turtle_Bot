"""One-shot 10 mm coordinated whole-arm forward trial. No grasp or return."""
import argparse
from pathlib import Path
import struct
import sys
import time
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from .core import Fault, Phase, finite
from .node import Runtime
from .opencr import OpenCR
from .robot_contract import mp_controller_trajectory
from .probe_limits import command_interval
from .forward_path import TICK, ForwardPath, ForwardEnvelope


class ForwardTransport(OpenCR):
    def __init__(self, cfg, permitted=False):
        self.permitted, self.envelope, self.holding = permitted, None, False
        super().__init__(cfg, execute=False)

    def _write(self, address, payload):
        if not self.permitted or self.envelope is None:
            raise Fault('Forward write requires explicit bounded trial')
        self.envelope.validate(address, payload, time.monotonic(), self.holding)
        comm, error = self.packet.writeTxRx(self.port, self.c['id'], address, len(payload), list(payload))
        if comm != self.ok or error:
            raise Fault('Forward SDK write failed')
        self.writes += 1

    def read(self):
        sample = super().read()
        if max(abs(x) for x in self.last_raw['arm_currents_raw']) > 100:
            raise Fault('Forward arm current limit')
        if self.envelope is not None:
            ticks = np.array(self.last_raw['ticks'])
            if np.any(ticks < self.envelope.low-4) or np.any(ticks > self.envelope.high+4):
                raise Fault('Measured arm escaped forward envelope')
            if np.max(np.abs(sample.dq)) > .15:
                raise Fault('Measured arm speed exceeded forward limit')
            if abs(self.last_raw['finger_tick']-self.envelope.finger_tick) > 4:
                raise Fault('Finger moved during no-grasp trial')
        return sample

    def check_start(self):
        if self.envelope is not None or self.last_raw['ros']:
            raise Fault('Forward unavailable or already used')
        if self.c['joint_signs'] != [1]*4 or self.c['joint_zero_ticks'] != [2048]*4:
            raise Fault('Unexpected encoder coordinate contract')
        if any(not 1 <= n <= 5 for key in ('profile_acc','profile_vel') for n in self.last_raw[key][:4]):
            raise Fault('Existing low arm profiles required; never changed automatically')

    def arm_forward(self, path):
        if not self.permitted:
            raise Fault('Forward execution not requested')
        self.check_start()
        if time.monotonic()-self.last.stamp > .1:
            raise Fault('Forward start feedback expired')
        self.envelope = ForwardEnvelope(path, self.last_raw['ticks'], self.last_raw['finger_tick'], time.monotonic())
        self.command_q, self.command_finger = self.last.q.copy(), self.last.finger
        self.last_command_time = time.monotonic()
        # Mark before the first write so cleanup also covers partially failed arming.
        self.armed = True
        self.beat()
        self._arm_goals(self.command_q)

    def _arm_goals(self, q):
        ticks = np.rint(q/TICK+2048).astype(int)
        self._write(200, struct.pack('<4i', *ticks))
        self._write(220, b'\x01')

    def command_forward(self, q):
        q = finite(q, (4,))
        now = time.monotonic()
        if not self.armed:
            raise Fault('Forward transport not armed')
        dt = command_interval(now, self.last_command_time, self.last.stamp)
        if np.max(np.abs(q-self.command_q)) > .03*dt+1e-6:
            raise Fault('Forward commanded speed limit')
        if np.max(np.abs(q-self.last.q)) > 10*TICK:
            raise Fault('Forward tracking error')
        q, self.last_trajectory = mp_controller_trajectory(self.last.q, q)
        self.beat()
        self._arm_goals(q)
        self.command_q, self.last_command_time = q.copy(), now

    def hold(self):
        if not self.armed or self.envelope is None:
            return
        self.holding = True
        try:
            try:
                target = self.read().q.copy()
            except Exception:
                target = self.command_q.copy()
            ticks = np.clip(np.rint(target/TICK+2048), self.envelope.low, self.envelope.high)
            self.beat()
            self._arm_goals((ticks-2048)*TICK)
        finally:
            self.armed, self.holding = False, False


class ForwardRuntime(Runtime):
    def __init__(self, args):
        self.done, self.path, self.motion_started = False, None, None
        self.progress, self.settled_since = 0., None
        super().__init__(args, transport_factory=lambda cfg, execute: ForwardTransport(cfg,args.execute_forward))
        self.emit('FORWARD_MODE', execute_forward=args.execute_forward, distance_m=.010,
                  controller='coordinated_cartesian_reference_not_PPO', full_mission_enabled=False,
                  gripper_commands=False, automatic_return=False)

    def on_request(self, message):
        self.emit('REQUEST_REJECTED', reason='Forward trial cannot accept pickup requests')

    def on_cancel(self, message):
        if message.data:
            self.stop('Operator cancelled coordinated forward trial')
            self.done = True

    def tick(self):
        if self.done:
            return
        super().tick()
        if self.fault:
            self.done = True
            return
        now, sample = time.monotonic(), self.transport.last
        try:
            if self.motion_started is None:
                if now-self.started > 15:
                    raise Fault('Forward start observation timeout')
                if not all(self.cache.fresh(i,now,.2) and self.camera_counts[f'pose_{i}']>=3 for i in (0,1)):
                    return
                if self.planner.robot_urdf is None or np.max(np.abs(sample.q-self.planner.stay)) > .08:
                    raise Fault('Forward requires bound URDF and near-Stay start')
                self.transport.check_start()
                self.planner.update_objects(self.cache,0)
                if self.path is None:
                    self.path = ForwardPath(self.planner,sample.q,sample.finger)
                    self.initial_objects = [p.position.copy() for p in self.cache.items]
                    if min(np.linalg.norm(self.path.eef-p) for p in self.initial_objects) < .15:
                        raise Fault('Forward path too close to an object')
                    self.start_q, self.start_finger = sample.q.copy(), sample.finger
                    self.emit('FORWARD_PATH_CHECKED', start_q=self.path.start, points=self.path.points,
                              start_eef=self.path.eef, endpoint_eef=self.planner.forward(self.path.points[-1]),
                              pitch_rad=self.path.pitch, raw_feedback=self.transport.last_raw,
                              motor_writes=self.transport.writes)
                    if not self.args.execute_forward:
                        self.done=True
                    return  # Reacquire telemetry after the IK/path computation.
                if np.max(np.abs(sample.q-self.path.start)) > TICK/2:
                    self.path=None
                    return
                if any(np.linalg.norm(p.position-start)>.005 for p,start in zip(self.cache.items,self.initial_objects)):
                    self.path=None
                    return
                history=[s for s in self.history if now-s.stamp<=.5]
                if len(history)<5 or np.max(np.ptp([s.q for s in history],axis=0)) > 4*TICK:
                    return
                self.transport.arm_forward(self.path)
                self.motion_started=time.monotonic()
                self.emit('FORWARD_ARMED',motor_writes=self.transport.writes)
                return
            if now-self.motion_started > 12:
                raise Fault('Forward settling timeout')
            if not all(self.cache.fresh(i,now,.5) for i in (0,1)):
                raise Fault('Forward vision observation expired')
            if any(np.linalg.norm(p.position-start)>.015 for p,start in zip(self.cache.items,self.initial_objects)):
                raise Fault('Observed object/FK changed during forward trial')
            self.planner.update_objects(self.cache,0)
            eef=self.planner.forward(sample.q,sample.finger)
            if np.linalg.norm(eef-self.path.eef)>.020 or abs(eef[2]-self.path.eef[2])>.006:
                raise Fault('Measured Cartesian motion escaped short forward envelope')
            if abs(sum(sample.q[1:])-self.path.pitch)>.035:
                raise Fault('Measured tool pitch drift during forward trial')
            dt=command_interval(time.monotonic(),self.transport.last_command_time,sample.stamp)
            self.progress=min(1.,self.progress+dt/8.)
            smooth=3*self.progress**2-2*self.progress**3
            candidate=self.path.sample(smooth)
            self.planner.path(sample.q,candidate,Phase.RECOGNIZE)
            self.planner.check(candidate,Phase.RECOGNIZE,sample.finger)
            self.transport.command_forward(candidate)
            self.emit('FORWARD_CONTROL',progress=self.progress,target_q=candidate,measured_q=sample.q,
                      measured_eef=eef,trajectory=self.transport.last_trajectory,motor_writes=self.transport.writes)
            reached=self.progress==1 and np.max(np.abs(sample.q-self.path.points[-1]))<=4*TICK
            self.settled_since=(now if self.settled_since is None else self.settled_since) if reached else None
            if self.settled_since is not None and now-self.settled_since>=.4:
                self.transport.hold()
                final=self.transport.read()
                endpoint=self.planner.forward(final.q,final.finger)
                self.emit('FORWARD_DONE',q_delta_rad=final.q-self.start_q,
                          encoder_FK_delta_m=endpoint-self.path.eef,
                          source='encoders_with_URDF_not_external_metrology',
                          raw_feedback=self.transport.last_raw,motor_writes=self.transport.writes,
                          not_a_pickup=True,not_PPO_control=True)
                self.done=True
        except Exception as exc:
            self.stop(str(exc))
            self.done=True


def main():
    root=Path(get_package_share_directory('omx_turtlebot3_runtime'))
    models=Path(get_package_share_directory('omx_turtlebot3_policy'))/'models/omx_turtlebot3_v6'
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-forward',action='store_true')
    parser.add_argument('--log-dir',type=Path,required=True)
    args=parser.parse_args()
    args.config,args.scene,args.model_dir=root/'config/runtime.yaml',root/'assets/scene.xml',models
    args.execute,args.check_plan_id,args.backend,args.device=False,None,'torchscript','cpu'
    rclpy.init()
    node=None
    try:
        node=ForwardRuntime(args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.025)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.finish()
        rclpy.shutdown()
    return 1 if node is not None and node.fault else 0


if __name__=='__main__':
    sys.exit(main())
