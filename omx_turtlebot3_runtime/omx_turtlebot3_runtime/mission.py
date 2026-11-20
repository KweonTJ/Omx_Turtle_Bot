"""Nine phases driven by measured state; guarded residual PPO, no blind timed pickup."""
import math
import numpy as np
from .core import Fault, Phase, Evidence, SlewLimiter, observation, residual_step


class Mission:
    def __init__(self, cfg, training, planner, actor):
        self.cfg, self.t, self.planner, self.actor = cfg, training, planner, actor
        self.active = self.done = False
        self.phase = Phase.RECOGNIZE
        self.evidence = Evidence(cfg['estimation'])
        self.prev_action = np.zeros(4)
        self.limiter = SlewLimiter(cfg['control'])
        self.reason = 'IDLE'

    def start(self, physical_id, sample, cache, now):
        if self.active or self.done:
            raise Fault('One mission per process; explicit restart required')
        if type(physical_id) is not int or physical_id not in (0, 1):
            raise Fault('Select physical marker 0 or 1')
        if not all(cache.fresh(i, now) for i in (0, 1)):
            raise Fault('Both physical IDs must have fresh poses before mission planning')
        if np.max(np.abs(sample.q-self.planner.stay)) > self.cfg['control']['start_stay_tolerance_rad']:
            raise Fault('Robot not at verified Stay; automatic homing is prohibited')
        self.plans, self.place = self.planner.plan(cache, physical_id)
        self.selected = physical_id  # Physical 0/1 -> policy slot 0/1, label 1/2.
        self.phase, self.waypoint, self.recognition_stage = Phase.RECOGNIZE, 0, 0
        self.target, self.gripper_target = sample.q.copy(), sample.finger
        self.stable_since = None
        self.start_time = self.phase_time = self.last_time = self.last_plan = now
        self.overview_stamps, self.target_stamps = set(), set()
        self.planned_object = cache.items[physical_id].position.copy()
        self.initial_objects = [p.position.copy() for p in cache.items]
        self.rotation_complete = False
        self.rotation_height = None
        self.active, self.reason = True, 'RUNNING'

    @property
    def goal(self):
        return self.plans[self.phase][self.waypoint]

    def advance(self, now, eef, next_target):
        if self.phase == Phase.RETURN_STAY:
            self.active, self.done, self.reason = False, True, 'SUCCESS'
            return
        old = self.phase
        self.phase = Phase(int(self.phase)+1)
        self.phase_time, self.stable_since, self.waypoint = now, None, 0
        self.limiter.velocity[:] = 0
        if self.phase == Phase.ROTATE_RIGHT:
            self.rotation_height = float(eef[2])
            self.rotation_joint_hold = next_target[1:].copy()
            # Exactly preserve the height joints through joint1-only rotation.
            self.plans[self.phase][0][1:] = self.rotation_joint_hold
        if old == Phase.ROTATE_RIGHT:
            self.rotation_complete = True

    def step(self, sample, cache, now):
        if not self.active:
            raise Fault('Mission is not active')
        c = self.cfg['control']
        dt = now-self.last_time
        if not 0 < dt <= c['max_cycle_s'] or now-sample.stamp > c['max_cycle_s']:
            raise Fault('Control/feedback deadline missed')
        self.last_time = now
        if now-self.phase_time > c['phase_timeout_s'] or now-self.start_time > c['task_timeout_s']:
            raise Fault('Mission phase timeout')
        if np.max(np.abs(sample.q-self.target)) > c['max_tracking_error_rad']:
            raise Fault('Joint tracking error')
        if Phase.REACH <= self.phase <= Phase.GRIP and not cache.fresh(self.selected, now, c['pose_timeout_s']):
            raise Fault('Target vision expired during approach/grip')
        if self.phase >= Phase.EXTEND_LOWER and not self.rotation_complete:
            raise Fault('Descent forbidden before rotation completion')
        other = 1-self.selected
        if cache.fresh(other, now) and np.linalg.norm(cache.items[other].position-self.initial_objects[other]) > .020:
            raise Fault('Non-selected object displaced')
        if self.phase == Phase.REACH:
            current_target = cache.items[self.selected].position
            if np.linalg.norm(current_target-self.planned_object) > .005:
                # Stop for an explicitly revalidated plan rather than moving a stale
                # waypoint or recomputing expensive collision paths in the motor loop.
                raise Fault('Target moved; stop and validate a new plan')
        self.planner.update_objects(cache, self.selected)
        eef = self.planner.forward(sample.q, sample.finger)
        self.evidence.update(self.phase, sample, eef, cache, self.selected, now,
                             self.t['gripper_close'], self.t['gripper_open'])
        if self.phase == Phase.ROTATE_RIGHT:
            if abs(eef[2]-self.rotation_height) > self.t['motion']['rotation_height_tolerance']:
                raise Fault('Carry height drift')
            if (cache.fresh(self.selected, now)
                    and abs(cache.items[self.selected].position[2]+self.t['grasp_offset_z']-self.rotation_height) > .010):
                raise Fault('Observed payload height drift')
        if self.phase in (Phase.ROTATE_RIGHT, Phase.EXTEND_LOWER) and not self.evidence.grasp_confirmed:
            raise Fault('Transport requires visual lift proof, not just a stalled gripper')
        obs = observation(sample, eef, self.goal, self.phase,
                          0 if self.phase == Phase.RECOGNIZE else self.waypoint,
                          1 if self.phase == Phase.RECOGNIZE else len(self.plans[self.phase]), cache, self.selected, self.place,
                          self.evidence.contact, self.evidence.placed, self.prev_action,
                          self.last_plan, self.recognition_stage, now)
        requested_grip = self.t['gripper_open']
        if self.phase == Phase.RECOGNIZE or self.phase == Phase.RETURN_STAY and self.waypoint == len(self.plans[self.phase])-1:
            requested_grip = 0.
        elif Phase.GRIP <= self.phase <= Phase.EXTEND_LOWER:
            requested_grip = self.t['gripper_close']
        if hasattr(self.actor, 'control_step'):
            candidate, action, velocity, finger = self.actor.control_step(
                obs, self.target, self.goal, sample.q, self.limiter.velocity,
                self.phase, self.waypoint, len(self.plans[self.phase]), dt,
                self.gripper_target, requested_grip, self.planner.low, self.planner.high)
            self.limiter.velocity[:] = velocity
        else:
            delta, action = residual_step(self.target, self.goal, self.actor(obs), self.phase,
                                          self.waypoint, len(self.plans[self.phase]), sample.q, self.t, dt)
            candidate = np.clip(self.target+self.limiter.step(delta, dt), self.planner.low, self.planner.high)
            grip_step = c['gripper_velocity_m_s']*dt
            finger = self.gripper_target+np.clip(requested_grip-self.gripper_target, -grip_step, grip_step)
        if self.phase == Phase.ROTATE_RIGHT:
            candidate[1:] = self.rotation_joint_hold
        self.planner.path(sample.q, candidate, self.phase, Phase.RETRACT <= self.phase <= Phase.EXTEND_LOWER)
        arrived = np.linalg.norm(sample.q-self.goal) < self.t['joint_tolerance']
        last = self.waypoint == len(self.plans[self.phase])-1
        if self.phase == Phase.RECOGNIZE:
            if arrived and all(cache.fresh(i, now, .2) for i in (0, 1)):
                stamps = tuple(cache.items[i].stamp for i in (0, 1))
                if abs(stamps[0]-stamps[1]) < .03:
                    self.overview_stamps.add(stamps)
            if self.waypoint == 0 and len(self.overview_stamps) < 3:
                arrived = False
            if self.waypoint == 1:
                self.recognition_stage = 1
                if arrived and cache.fresh(self.selected, now, .2):
                    self.target_stamps.add(cache.items[self.selected].stamp)
                arrived &= len(self.target_stamps) >= 3
        if arrived and not last:
            # Lift is the only permitted motion before grasp confirmation.
            if self.phase != Phase.RETRACT or self.waypoint != 0 or self.evidence.grasp_confirmed:
                self.waypoint += 1
                self.stable_since = None
        elif last:
            ready = arrived
            if self.phase == Phase.OPEN:
                ready &= sample.finger > .017
            elif self.phase == Phase.REACH:
                ready &= np.linalg.norm(eef-(cache.items[self.selected].position+[0, 0, self.t['grasp_offset_z']])) < self.t['position_tolerance']
            elif self.phase == Phase.GRIP:
                ready &= self.evidence.contact
            elif self.phase == Phase.RETRACT:
                ready &= self.evidence.grasp_confirmed and self.evidence.contact
            elif self.phase == Phase.ROTATE_RIGHT:
                ready &= self.evidence.contact and abs(sample.q[0]+math.pi/2) < .02 and abs(sample.dq[0]) < .03
            elif self.phase == Phase.EXTEND_LOWER:
                ready &= self.evidence.contact and self.rotation_complete
            elif self.phase == Phase.PLACE:
                ready &= self.evidence.placed and not self.evidence.contact
            elif self.phase == Phase.RETURN_STAY:
                ready &= self.evidence.placed and np.max(np.abs(sample.q-self.planner.stay)) < .04
            self.stable_since = (now if self.stable_since is None else self.stable_since) if ready else None
            if self.stable_since is not None and now-self.stable_since >= c['stable_s']:
                self.advance(now, eef, candidate)
        self.target, self.gripper_target, self.prev_action = candidate, float(finger), action
        return candidate, float(finger), obs

    def abort(self, reason):
        self.active, self.done, self.reason = False, True, str(reason)
