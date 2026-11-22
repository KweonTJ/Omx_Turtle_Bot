"""Stay-first sequencing; all trajectory generation and hardware writes are native."""
import numpy as np
from .core import Fault


class StayStartup:
    def __init__(self, transport, planner):
        self.transport, self.planner = transport, planner
        self.state = 'WAIT_DEADMAN'

    @property
    def active(self):
        return self.state in ('MOVING_STAY', 'READY')

    def step(self, sample, deadman_fresh):
        if self.state == 'MISSION':
            return self.state
        if not deadman_fresh:
            if self.active:
                raise Fault('Operator deadman expired during Stay startup/hold')
            return self.state
        if self.state == 'WAIT_DEADMAN':
            # No motor writes until the entire path passes. Refresh encoders
            # afterwards so expensive geometry checks cannot arm stale feedback.
            self.planner.stay_path(sample.q, sample.finger)
            fresh = self.transport.read()
            if (np.max(np.abs(fresh.q-sample.q)) > .01
                    or abs(fresh.finger-sample.finger) > .001):
                raise Fault('Robot moved during Stay path validation; restart required')
            self.transport.begin_stay(self.planner.stay)
            self.state = 'MOVING_STAY'
        else:
            ready = self.transport.stay_step()
            if self.state == 'READY' and not ready:
                raise Fault('Verified Stay lost before mission handoff')
            if ready:
                self.state = 'READY'
        return self.state

    def handoff(self):
        if self.state != 'READY':
            raise Fault('Mission requires completed Stay startup')
        self.transport.finish_stay()
        self.state = 'MISSION'
