"""Verify startup sequencing independently of ROS and physical hardware."""
from types import SimpleNamespace
import numpy as np
import pytest
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.startup import StayStartup


class Transport:
    def __init__(self):
        self.sample = SimpleNamespace(q=np.zeros(4), finger=.01)
        self.calls = []
        self.arrived = False

    def read(self):
        self.calls.append('read')
        return self.sample

    def begin_stay(self, goal):
        self.calls.append('begin')

    def stay_step(self):
        self.calls.append('step')
        return self.arrived

    def finish_stay(self):
        self.calls.append('handoff')


def setup(check=None):
    transport = Transport()
    def path(q, finger):
        transport.calls.append('path')
        if check:
            check()
    planner = SimpleNamespace(stay=np.ones(4)*.01, stay_path=path)
    return transport, StayStartup(transport, planner)


def test_waits_for_deadman_and_feedback_before_mission():
    transport, startup = setup()
    assert startup.step(transport.sample, False) == 'WAIT_DEADMAN'
    assert transport.calls == []
    with pytest.raises(Fault, match='completed Stay'):
        startup.handoff()
    assert startup.step(transport.sample, True) == 'MOVING_STAY'
    assert transport.calls == ['path', 'read', 'begin']
    assert startup.step(transport.sample, True) == 'MOVING_STAY'
    transport.arrived = True
    assert startup.step(transport.sample, True) == 'READY'
    startup.step(transport.sample, True)  # Keep native watchdog alive while waiting.
    startup.handoff()
    assert startup.state == 'MISSION' and transport.calls[-1] == 'handoff'


def test_collision_refusal_happens_before_native_begin():
    def collision():
        raise Fault('Collision')
    transport, startup = setup(collision)
    with pytest.raises(Fault, match='Collision'):
        startup.step(transport.sample, True)
    assert transport.calls == ['path']


def test_detects_pose_change_during_path_preflight():
    transport, startup = setup()
    sample = SimpleNamespace(q=np.zeros(4), finger=.01)
    transport.sample.q[0] = .02
    with pytest.raises(Fault, match='moved during Stay'):
        startup.step(sample, True)
    assert 'begin' not in transport.calls


@pytest.mark.parametrize('ready', [False, True])
def test_deadman_expiry_stops_both_motion_and_ready_hold(ready):
    transport, startup = setup()
    startup.step(transport.sample, True)
    transport.arrived = ready
    startup.step(transport.sample, True)
    with pytest.raises(Fault, match='deadman expired'):
        startup.step(transport.sample, False)


def test_ready_pose_cannot_drift_before_handoff():
    transport, startup = setup()
    startup.step(transport.sample, True)
    transport.arrived = True
    startup.step(transport.sample, True)
    transport.arrived = False
    with pytest.raises(Fault, match='Stay lost'):
        startup.step(transport.sample, True)


@pytest.mark.parametrize('state', ['WAIT_DEADMAN', 'MOVING_STAY'])
def test_runtime_rejects_requests_before_startup_ready(state):
    from omx_turtlebot3_runtime.node import Runtime
    records = []
    node = SimpleNamespace(fault=None, pending=None,
        mission=SimpleNamespace(active=False, done=False),
        startup=SimpleNamespace(state=state),
        emit=lambda event, **payload: records.append((event, payload)))
    Runtime.on_request(node, SimpleNamespace(data='{"request_id":"early","object_id":0}'))
    assert records[0][0] == 'REQUEST_REJECTED'
    assert records[0][1]['reason'] == 'Stay startup incomplete: '+state


def test_one_shot_runtime_finishes_and_shuts_down_after_mission(monkeypatch, tmp_path):
    from omx_turtlebot3_runtime import node
    import time
    calls = []
    instance = SimpleNamespace(plan_checked=False, fault=None, started=time.monotonic(),
        mission=SimpleNamespace(done=False), finish=lambda: calls.append('finish'))
    monkeypatch.setattr(node, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(node, 'Runtime', lambda args: instance)
    monkeypatch.setattr(node.rclpy, 'init', lambda **_: calls.append('init'))
    monkeypatch.setattr(node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(node.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    def spin(runtime, **_):
        calls.append('spin')
        assert calls.count('spin') <= 2, 'Completed mission must not keep the runtime spinning'
        runtime.mission.done = calls.count('spin') == 2
    monkeypatch.setattr(node.rclpy, 'spin_once', spin)
    node.main(['--backend', 'cpp_onnx', '--execute', '--exit-after-mission'])
    assert calls == ['init', 'spin', 'spin', 'finish', 'shutdown']
