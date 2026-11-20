import struct
import numpy as np
import pytest
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.probe_limits import ProbeEnvelope, MAX_TICKS, TICK, command_interval

START = [2114, 2071, 2048, 992]
def payload(delta=0, joint=2):
    ticks = START.copy(); ticks[joint] += delta
    return struct.pack('<4i', *ticks)

def test_envelope_total_is_below_two_degrees():
    assert np.rad2deg(MAX_TICKS*TICK) < 2.
    e = ProbeEnvelope(START, 2115, 10.)
    for step in range(MAX_TICKS+1):
        e.validate(200, payload(step), 10.+step*.1)
        e.validate(220, b'\x01', 10.+step*.1)
    with pytest.raises(Fault, match='total travel'):
        e.validate(200, payload(MAX_TICKS+1), 13.)

@pytest.mark.parametrize('joint', [0,1,3])
def test_other_joints_cannot_be_commanded(joint):
    with pytest.raises(Fault, match='joint3 only'):
        ProbeEnvelope(START, 2115, 10.).validate(200, payload(1,joint), 10.1)

@pytest.mark.parametrize('address', [0,7,149,150,199,216,221,284,304,308,328,340,343])
def test_probe_write_allowlist_is_narrow(address):
    with pytest.raises(Fault):
        ProbeEnvelope(START, 2115, 10.).validate(address, b'\x01', 10.1)

def test_probe_rejects_expired_and_large_step():
    e = ProbeEnvelope(START, 2115, 10.)
    with pytest.raises(Fault, match='lease'):
        e.validate(19, b'\x01', 14.01)
    with pytest.raises(Fault, match='two ticks'):
        e.validate(200, payload(3), 10.1)

def test_hold_cannot_escape_total_or_move_other_joints():
    e = ProbeEnvelope(START, 2115, 10.)
    e.validate(200, payload(1), 10.1)
    e.validate(200, payload(0), 15., holding=True)
    with pytest.raises(Fault):
        e.validate(200, payload(MAX_TICKS+1), 15., holding=True)
    with pytest.raises(Fault):
        e.validate(200, payload(1,0), 15., holding=True)

def test_default_full_mission_gates_remain_closed():
    from pathlib import Path
    import yaml
    from omx_turtlebot3_runtime.core import commissioning_issues
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/runtime.yaml').read_text())
    assert 'camera_extrinsics_verified' in commissioning_issues(cfg)
    assert 'current_thresholds_missing' in commissioning_issues(cfg)

def test_interval_uses_actual_command_clock_after_inference():
    # Previous cold inference ran until 10.12; next callback started at 10.15.
    # Counting from the old callback at 10.02 would allow too large a step.
    assert command_interval(10.16, 10.12, 10.15) == pytest.approx(.04)
    for previous, feedback in ((10., 10.15), (10.15, 10.), (10.16, 10.15)):
        with pytest.raises(Fault):
            command_interval(10.16, previous, feedback)

@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_interval_rejects_nonfinite_times(bad):
    with pytest.raises(Fault):
        command_interval(bad, 10., 10.)
