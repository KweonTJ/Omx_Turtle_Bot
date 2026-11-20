import struct
import time
from pathlib import Path
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.opencr import OpenCR
from omx_turtlebot3_runtime.core import Fault


def bare():
    driver = OpenCR.__new__(OpenCR)
    driver.cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/runtime.yaml').read_text())
    driver.c = driver.cfg['serial']
    driver.last = None
    driver.armed = False
    driver.execute = False
    driver.ok, driver.port, driver.writes = 0, None, 0
    return driver


def packet(millis=1000):
    raw = bytearray(344)
    struct.pack_into('<I', raw, 10, millis)
    raw[16], raw[199] = 1, 1
    struct.pack_into('<I', raw, 42, 1217)
    struct.pack_into('<4i', raw, 224, 2048, 2048, 2048, 2048)
    struct.pack_into('<i', raw, 240, 2048)
    struct.pack_into('<h', raw, 272, -20)
    return raw


def feed(driver, raw):
    class Packet:
        def readTxRx(self, *args):
            return list(raw), 0, 0
    driver.packet = Packet()
    return driver.read()


def test_read_only_rejects_all_writes():
    d = bare()
    with pytest.raises(Fault, match='Read-only'):
        d._write(19, b'\x01')
    d.hold()
    assert d.writes == 0


def test_decoder_preserves_current_sign_and_correct_voltage():
    d = bare()
    s = feed(d, packet())
    assert s.voltage == 12.17 and s.gripper_current_raw == -20
    np.testing.assert_array_equal(s.q, 0)
    assert s.finger == 0 and d.writes == 0


def test_stale_encoder_feedback_rejected():
    d = bare()
    feed(d, packet())
    with pytest.raises(Fault, match='stalled'):
        feed(d, packet())


def test_base_motion_and_torque_loss_rejected():
    for address, value in ((150, 1), (128, 10), (199, 0)):
        d, raw = bare(), packet()
        if address == 199:
            raw[address] = value
        else:
            struct.pack_into('<i', raw, address, value)
        with pytest.raises(Fault):
            feed(d, raw)


def test_no_wheel_or_torque_register_can_be_written():
    d = bare()
    d.execute = True
    for address in (149, 150, 199, 0, 7):
        with pytest.raises(Fault, match='allowlist'):
            d._write(address, b'\x01')
    assert d.writes == 0


def test_invalid_arm_profile_or_current_is_rejected_before_any_write():
    for key, value in (('arm_profile_velocity', 0), ('arm_profile_acceleration', 21),
                       ('gripper_goal_current_raw', 81), ('gripper_goal_current_raw', True)):
        d = bare()
        feed(d, packet())
        d.execute = True
        d.c[key] = value
        with pytest.raises(Fault, match='Unapproved'):
            d.arm()
        assert d.writes == 0


def test_mp_route_reaches_serial_goals_without_double_joint4_correction():
    d = bare()
    feed(d, packet())
    d.armed, d.execute = True, True
    d.command_q, d.command_finger = np.zeros(4), 0.
    d.last_command_time = time.monotonic()-.05
    goals = []
    d.beat = lambda: None
    d._goals = lambda q, finger: goals.append((q.copy(), finger))
    target = np.array([.001, .001, .003, -.004])
    d.command(target, 0.)
    np.testing.assert_allclose(goals[0][0], target)
    assert d.last_trajectory['raw_q'][2] == pytest.approx(-.003)
    assert d.last_trajectory['controller_q'][3] == pytest.approx(-.004)
    assert d.writes == 0  # Mock sink; no real SDK write.
