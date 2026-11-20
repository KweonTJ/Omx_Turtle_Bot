"""Exercise the real C++ Dynamixel SDK over a Linux pseudo-terminal, never a robot."""
import errno
import os
import pty
import select
import struct
import threading
import time
from pathlib import Path
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.cpp_backend import NativeTransport

ROOT = Path(__file__).resolve().parents[1]


def crc(data):
    value = 0
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x8005 if value & 0x8000 else 0)) & 0xffff
    return struct.pack('<H', value)


def status(data=b''):
    payload = (b'\x55\x00'+data).replace(b'\xff\xff\xfd', b'\xff\xff\xfd\xfd')
    packet = b'\xff\xff\xfd\x00\xc8'+struct.pack('<H', len(payload)+2)+payload
    return packet+crc(packet)


class OpenCrEmulator:
    def __init__(self):
        self.master, slave = pty.openpty()
        self.device = os.ttyname(slave)
        os.close(slave)
        self.memory = bytearray(345)
        self.memory[16] = self.memory[199] = 1
        struct.pack_into('<I', self.memory, 42, 1217)
        struct.pack_into('<4i', self.memory, 224, *([2048]*4))
        struct.pack_into('<i', self.memory, 240, 2048)
        struct.pack_into('<h', self.memory, 272, -20)
        self.writes = []
        self.errors = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        pending = bytearray()
        try:
            while not self.stop.is_set():
                if not select.select([self.master], [], [], .01)[0]:
                    continue
                try:
                    pending.extend(os.read(self.master, 4096))
                except OSError as e:
                    if e.errno == errno.EIO:  # Slave is not open yet.
                        self.stop.wait(.005)
                        continue
                    raise
                while len(pending) >= 7:
                    assert pending[:4] == b'\xff\xff\xfd\x00'
                    length = struct.unpack_from('<H', pending, 5)[0]+7
                    if len(pending) < length:
                        break
                    packet = bytes(pending[:length])
                    del pending[:length]
                    assert packet[-2:] == crc(packet[:-2])
                    assert packet[4] == 200
                    payload = packet[7:-2].replace(b'\xff\xff\xfd\xfd', b'\xff\xff\xfd')
                    instruction, params = payload[0], payload[1:]
                    if instruction == 1:
                        reply = status(struct.pack('<HB', 20480, 1))
                    elif instruction == 2:
                        address, size = struct.unpack('<HH', params)
                        millis = struct.unpack_from('<I', self.memory, 10)[0]+50
                        struct.pack_into('<I', self.memory, 10, millis)
                        reply = status(self.memory[address:address+size])
                    elif instruction == 3:
                        address = struct.unpack_from('<H', params)[0]
                        data = params[2:]
                        self.writes.append((address, bytes(data)))
                        self.memory[address:address+len(data)] = data
                        if address == 19:
                            self.memory[15] = 1
                        if address == 220:
                            self.memory[224:240] = self.memory[200:216]
                        if address == 221:
                            self.memory[240:244] = self.memory[216:220]
                        reply = status()
                    else:
                        raise AssertionError(f'Unexpected instruction {instruction}')
                    os.write(self.master, reply)
        except BaseException as exc:
            self.errors.append(exc)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        assert not self.thread.is_alive()
        assert not self.errors


@pytest.fixture
def emulated(tmp_path):
    emulator = OpenCrEmulator()
    cfg = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    cfg['serial']['device'] = emulator.device
    path = tmp_path/'synthetic_only.yaml'
    try:
        yield emulator, cfg, path
    finally:
        emulator.close()


def driver(cfg, path, execute=False):
    path.write_text(yaml.safe_dump(cfg))
    return NativeTransport(cfg, path, ROOT/'assets/robot_real.urdf', execute=execute,
                           library_path=os.environ.get('OMX_V6_CONTROL_TEST_LIBRARY'))


def test_cpp_sdk_reads_real_protocol_packets_without_writes(emulated):
    emulator, cfg, path = emulated
    transport = driver(cfg, path)
    try:
        sample = transport.read()
        assert sample.voltage == 12.17 and sample.gripper_current_raw == -20
        np.testing.assert_array_equal(sample.q, 0)
        with pytest.raises(Fault, match='Cannot arm'):
            transport.arm()
        assert transport.writes == 0 and emulator.writes == []
    finally:
        transport.close()


def test_cpp_sdk_grouped_command_feedback_and_hold(emulated):
    emulator, cfg, path = emulated
    for key in cfg['commissioning']:
        if key.endswith('_verified'):
            cfg['commissioning'][key] = True
    cfg['commissioning']['evidence_directory'] = 'SYNTHETIC_PTY_TEST_ONLY'
    cfg['estimation'].update(current_idle_raw=0, current_grip_delta_raw=10,
                             current_release_delta_raw=5)
    transport = driver(cfg, path, execute=True)
    try:
        transport.arm()
        transport.read()
        time.sleep(.03)
        goal = np.array([.001, .001, .001, -.001])
        transport.command(goal, 0)
        sample = transport.read()
        np.testing.assert_allclose(sample.q, goal, atol=np.pi/4096)
        before = len(emulator.writes)
        transport.hold()
        assert {address for address, _ in emulator.writes[before:]} == {19, 200, 220}
        assert all(address not in (149, 150, 198, 199) for address, _ in emulator.writes)
        assert transport.last_trajectory['route'] == 'cpp_mp_control_equivalent_opencr'
    finally:
        transport.close()
