import struct
import time
import pytest
pytest.importorskip('rclpy')
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.probe import ProbeTransport
from omx_turtlebot3_runtime.probe_limits import ProbeEnvelope

def bare():
    d = ProbeTransport.__new__(ProbeTransport)
    d.probe_permitted = False
    d.envelope = None
    d.holding_probe = False
    d.c, d.ok, d.port, d.writes = {'id':200}, 0, None, 0
    d.sent = []
    class Packet:
        def writeTxRx(self, *args):
            d.sent.append(args)
            return 0,0
    d.packet = Packet()
    return d

def test_probe_cannot_write_without_explicit_capability():
    d=bare()
    with pytest.raises(Fault): d._write(19,b'\x01')
    d.probe_permitted=True
    with pytest.raises(Fault): d._write(19,b'\x01')
    assert d.writes == 0 and not d.sent

def test_probe_validates_before_sdk_write():
    d=bare(); d.probe_permitted=True
    d.envelope=ProbeEnvelope([2114,2071,2048,992],2115,time.monotonic())
    with pytest.raises(Fault): d._write(216,struct.pack('<i',2048))
    with pytest.raises(Fault): d._write(200,struct.pack('<4i',2115,2071,2048,992))
    assert not d.sent
    d._write(200,struct.pack('<4i',2114,2071,2049,992))
    assert d.writes == 1
