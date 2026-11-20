import struct
import time
import numpy as np
import pytest
pytest.importorskip('rclpy')
from omx_turtlebot3_runtime.core import Fault
from omx_turtlebot3_runtime.forward_trial import ForwardTransport
from omx_turtlebot3_runtime.forward_path import ForwardEnvelope,TICK

def test_forward_checks_before_sdk_writes():
    d=ForwardTransport.__new__(ForwardTransport)
    d.permitted,d.envelope,d.holding=False,None,False
    d.sent=[];d.c={'id':200};d.ok=0;d.port=None;d.writes=0
    class SDK:
        def writeTxRx(self,*args):
            d.sent.append(args);return 0,0
    d.packet=SDK()
    with pytest.raises(Fault):d._write(19,b'\x01')
    d.permitted=True
    with pytest.raises(Fault):d._write(19,b'\x01')
    class Path:
        start=np.zeros(4)
        points=np.array([[0.,0.,0.,0.],[.001,.01,-.01,0.]])
    d.envelope=ForwardEnvelope(Path(),[2048]*4,2115,time.monotonic())
    with pytest.raises(Fault):d._write(216,struct.pack('<i',2115))
    assert not d.sent
    d._write(200,struct.pack('<4i',*[2048]*4))
    assert d.writes==1
