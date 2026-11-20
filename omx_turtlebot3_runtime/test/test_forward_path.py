from pathlib import Path
import struct
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.core import Fault, ObjectPose, PoseCache
from omx_turtlebot3_runtime.forward_path import ForwardPath, ForwardEnvelope, TICK
from omx_turtlebot3_runtime.planning import Planner

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def case():
    cfg=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training=yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    p=Planner(ROOT/'assets/scene.xml',training,cfg)
    cache=PoseCache();cache.begin_frame(1.)
    for i,xyz in enumerate(([.2155,.0875,.3826],[.2211,-.0273,.3826])):
        cache.update(ObjectPose(i,1.,np.array(xyz),np.array([0.,0.,0.,1.]),.2),1.)
    p.update_objects(cache,0)
    ticks=np.array([2114,2074,2048,994])
    path=ForwardPath(p,(ticks-2048)*TICK,-.00155)
    return p,path,ticks

def test_forward_uses_all_joints_and_holds_height_pitch(case):
    p,path,_=case
    delta=path.points[-1]-path.start
    assert np.all(np.abs(delta)>TICK)
    for progress in np.linspace(0,1,101):
        q=path.sample(progress)
        np.testing.assert_allclose(p.forward(q),path.eef+[.01*progress,0,0],atol=2e-6)
        assert abs(sum(q[1:])-path.pitch)<1e-6
    assert np.max(np.abs(delta))<.12

def test_no_forward_distance_expansion(case):
    p,path,_=case
    for distance in (0,-.01,.011,float('nan')):
        with pytest.raises(Fault):ForwardPath(p,path.start,-.00155,distance)
    for progress in (-.1,1.1,float('nan')):
        with pytest.raises(Fault):path.sample(progress)

def test_whole_checked_path_passes_write_envelope(case):
    _,path,ticks=case
    envelope=ForwardEnvelope(path,ticks,2115,10.)
    for i,progress in enumerate(np.linspace(0,1,201)):
        goals=np.rint(path.sample(progress)/TICK+2048).astype(int)
        now=10.+i*.04
        envelope.validate(19,b'\x01',now)
        envelope.validate(200,struct.pack('<4i',*goals),now)
        envelope.validate(220,b'\x01',now)

@pytest.mark.parametrize('address',[0,7,149,150,199,216,221,284,304,308,328,340,343])
def test_forward_forbids_non_arm_goal_registers(case,address):
    _,path,ticks=case
    with pytest.raises(Fault):
        ForwardEnvelope(path,ticks,2115,10.).validate(address,b'\x01',10.)

def test_forward_rejects_mixed_uncoordinated_goal(case):
    _,path,ticks=case
    envelope=ForwardEnvelope(path,ticks,2115,10.)
    goals=ticks.copy();goals[1]+=3
    with pytest.raises(Fault,match='coordinated path'):
        envelope.validate(200,struct.pack('<4i',*goals),10.05)

def test_forward_step_lease_and_hold_are_bounded(case):
    _,path,ticks=case
    envelope=ForwardEnvelope(path,ticks,2115,10.)
    with pytest.raises(Fault,match='lease'):
        envelope.validate(19,b'\x01',25.01)
    goals=np.rint(path.points[-1]/TICK+2048).astype(int)
    with pytest.raises(Fault,match='step/deadline'):
        envelope.validate(200,struct.pack('<4i',*goals),10.05)
    envelope.validate(200,struct.pack('<4i',*ticks),30.,holding=True)
    goals[1]=envelope.high[1]+1
    with pytest.raises(Fault,match='total travel'):
        envelope.validate(200,struct.pack('<4i',*goals),30.,holding=True)

def test_mismatched_encoder_start_rejected(case):
    _,path,ticks=case
    with pytest.raises(Fault,match='start differs'):
        ForwardEnvelope(path,ticks+[0,2,0,0],2115,10.)
