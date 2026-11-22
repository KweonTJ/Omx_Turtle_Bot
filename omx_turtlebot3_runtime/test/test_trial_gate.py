import time
from types import SimpleNamespace
from omx_turtlebot3_runtime.ros_trial_gate import TrialGate,at_stay,STAY


def test_stay_gate_uses_named_measured_positions_and_stopped_velocity():
    names=['joint4','joint2','joint1','joint3'];q=[STAY[3],STAY[1],STAY[0],STAY[2]]
    assert at_stay(names,q,[0.]*4)
    assert not at_stay(names,q,[.1,0.,0.,0.])
    assert not at_stay(names,[0.]*4,[0.]*4)
    assert not at_stay(names,q,[])


def gate(execute):
    now=time.monotonic();commands=[];events=[]
    pub=SimpleNamespace(get_subscription_count=lambda:1,publish=lambda m:commands.append(m.data))
    node=SimpleNamespace(args=SimpleNamespace(execute=execute,startup_timeout=45.,mission_timeout=300.,selected_id=0),
        completed=False,fault=None,started=False,patched=True,controllers=True,
        inputs={'observer':(True,now),'policy':(True,now)},stay=(True,now),
        since=now-.4,requested=now,query=lambda:None,start_pub=pub,
        emit=lambda event,**_:events.append(event))
    return node,commands,events


def test_monitor_never_starts_the_gripper_or_mission():
    node,commands,events=gate(False)
    assert TrialGate.step(node)
    assert node.completed and not node.started and not commands
    assert events==['MONITOR_READY']


def test_operator_execute_starts_exactly_once_after_ready():
    node,commands,events=gate(True)
    assert not TrialGate.step(node)
    assert node.started and commands==[True]
    assert not TrialGate.step(node)
    assert commands==[True]


def test_unpatched_controller_cannot_start_even_with_live_markers():
    node,commands,events=gate(True);node.patched=False;node.requested-=46
    assert TrialGate.step(node)
    assert node.fault and not commands
