import time
from types import SimpleNamespace
from omx_turtlebot3_runtime.ros_trial_gate import TrialGate,at_stay,held_and_stopped,STAY


def test_stay_gate_uses_named_measured_positions_and_stopped_velocity():
    names=['joint4','joint2','joint1','joint3'];q=[STAY[3],STAY[1],STAY[0],STAY[2]]
    assert at_stay(names,q,[0.]*4)
    assert not at_stay(names,q,[.1,0.,0.,0.])
    assert not at_stay(names,[0.]*4,[0.]*4)
    assert not at_stay(names,q,[])


def gate(execute):
    now=time.monotonic();commands=[];events=[]
    pub=SimpleNamespace(get_subscription_count=lambda:1,publish=lambda m:commands.append(m.data))
    node=SimpleNamespace(args=SimpleNamespace(execute=execute,startup_timeout=45.,mission_timeout=300.,selected_id=0,resume_held=False,retract_mode='cartesian'),
        completed=False,fault=None,started=False,patched=True,controllers=True,
        inputs={'observer':(True,now),'policy':(True,now)},stay=(True,now),
        since=now-.4,requested=now,query=lambda:None,start_pub=pub,
        emit=lambda event,**_:events.append(event))
    return node,commands,events


def test_held_recovery_requires_fresh_stopped_closed_finger_feedback():
    names=['joint1','joint2','joint3','joint4','gripper_left_joint']
    q=[.1,1.24,-2.62,-.36,.005]
    assert held_and_stopped(names,q,[0.]*5)
    assert not held_and_stopped(names,q[:4]+[.019],[0.]*5)
    assert not held_and_stopped(names,q,[0.,.1,0.,0.,0.])
    assert not held_and_stopped(names,q[:4]+[float('nan')],[0.]*5)
    assert not held_and_stopped(names,q,[])


def test_confirmed_held_recovery_needs_policy_but_no_new_marker():
    node,commands,_=gate(True);node.args.resume_held=True
    node.inputs['observer']=(False,0.)
    assert not TrialGate.step(node)
    assert node.started and commands==[True]


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


def test_startup_timeout_names_the_selected_marker():
    node,commands,events=gate(True)
    node.args.selected_id=1;node.inputs['observer']=(False,time.monotonic());node.requested-=46
    assert TrialGate.step(node)
    assert 'selected marker ID 1' in node.fault
    assert not node.started and not commands
