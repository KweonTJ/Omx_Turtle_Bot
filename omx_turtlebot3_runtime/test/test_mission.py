from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.core import Fault, Phase, RobotSample, PoseCache, ObjectPose
from omx_turtlebot3_runtime.mission import Mission


def instance():
    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root/'config/runtime.yaml').read_text())
    training = yaml.safe_load((root/'config/training_contract.yaml').read_text())
    planner = SimpleNamespace(stay=np.asarray(training['stay']))
    actor = lambda x: np.zeros(4)
    m = Mission(cfg, training, planner, actor)
    m.active, m.last_time, m.start_time, m.phase_time = True, 10., 10., 10.
    m.target = planner.stay.copy()
    m.selected, m.rotation_complete = 0, False
    sample = RobotSample(10.05, planner.stay.copy(), np.zeros(4), 0., 0., 0., 12., (0, 0), 10050)
    return m, sample


def test_no_descent_before_rotation_gate():
    m, sample = instance()
    m.phase = Phase.EXTEND_LOWER
    with pytest.raises(Fault, match='Descent forbidden'):
        m.step(sample, PoseCache(), 10.05)


def test_no_approach_with_stale_vision():
    m, sample = instance()
    m.phase = Phase.REACH
    with pytest.raises(Fault, match='vision expired'):
        m.step(sample, PoseCache(), 10.05)


def test_no_automatic_homing():
    m, sample = instance()
    m.active = False
    cache = PoseCache()
    cache.begin_frame(10.05)
    for i in (0, 1):
        cache.update(ObjectPose(i, 10.05, np.array([.22, .05*(-1)**i, .32]), np.array([0, 0, 0, 1]), .1), 10.05)
    sample.q[1] += .4
    with pytest.raises(Fault, match='automatic homing is prohibited'):
        m.start(0, sample, cache, 10.05)


def test_phase_timeout_and_deadline():
    m, sample = instance()
    with pytest.raises(Fault, match='deadline'):
        m.step(sample, PoseCache(), 11.)
    m.last_time = 100.
    sample.stamp = 100.05
    with pytest.raises(Fault, match='timeout'):
        m.step(sample, PoseCache(), 100.05)


def test_unknown_physical_id_never_falls_back():
    for index in (-1, 2, True, '0'):
        m, sample = instance()
        m.active = False
        with pytest.raises(Fault, match='physical marker'):
            m.start(index, sample, PoseCache(), 10.05)
