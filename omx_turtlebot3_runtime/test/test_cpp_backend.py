"""Real native inference/control parity. Requires the built C++ library, no motors."""
import os
from pathlib import Path
import numpy as np
import pytest
import yaml
from omx_turtlebot3_runtime.core import Fault, Phase, SlewLimiter, residual_step
from omx_turtlebot3_runtime.cpp_backend import NativeActor

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent/'omx_turtlebot3_policy/models/omx_turtlebot3_v6'


@pytest.fixture(scope='module')
def native():
    # CI must build the package first; explicitly supplied missing paths fail.
    lib = os.environ.get('OMX_V6_CONTROL_TEST_LIBRARY')
    if not lib:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
        try:
            lib = str(Path(get_package_prefix('omx_turtlebot3_control_cpp'))/'lib/libomx_v6_control.so')
        except PackageNotFoundError:
            pytest.skip('Build omx_turtlebot3_control_cpp for native integration checks')
    actor = NativeActor(MODEL, ROOT/'config/runtime.yaml', ROOT/'assets/robot_real.urdf', lib)
    yield actor
    actor.close()


def test_original_973_vectors_use_actual_onnx(native):
    with np.load(MODEL/'parity_vectors.npz') as data:
        assert set(data.files) == {'observations', 'actions'}
        actual = np.array([native(o) for o in data['observations']])
        np.testing.assert_allclose(actual, data['actions'], rtol=0, atol=1e-5)
        assert len(actual) == 973


@pytest.mark.parametrize('phase', list(Phase))
def test_native_residual_velocity_and_gripper_match_reference(native, phase):
    cfg = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    rng = np.random.default_rng(642+int(phase))
    low = np.array([-3., -1.7, -.9, -1.7])
    high = np.array([3., 1.4, 1.3, 1.9])
    for waypoint in range(4):
        for dt in (.02, .05, .09):
            obs = rng.normal(size=69).astype(np.float32)
            target = rng.uniform(low+.2, high-.2)
            goal = np.clip(target+rng.normal(0, .1, 4), low, high)
            velocity = rng.uniform(-.08, .08, 4)
            if phase == Phase.ROTATE_RIGHT:
                # Mission resets and holds pitch-joint velocity on phase entry.
                velocity[1:] = 0
                goal[1:] = target[1:]
            action = native(obs)
            delta, expected_action = residual_step(target, goal, action, phase, waypoint, 4,
                                                    target, training, dt)
            limiter = SlewLimiter(cfg['control'])
            limiter.velocity = velocity.copy()
            expected = np.clip(target+limiter.step(delta, dt), low, high)
            actual, actual_action, actual_velocity, finger = native.control_step(
                obs, target, goal, target, velocity, phase, waypoint, 4, dt, 0, .019, low, high)
            np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
            np.testing.assert_allclose(actual_action, expected_action, rtol=0, atol=1e-12)
            np.testing.assert_allclose(actual_velocity, limiter.velocity, rtol=0, atol=1e-12)
            assert finger == pytest.approx(cfg['control']['gripper_velocity_m_s']*dt)


def test_native_rejects_stale_cycle_and_urdf_violation(native):
    q = np.zeros(4)
    with pytest.raises(Fault, match='deadline'):
        native.control_step(np.zeros(69), q, q, q, q, 0, 0, 1, .3, 0, 0, q-1, q+1)
    goal = np.array([0, 0, -2., 0])
    with pytest.raises(Fault, match='URDF'):
        native.control_step(np.zeros(69), q, goal, q, q, 0, 0, 1, .05, 0, 0, q-3, q+3)


def test_native_execute_rejected_before_serial_open(native):
    from omx_turtlebot3_runtime.cpp_backend import NativeTransport
    cfg = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    with pytest.raises(Fault, match='Commissioning incomplete'):
        NativeTransport(cfg, ROOT/'config/runtime.yaml', ROOT/'assets/robot_real.urdf',
                        execute=True, library_path=native.lib._name)


def test_mission_uses_native_control_and_still_checks_collision(native, monkeypatch):
    from types import SimpleNamespace
    from omx_turtlebot3_runtime.core import RobotSample, PoseCache, ObjectPose
    from omx_turtlebot3_runtime.mission import Mission
    cfg = yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    training = yaml.safe_load((ROOT/'config/training_contract.yaml').read_text())
    q = np.asarray(training['stay'])
    paths = []
    planner = SimpleNamespace(stay=q, low=q-.1, high=q+.1,
        update_objects=lambda *args: None, forward=lambda *args: np.array([.1, 0, .3]),
        path=lambda *args: paths.append(args))
    mission = Mission(cfg, training, planner, native)
    mission.active = True
    mission.phase, mission.waypoint = Phase.OPEN, 0
    mission.selected, mission.recognition_stage = 0, 0
    mission.target, mission.gripper_target = q.copy(), 0.
    mission.plans, mission.place = {Phase.OPEN: [q+.01]}, np.array([-.1, -.3, .07])
    mission.start_time = mission.phase_time = mission.last_time = mission.last_plan = 10.
    mission.stable_since = None
    cache = PoseCache()
    cache.begin_frame(10.05)
    for i in (0, 1):
        cache.update(ObjectPose(i, 10.05, np.array([.22, .05*(-1)**i, .32]),
                               np.array([0, 0, 0, 1]), .1), 10.05)
    mission.initial_objects = [p.position.copy() for p in cache.items]
    sample = RobotSample(10.05, q.copy(), np.zeros(4), 0., 0., 0., 12., (0, 0), 10050)
    def forbidden(*args, **kwargs):
        raise AssertionError('Python control path must not be used in C++ mode')
    monkeypatch.setattr('omx_turtlebot3_runtime.mission.residual_step', forbidden)
    monkeypatch.setattr(mission.limiter, 'step', forbidden)
    candidate, finger, obs = mission.step(sample, cache, 10.05)
    assert len(paths) == 1 and obs.shape == (69,)
    np.testing.assert_allclose(paths[0][1], candidate)
    assert finger > 0
    def collision(*args):
        raise Fault('Synthetic path collision')
    planner.path = collision
    sample.stamp = 10.10
    with pytest.raises(Fault, match='path collision'):
        mission.step(sample, cache, 10.10)
