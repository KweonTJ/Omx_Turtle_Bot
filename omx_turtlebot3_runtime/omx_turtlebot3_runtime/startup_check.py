"""Load the native policy and collision geometry without opening a hardware device."""
import hashlib
import json
import numpy as np
from .commissioning import commissioning_issues
from .core import Fault
from .cpp_backend import NativeActor
from .planning import Planner


def check(args, config, training):
    if args.backend != 'cpp_onnx':
        raise Fault('--check-startup requires the cpp_onnx backend')
    issues = commissioning_issues(config)
    if args.execute and issues:
        raise Fault('Execute blocked: '+', '.join(issues))
    robot = args.scene.parent/config['vision']['robot_urdf_asset']
    actor = NativeActor(args.model_dir, args.config, robot)
    try:
        planner = Planner(args.scene, training, config)
        action = actor(np.zeros(69))
        if action.shape != (4,) or not np.isfinite(action).all():
            raise Fault('Invalid native ONNX output')
        return dict(event='STARTUP_CHECK_PASSED', backend=actor.backend,
            motor_transport='cpp_opencr', model_version='omx_turtlebot3_v6',
            observation_size=69, action_size=4, execution_config_ready=not issues,
            startup_parity_vectors=973, onnx_output_route='original_l3_then_native_clip_-1_1',
            commissioning_issues=issues, physical_robot_checked=False,
            camera_input_checked=False, serial_opened=False, motor_writes=0,
            startup_sequence=['WAIT_DEADMAN', 'PATH_PREFLIGHT', 'MOVING_STAY', 'READY', 'MISSION'],
            stay_arrival_checked=False,
            config_path=str(args.config.resolve()),
            config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
            actor_sha256=hashlib.sha256((args.model_dir/'actor.onnx').read_bytes()).hexdigest(),
            robot_urdf_sha256=planner.robot_urdf.sha256,
            stay_q=planner.stay.tolist(), inference_only_action=action.tolist())
    finally:
        actor.close()


def print_report(report):
    print(json.dumps(report, allow_nan=False), flush=True)
