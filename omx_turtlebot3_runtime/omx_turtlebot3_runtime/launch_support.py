"""Build one native-control launch without importing the numerical Python stack."""
import math
import os
from pathlib import Path
import sys
import yaml
from .commissioning import commissioning_issues


def runtime_python(prefix, explicit=''):
    if explicit:
        candidates = [Path(explicit).expanduser()]
    elif os.environ.get('OMX_CPP_PYTHON'):
        candidates = [Path(os.environ['OMX_CPP_PYTHON']).expanduser()]
    else:
        prefix = Path(prefix)
        candidates = [base/name/'bin/python' for base in (prefix.parent, prefix.parent.parent)
                      for name in ('.venv_omx_cpp', '.venv_omx_runtime')]
        candidates.append(Path(sys.executable))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            # Keep the venv launcher path. Resolving its symlink selects system Python.
            return str(candidate.absolute())
    raise ValueError('No executable Python environment; set python_executable or OMX_CPP_PYTHON')


def calibrated_camera(path):
    value = yaml.safe_load(Path(path).read_text())
    if (value.get('image_width'), value.get('image_height')) != (320, 240):
        raise ValueError('Camera calibration must match 320x240')
    k = value.get('camera_matrix', {})
    d = value.get('distortion_coefficients', {})
    matrix, distortion = k.get('data', []), d.get('data', [])
    if (k.get('rows'), k.get('cols')) != (3, 3) or len(matrix) != 9:
        raise ValueError('Invalid camera matrix')
    numbers = matrix+distortion
    if (any(type(x) not in (float, int) or not math.isfinite(x) for x in numbers)
            or matrix[0] <= 0 or matrix[4] <= 0 or matrix[6:] != [0, 0, 1]
            or len(distortion) not in (4, 5, 8, 12, 14)
            or value.get('distortion_model') not in ('plumb_bob', 'rational_polynomial')):
        raise ValueError('Invalid camera intrinsics/distortion')
    return value


def build_actions(context, *, runtime_share, policy_share, runtime_prefix):
    from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_prefix

    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('mode')
    if mode not in ('read_only', 'execute'):
        raise ValueError('mode must be read_only or execute')
    config_path = Path(value('config')).expanduser().resolve(strict=True)
    config = yaml.safe_load(config_path.read_text())
    issues = commissioning_issues(config)
    if mode == 'execute' and issues:
        raise ValueError('Execution blocked before hardware startup: '+', '.join(issues))
    python = runtime_python(runtime_prefix, value('python_executable'))
    scene = Path(value('scene')).expanduser().resolve(strict=True)
    model = Path(value('model_dir')).expanduser().resolve(strict=True)
    seconds = float(value('duration_s'))
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError('duration_s must be finite and nonnegative')
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('Invalid ROS domain ID')
    logs = Path(value('log_dir')).expanduser().absolute()
    if logs.exists():
        raise ValueError('Use a new log_dir; existing run logs are never overwritten')
    common_env = {'PYTHONNOUSERSITE': '1', 'ROS_LOG_DIR': str(logs/'ros'),
                  'ROS_DOMAIN_ID': str(domain)}
    camera = None
    start_camera = value('start_camera')
    if start_camera not in ('true', 'false'):
        raise ValueError('start_camera must be true or false')
    if start_camera == 'true':
        url = value('camera_info_url')
        if not url.startswith('file://'):
            raise ValueError('camera_info_url must be file:///...yaml')
        calibrated_camera(url[7:])
        try:
            get_package_prefix('v4l2_camera')
        except LookupError as exc:
            raise ValueError('v4l2_camera is not installed; install it or use start_camera:=false with an existing camera publisher') from exc
        camera = Node(package='v4l2_camera', executable='v4l2_camera_node',
            name='omx_v6_eef_camera', output='screen', additional_env=common_env,
            parameters=[{
                'video_device': value('video_device'),
                'camera_frame_id': config['vision']['optical_frame'],
                'camera_name': 'eef_usb_camera', 'camera_info_url': url,
                'pixel_format': 'YUYV', 'output_encoding': 'rgb8', 'image_size': [320, 240],
            }], remappings=[('image_raw', config['vision']['image_topic']),
                            ('camera_info', config['vision']['camera_info_topic'])])
    command = [python, '-m', 'omx_turtlebot3_runtime.node', '--backend', 'cpp_onnx',
               '--config', str(config_path), '--scene', str(scene), '--model-dir', str(model),
               '--duration', str(seconds), '--log-dir', str(logs/'runtime')]
    if mode == 'execute':
        command.extend(['--execute', '--exit-after-mission'])
    startup = ExecuteProcess(cmd=command+['--check-startup'], name='omx_cpp_startup_check',
                             output='screen', additional_env=common_env)
    runtime = ExecuteProcess(cmd=command, name='omx_cpp_runtime', output='screen',
                             additional_env=common_env)

    def startup_done(event, _context):
        if event.returncode != 0:
            raise RuntimeError('C++ startup check failed; camera/controller were not started')
        return [LogInfo(msg='C++ ONNX startup check passed; starting '+mode+' runtime. '
                       'Execute waits for deadman, then measured Stay arrival before accepting mission requests.'),
                *([camera] if camera is not None else []), runtime]

    def runtime_done(event, context):
        if event.returncode != 0 and not context.is_shutdown:
            raise RuntimeError('OMX C++ runtime failed with code '+str(event.returncode))
        return [EmitEvent(event=Shutdown(reason='OMX C++ runtime exited'))]

    handlers = [RegisterEventHandler(OnProcessExit(target_action=startup, on_exit=startup_done)),
                RegisterEventHandler(OnProcessExit(target_action=runtime, on_exit=runtime_done))]
    if camera is not None:
        def camera_done(event, context):
            if not context.is_shutdown:
                raise RuntimeError('EEF camera exited unexpectedly with code '+str(event.returncode))
            return None
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=camera_done)))
    return [LogInfo(msg='OMX v6 mode='+mode+'; commissioning issues='+str(issues)),
            *handlers, startup]
