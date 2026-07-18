"""Static package and ROS interface contracts; no DDS or hardware is used."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'turtlebot3_position'
CONTROLLER = MODULE / 'position_controller_node.py'
SERIAL_NODE = MODULE / 'uwb_serial_node.py'
CONSOLE = MODULE / 'goal_console.py'
LAUNCH = ROOT / 'launch' / 'position.launch.py'
CONFIG = ROOT / 'config' / 'position.yaml'


def _source(path):
    return path.read_text(encoding='utf-8')


def _tree(path):
    return ast.parse(_source(path), filename=str(path))


def _method_calls(path, method_name):
    calls = []
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if isinstance(function, ast.Attribute) and function.attr == method_name:
            calls.append(node)
    return calls


def _name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


def _literal_strings(path):
    return {
        node.value
        for node in ast.walk(_tree(path))
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def _walk_scalars(value):
    if isinstance(value, dict):
        for item in value.values():
            yield from _walk_scalars(item)
    elif isinstance(value, list):
        for item in value:
            yield from _walk_scalars(item)
    else:
        yield value


def _config():
    return yaml.safe_load(_source(CONFIG))


def _assert_mapping_contains(mapping, expected):
    for key, value in expected.items():
        assert mapping[key] == value


def test_required_ament_python_package_structure_exists():
    expected = {
        ROOT / 'package.xml',
        ROOT / 'setup.py',
        ROOT / 'setup.cfg',
        ROOT / 'README.md',
        ROOT / 'resource' / 'turtlebot3_position',
        CONFIG,
        LAUNCH,
        MODULE / '__init__.py',
        MODULE / 'core.py',
        SERIAL_NODE,
        CONTROLLER,
        CONSOLE,
    }
    missing = sorted(str(path.relative_to(ROOT)) for path in expected if not path.is_file())
    assert not missing, f'missing package files: {missing}'


def test_package_xml_declares_build_type_and_runtime_dependencies():
    package = ET.parse(ROOT / 'package.xml').getroot()
    assert package.findtext('name') == 'turtlebot3_position'
    assert package.findtext('version')
    assert package.findtext('license')
    assert {item.text for item in package.findall('buildtool_depend')} >= {
        'ament_python'
    }
    assert package.findtext('./export/build_type') == 'ament_python'

    dependencies = {
        item.text
        for tag in ('depend', 'exec_depend')
        for item in package.findall(tag)
    }
    assert dependencies >= {
        'ament_index_python',
        'geometry_msgs',
        'launch',
        'launch_ros',
        'nav_msgs',
        'python3-serial',
        'rclpy',
        'std_msgs',
    }
    test_dependencies = {item.text for item in package.findall('test_depend')}
    assert test_dependencies >= {'python3-pytest', 'python3-yaml'}


def test_python_imports_have_matching_ros_dependencies():
    package = ET.parse(ROOT / 'package.xml').getroot()
    dependencies = {
        item.text
        for tag in ('depend', 'exec_depend')
        for item in package.findall(tag)
    }
    import_to_dependency = {
        'ament_index_python': 'ament_index_python',
        'geometry_msgs': 'geometry_msgs',
        'launch': 'launch',
        'launch_ros': 'launch_ros',
        'nav_msgs': 'nav_msgs',
        'rclpy': 'rclpy',
        'serial': 'python3-serial',
        'std_msgs': 'std_msgs',
        'yaml': 'python3-yaml',
    }
    imported = set()
    for path in [*MODULE.glob('*.py'), LAUNCH]:
        for node in ast.walk(_tree(path)):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split('.')[0])
    for imported_name, dependency in import_to_dependency.items():
        if imported_name in imported:
            assert dependency in dependencies


def test_setup_installs_resource_package_config_and_launch():
    source = _source(ROOT / 'setup.py')
    compact_source = ' '.join(source.split())
    ast.parse(source)
    assert "package_name = 'turtlebot3_position'" in source
    assert 'share/ament_index/resource_index/packages' in source
    assert "['resource/' + package_name]" in source
    assert "('share/' + package_name, ['package.xml'])" in source
    assert "glob('config/*.yaml')" in source
    assert "glob('launch/*.launch.py')" in source
    assert "os.path.join('share', package_name, 'config')" in compact_source
    assert "os.path.join('share', package_name, 'launch')" in compact_source

    marker = ROOT / 'resource' / 'turtlebot3_position'
    assert marker.is_file()


def test_setup_cfg_installs_scripts_in_package_lib_directory():
    source = _source(ROOT / 'setup.cfg')
    assert 'script_dir=$base/lib/turtlebot3_position' in source
    assert 'install_scripts=$base/lib/turtlebot3_position' in source


def test_console_scripts_point_to_existing_main_functions():
    setup_tree = _tree(ROOT / 'setup.py')
    setup_calls = [
        node for node in ast.walk(setup_tree)
        if isinstance(node, ast.Call) and _name(node.func) == 'setup'
    ]
    assert len(setup_calls) == 1
    entry_keyword = next(
        keyword for keyword in setup_calls[0].keywords
        if keyword.arg == 'entry_points'
    )
    entry_points = ast.literal_eval(entry_keyword.value)['console_scripts']
    expected = {
        'uwb_serial_node': SERIAL_NODE,
        'position_controller_node': CONTROLLER,
        'goal_console': CONSOLE,
    }
    parsed = {}
    for entry in entry_points:
        executable, target = (part.strip() for part in entry.split('=', 1))
        module_name, function_name = target.split(':', 1)
        parsed[executable] = (module_name, function_name)

    assert set(parsed) == set(expected)
    for executable, path in expected.items():
        module_name, function_name = parsed[executable]
        assert module_name == f'turtlebot3_position.{path.stem}'
        functions = {
            node.name for node in _tree(path).body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        assert function_name == 'main'
        assert function_name in functions


def test_launch_starts_only_serial_and_controller_with_position_yaml():
    tree = _tree(LAUNCH)
    node_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and _name(node.func) == 'Node'
    ]
    assert len(node_calls) == 2

    launch_nodes = {}
    for call in node_calls:
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        package = ast.literal_eval(keywords['package'])
        executable = ast.literal_eval(keywords['executable'])
        name = ast.literal_eval(keywords['name'])
        assert package == 'turtlebot3_position'
        assert name == executable
        assert 'parameters' in keywords
        assert 'remappings' not in keywords
        launch_nodes[executable] = keywords

    assert set(launch_nodes) == {'uwb_serial_node', 'position_controller_node'}
    source = _source(LAUNCH)
    assert "get_package_share_directory('turtlebot3_position')" in source
    strings = _literal_strings(LAUNCH)
    assert {'config', 'position.yaml'} <= strings
    assert source.count('parameters=[config]') == 2
    assert "executable='goal_console'" not in source


def test_yaml_contains_all_topic_contracts_and_major_parameters():
    config = _config()
    serial = config['uwb_serial_node']['ros__parameters']
    controller = config['position_controller_node']['ros__parameters']
    console = config['goal_console']['ros__parameters']

    _assert_mapping_contains(serial, {
        'port': '/dev/ttyUSB0',
        'baud': 115200,
        'reconnect_sec': 2.0,
        'frame_id': 'uwb_map',
        'pose_topic': '/turtlebot3_position/pose',
        'uwb_valid_topic': '/turtlebot3_position/uwb/valid',
        'uwb_raw_topic': '/turtlebot3_position/uwb/raw',
        'odom_topic': '/odom',
        'include_odom_yaw': True,
        'odom_timeout_sec': 1.0,
        'position_variance_x': 0.0121,
        'position_variance_y': 0.0121,
        'yaw_variance': 0.04,
        'yaw_unavailable_variance': 9.8696,
    })
    _assert_mapping_contains(controller, {
        'frame_id': 'uwb_map',
        'pose_topic': '/turtlebot3_position/pose',
        'odom_topic': '/odom',
        'goal_topic': '/turtlebot3_position/goal',
        'enable_topic': '/turtlebot3_position/enable',
        'safety_stop_topic': '/safety_stop',
        'status_topic': '/turtlebot3_position/status',
        'nav_cmd_vel_topic': '/turtlebot3_control/nav_cmd_vel',
        'base_arrived_topic': '/turtlebot3_control/base_arrived',
        'delivery_request_topic': '/delivery/request',
        'delivery_event_topic': '/delivery/event',
        'arrival_tolerance': 0.13,
        'arrival_confirmations': 3,
        'linear_speed': 0.055,
        'angular_speed': 0.28,
        'heading_tolerance': 0.30,
        'final_yaw_tolerance': 0.20,
        'drive_pulse_sec': 0.45,
        'turn_pulse_sec': 0.35,
        'settle_sec': 1.6,
        'uwb_timeout_sec': 2.0,
        'odom_timeout_sec': 1.0,
        'pickup_wait_sec': 2.0,
        'delivery_wait_sec': 2.0,
        'initial_yaw': 0.0,
        'use_odom_yaw': True,
        'use_goal_yaw': False,
    })
    _assert_mapping_contains(console, {
        'goal_topic': '/turtlebot3_position/goal',
        'enable_topic': '/turtlebot3_position/enable',
        'frame_id': 'uwb_map',
        'auto_enable': True,
    })


def test_controller_and_console_share_the_same_waypoint_values():
    config = _config()
    controller = config['position_controller_node']['ros__parameters']
    console = config['goal_console']['ros__parameters']
    expected = {
        'pickup_x': 0.80,
        'pickup_y': 0.93,
        'tower1_x': 0.20,
        'tower1_y': 0.20,
        'tower2_x': 1.55,
        'tower2_y': 0.93,
        'tower3_x': 1.55,
        'tower3_y': 0.90,
        'safe_x': 0.80,
        'safe_y': 0.20,
    }
    for name, value in expected.items():
        assert controller[name] == value
        assert console[name] == value


def test_python_defaults_do_not_duplicate_waypoint_coordinates():
    waypoint_parameters = {
        f'{waypoint}_{axis}'
        for waypoint in ('pickup', 'tower1', 'tower2', 'tower3', 'safe')
        for axis in ('x', 'y')
    }
    offenders = []
    for path in MODULE.glob('*.py'):
        for node in ast.walk(_tree(path)):
            if not isinstance(node, ast.Dict):
                continue
            for key in node.keys:
                if (
                    isinstance(key, ast.Constant)
                    and key.value in waypoint_parameters
                ):
                    offenders.append(f'{path.name}:{key.value}')
    assert not offenders, (
        'waypoint coordinate values must come from position.yaml parameters: '
        f'{offenders}')


def test_serial_node_publishes_pose_with_covariance_and_diagnostics():
    publishers = _method_calls(SERIAL_NODE, 'create_publisher')
    publisher_types = {_name(call.args[0]) for call in publishers if call.args}
    assert {'PoseWithCovarianceStamped', 'Bool', 'String'} <= publisher_types
    subscriber_types = {
        _name(call.args[0])
        for call in _method_calls(SERIAL_NODE, 'create_subscription')
        if call.args
    }
    assert 'Odometry' in subscriber_types

    strings = _literal_strings(SERIAL_NODE)
    assert '/turtlebot3_position/pose' in strings
    assert '/turtlebot3_position/uwb/valid' in strings
    assert '/turtlebot3_position/uwb/raw' in strings
    assert '/odom' in strings


def test_serial_node_reconnect_decode_covariance_and_shutdown_contracts():
    source = _source(SERIAL_NODE)
    compact_source = ' '.join(source.split())
    assert 'serial.Serial(' in source
    assert "decode('utf-8', errors='replace')" in source
    assert '_stop_event.wait(self._reconnect_sec)' in source
    assert 'device.close()' in source
    assert '_thread.join(timeout=2.0)' in source
    for index in (0, 7, 35):
        assert f'covariance[{index}]' in source
    assert 'msg.pose.covariance = covariance' in compact_source


def test_controller_message_types_topics_and_arrived_timer():
    publishers = _method_calls(CONTROLLER, 'create_publisher')
    publisher_types = {_name(call.args[0]) for call in publishers if call.args}
    assert {'Twist', 'Bool', 'String'} <= publisher_types

    subscriptions = _method_calls(CONTROLLER, 'create_subscription')
    subscription_types = {_name(call.args[0]) for call in subscriptions if call.args}
    assert {
        'PoseWithCovarianceStamped', 'PoseStamped', 'Odometry', 'Bool', 'String'
    } <= subscription_types

    strings = _literal_strings(CONTROLLER)
    assert '/turtlebot3_control/nav_cmd_vel' in strings
    assert '/turtlebot3_control/base_arrived' in strings
    assert '/turtlebot3_position/goal' in strings
    assert '/turtlebot3_position/enable' in strings
    assert '/delivery/request' in strings
    assert '/delivery/event' in strings

    arrived_timers = []
    for call in _method_calls(CONTROLLER, 'create_timer'):
        if len(call.args) < 2:
            continue
        try:
            period = ast.literal_eval(call.args[0])
        except (ValueError, TypeError):
            continue
        callback_name = _name(call.args[1]) or ''
        if math_isclose(period, 0.1) and 'arrived' in callback_name:
            arrived_timers.append(call)
    assert arrived_timers, 'base_arrived must be published by a 0.1 second timer'


def math_isclose(left, right):
    return abs(float(left) - float(right)) <= 1.0e-12


def test_goal_console_publishes_pose_goal_and_enable_only():
    publishers = _method_calls(CONSOLE, 'create_publisher')
    publisher_types = {_name(call.args[0]) for call in publishers if call.args}
    assert publisher_types == {'PoseStamped', 'Bool'}
    strings = _literal_strings(CONSOLE)
    # Exact defaults live only in the shared YAML; the console reads parameters.
    assert {'goal_topic', 'enable_topic', 'position.yaml'} <= strings
    source = _source(CONSOLE)
    assert 'Bool(data=True)' in source
    assert 'Bool(data=False)' in source
    assert '/delivery/request' not in strings
    assert '/delivery/event' not in strings


def test_manual_override_and_disable_reset_controller_motion_state():
    source = _source(CONTROLLER)
    tree = _tree(CONTROLLER)
    classes = {
        node.name: node for node in tree.body if isinstance(node, ast.ClassDef)
    }
    controller = classes['PositionControllerNode']
    methods = {
        node.name: ast.get_source_segment(source, node) or ''
        for node in controller.body if isinstance(node, ast.FunctionDef)
    }

    on_goal = methods['_on_goal']
    assert 'self._stop()' in on_goal
    assert 'self.interlock.new_goal()' in on_goal
    assert 'self._publish_arrived()' in on_goal
    assert 'self.mode.start_manual()' in on_goal
    assert 'self._reset_motion_state(clear_wait=True)' in on_goal
    assert 'self._set_goal(target, goal_yaw)' in on_goal
    assert 'manual_goal_rejected' in on_goal

    reset = methods['_reset_motion_state']
    for assignment in (
        'self.motion_until = None',
        'self.settle_until = None',
        'self.wait_until = None',
        'self.arrival_count = 0',
    ):
        assert assignment in reset

    on_enable = methods['_on_enable']
    assert 'self.mode.cancel(' in on_enable
    assert 'self._clear_goal_and_timers()' in on_enable
    assert 'self._stop()' in on_enable


def test_safety_callbacks_stop_and_require_fresh_enable():
    source = _source(CONTROLLER)
    tree = _tree(CONTROLLER)
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'PositionControllerNode'
    )
    methods = {
        node.name: ast.get_source_segment(source, node) or ''
        for node in controller.body if isinstance(node, ast.FunctionDef)
    }
    on_safety = methods['_on_safety']
    assert 'self.mode.enter_estop()' in on_safety
    assert 'self.interlock.set_safety_stop(True)' in on_safety
    assert 'self.interlock.set_safety_stop(False)' in on_safety
    assert 'if not self.interlock.safety_stop' in on_safety
    assert 'SAFETY_CLEARED_REENABLE_REQUIRED' in on_safety
    assert 'self._stop()' in on_safety
    assert 'self._publish_arrived()' in on_safety

    on_enable = methods['_on_enable']
    assert 'self.mode.resume_after_estop()' in on_enable


def test_no_direct_cmd_vel_publisher_or_remapping_anywhere():
    checked_python = [*MODULE.glob('*.py'), LAUNCH]
    for path in checked_python:
        assert '/cmd_vel' not in _literal_strings(path), path

    config = _config()
    assert '/cmd_vel' not in set(_walk_scalars(config))

    launch_tree = _tree(LAUNCH)
    remapping_keywords = [
        keyword
        for node in ast.walk(launch_tree)
        if isinstance(node, ast.Call)
        for keyword in node.keywords
        if keyword.arg == 'remappings'
    ]
    assert not remapping_keywords


def test_status_prefixes_are_present_in_controller():
    source = _source(CONTROLLER)
    for prefix in (
        'WAIT_SENSOR', 'IDLE', 'ROTATE_TO_GOAL', 'DRIVE', 'FINAL_ALIGN',
        'ARRIVED', 'FAULT', 'DISABLED', 'SAFETY_STOP',
    ):
        assert prefix in source
