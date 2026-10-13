"""Host-only, read-only display. Never starts the actual robot controller."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true/false')
        return text == 'true'

    description = Path(get_package_share_directory('turtlebot3_manipulation_description'))
    demo = boolean('demo')
    bridge = boolean('start_bridge')
    if demo and bridge:
        raise ValueError('Demo must not be connected to the real telemetry bridge')
    prefix = '/leader_monitor/demo' if demo else '/leader_monitor/input'
    request = prefix + '/host_request' if demo else '/leader_monitor/host_request'
    host_domain = 223 if demo else int(value('host_domain'))
    if not 0 <= host_domain <= 232:
        raise ValueError('host_domain must be in 0..232')
    if bridge:
        if not value('source_domain'):
            raise ValueError('source_domain is required when starting the bridge')
        source_domain = int(value('source_domain'))
        if not 0 <= source_domain <= 232 or source_domain == host_domain:
            raise ValueError('source_domain must be 0..232 and different from host_domain')
    env = {'ROS_DOMAIN_ID': str(host_domain)}
    if demo:
        env['ROS_LOCALHOST_ONLY'] = '1'
    transport = value('image_transport')
    if transport not in ('raw', 'compressed'):
        raise ValueError('image_transport must be raw/compressed')
    robot_description = ParameterValue(Command([
        'python3 ', str(description / 'scripts/export_leader_urdf.py'), ' --stdout'
    ]), value_type=str)
    viewer = Node(
        package='omx_leader_monitor', executable='leader_monitor', output='screen',
        parameters=[value('config'), {
            'input_prefix': prefix, 'host_request_topic': request,
            'image_transport': transport, 'demo_mode': demo,
            'fullscreen': boolean('fullscreen'), 'enable_3d': boolean('enable_3d'),
            'diagnostics_path': value('diagnostics_path'),
            'auto_close_s': float(value('auto_close_s')),
        }],
        remappings=[('/tf', '/leader_monitor/visualization/tf'),
                    ('/tf_static', '/leader_monitor/visualization/tf_static')],
        additional_env=env,
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='leader_monitor_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', prefix + '/joint_states'),
                    ('robot_description', '/leader_monitor/visualization/robot_description'),
                    ('/tf', '/leader_monitor/visualization/tf'),
                    ('/tf_static', '/leader_monitor/visualization/tf_static')],
        additional_env=env,
    )
    nodes = [rsp, viewer]
    if demo:
        nodes.append(Node(
            package='omx_leader_monitor', executable='demo_telemetry.py', output='screen',
            arguments=['--transport', transport], additional_env=env,
        ))
    if bridge:
        nodes.append(Node(
            package='omx_leader_monitor', executable='topic_bridge.py', output='screen',
            arguments=['--config', value('bridge_config'), '--source-domain', value('source_domain'),
                       '--host-domain', str(host_domain), '--transport', transport],
        ))
    # Closing the display ends its own RSP, bridge and demo; never touches another launch.
    nodes.append(RegisterEventHandler(OnProcessExit(
        target_action=viewer, on_exit=[EmitEvent(event=Shutdown(reason='Host monitor closed'))])))
    return nodes


def generate_launch_description():
    package = Path(get_package_share_directory('omx_leader_monitor'))
    args = {
        'config': str(package / 'config/monitor.yaml'),
        'bridge_config': str(package / 'config/bridge_topics.yaml'),
        'demo': 'false', 'start_bridge': 'false', 'source_domain': '', 'host_domain': '16',
        'image_transport': 'raw', 'fullscreen': 'false', 'enable_3d': 'true',
        'diagnostics_path': '', 'auto_close_s': '0.0',
    }
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in args.items()], OpaqueFunction(function=setup)])
