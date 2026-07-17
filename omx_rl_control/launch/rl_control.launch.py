"""Launch the arm-only PPO runtime, optionally with robot bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the standalone RL control launch description."""
    config_file = LaunchConfiguration('config_file')
    start_hardware = LaunchConfiguration('start_hardware')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_rviz = LaunchConfiguration('start_rviz')

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_manipulation_bringup'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'start_rviz': start_rviz,
            'start_camera': 'false',
            'start_eef_camera_driver': 'false',
            'start_state_relays': 'false',
            'start_lidar': 'false',
            'move_to_stay_pose': 'false',
        }.items(),
        condition=IfCondition(start_hardware),
    )

    runtime = Node(
        package='omx_rl_control',
        executable='rl_control_node',
        name='rl_control_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_rl_control'),
                'config',
                'rl_control.yaml',
            ]),
        ),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        hardware,
        runtime,
    ])
