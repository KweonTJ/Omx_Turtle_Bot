"""Launch the RL pickup scene in Gazebo Sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the Gazebo scene, controllers, and optional RL runtime."""
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    start_rviz = LaunchConfiguration('start_rviz')
    start_rl_control = LaunchConfiguration('start_rl_control')
    config_file = LaunchConfiguration('config_file')
    residual_action_scale_override = LaunchConfiguration(
        'residual_action_scale_override')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_manipulation_gazebo'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'world': world,
            'gz_args': gz_args,
            'start_rviz': start_rviz,
            'start_depth_camera': 'true',
            'use_sim': 'true',
            'move_to_stay_pose': 'false',
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    runtime = Node(
        package='omx_rl_control',
        executable='rl_control_node',
        name='rl_control_node',
        output='screen',
        condition=IfCondition(start_rl_control),
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'odom_topic': '/diff_drive_controller/odom',
                'require_base_arrived': False,
                'joint_state_timeout_s': 1.0,
                'trajectory_time_s': 0.25,
                'release_settle_time_s': 0.75,
                'initialize_to_policy_stay': True,
                'residual_action_scale_override': ParameterValue(
                    residual_action_scale_override,
                    value_type=float,
                ),
                # 5.5 cm box with 4 mm compression:
                # (0.055 - 0.004) / 2 - 0.021 = 0.0045 m.
                'gripper_grasp_position': 0.0045,
                'gripper_max_effort': -1.0,
                # The grasped box center trails the EEF reference by 2.3 cm.
                'fallback_delivery_position': [0.293, 0.0, 0.1815],
            },
        ],
    )

    default_world = PathJoinSubstitution([
        FindPackageShare('omx_rl_control'),
        'worlds',
        'rl_pick_place.world',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r ', world],
            description='Gazebo Sim arguments; the default starts the GUI.',
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_rl_control', default_value='true'),
        DeclareLaunchArgument(
            'residual_action_scale_override',
            default_value='-1.0',
            description=(
                '-1 uses the policy contract; 0 runs reference-only A/B mode.'
            ),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_rl_control'),
                'config',
                'rl_control.yaml',
            ]),
        ),
        gazebo,
        runtime,
    ])
