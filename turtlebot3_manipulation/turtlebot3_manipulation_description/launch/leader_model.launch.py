"""Isolated leader model preview. No controller, camera, or hardware driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('turtlebot3_manipulation_description')
    description = ParameterValue(Command([
        'python3 ', PathJoinSubstitution([package,'scripts','export_leader_urdf.py']), ' --stdout'
    ]),value_type=str)
    use_gui = LaunchConfiguration('use_gui')
    tf_remaps = [('/tf','/leader_preview/tf'),('/tf_static','/leader_preview/tf_static')]
    state_remaps = [('joint_states','/leader_preview/joint_states'),
                    ('robot_description','/leader_preview/robot_description')]
    parameters = [{'robot_description':description,
                   'zeros':{'joint2':-1.15,'joint3':.60,'joint4':.55,'gripper_left_joint':.010}}]
    return LaunchDescription([
        DeclareLaunchArgument('use_gui',default_value='true'),
        DeclareLaunchArgument('start_rviz',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             name='leader_preview_state_publisher',parameters=[{'robot_description':description}],
             remappings=tf_remaps+state_remaps),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',
             name='leader_preview_joints_gui',parameters=parameters,remappings=state_remaps,
             condition=IfCondition(use_gui)),
        Node(package='joint_state_publisher',executable='joint_state_publisher',
             name='leader_preview_joints',parameters=parameters,remappings=state_remaps,
             condition=UnlessCondition(use_gui)),
        Node(package='rviz2',executable='rviz2',name='leader_preview_rviz',
             arguments=['-d',PathJoinSubstitution([package,'rviz','leader_model.rviz'])],
             remappings=tf_remaps,condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
