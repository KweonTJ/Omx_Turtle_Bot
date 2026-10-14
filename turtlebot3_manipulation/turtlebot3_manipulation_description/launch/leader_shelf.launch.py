# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Display the leader and two open bins using isolated, synthetic joint states."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Never start controllers or publish on the real robot TF/joint topics."""
    package = FindPackageShare('turtlebot3_manipulation_description')
    script = PathJoinSubstitution([package, 'scripts', 'export_shelf_scene.py'])
    command = ['/usr/bin/python3 ', script, ' --with-leader']
    defaults = {'stack_pitch': '0.156', 'shelf_x': '0.450',
                'shelf_y': '0.0', 'shelf_z': '0.0', 'shelf_yaw': '0.0'}
    arguments = []
    for key, value in defaults.items():
        arguments.append(DeclareLaunchArgument(key, default_value=value))
        command.extend([' --'+key.replace('_', '-')+' ', LaunchConfiguration(key)])
    description = ParameterValue(Command(command), value_type=str)
    tf_remaps = [('/tf', '/shelf_preview/tf'),
                 ('/tf_static', '/shelf_preview/tf_static')]
    state_remaps = [('joint_states', '/shelf_preview/joint_states'),
                    ('robot_description', '/shelf_preview/robot_description')]
    return LaunchDescription(arguments + [
        DeclareLaunchArgument('start_rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='shelf_preview_state_publisher',
             parameters=[{'robot_description': description}],
             remappings=tf_remaps+state_remaps),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='shelf_preview_joints',
             parameters=[{'robot_description': description,
                          'zeros': {'joint2': -1.15, 'joint3': .60,
                                    'joint4': .55, 'gripper_left_joint': .010}}],
             remappings=state_remaps),
        Node(package='rviz2', executable='rviz2', name='shelf_preview_rviz',
             arguments=['-d', PathJoinSubstitution([
                 package, 'rviz', 'leader_shelf.rviz'])], remappings=tf_remaps,
             condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
