# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Start perception and optionally a camera, never a robot controller."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def camera_nodes(context):
    """Avoid resolving optional camera driver when consuming an existing stream."""
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if value('start_camera').lower() != 'true':
        return []
    try:
        get_package_share_directory('v4l2_camera')
    except LookupError as error:
        raise RuntimeError(
            'v4l2_camera is missing. Install ros-humble-v4l2-camera on the '
            'camera computer, or use start_camera:=false with an existing stream.'
        ) from error
    return [Node(
        package='v4l2_camera', executable='v4l2_camera_node',
        namespace='eef_camera', name='v4l2_camera', output='screen',
        parameters=[{
            'video_device': value('video_device'),
            'camera_frame_id': value('camera_frame'),
            'camera_name': 'eef_usb_camera',
            'camera_info_url': value('camera_info_url'),
            'pixel_format': 'YUYV', 'output_encoding': 'rgb8',
            'image_size': [int(value('width')), int(value('height'))],
        }])]


def generate_launch_description():
    """Keep camera resolution and its matching calibration explicit."""
    arguments = [
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('omx_eef_vision'), 'config', 'shelf_perception.yaml'])),
        DeclareLaunchArgument('start_camera', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_frame',
                              default_value='eef_usb_camera_optical_frame'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    return LaunchDescription(arguments + [
        OpaqueFunction(function=camera_nodes),
        Node(package='omx_eef_vision', executable='multi_object_vision_node',
             name='multi_object_vision', output='screen',
             parameters=[LaunchConfiguration('config_file'), {
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
