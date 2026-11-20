"""Reuse the existing tracker independently per ID; never launch motion.

This is camera-frame perception commissioning, not the v6 control runtime.
No old ArUco bridge, controller, hardware bringup or automatic Stay motion.
Private topics prevent a detection from triggering legacy pickup nodes.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    root = Path(get_package_share_directory('aruco_eef_tracker'))
    cfg = yaml.safe_load((root/'config/v6_marker_bindings.yaml').read_text())
    bindings = [(b['physical_marker_id'], b['policy_object_id'], b['policy_slot'])
                for b in cfg['bindings']]
    if (bindings != [(0, 1, 0), (1, 2, 1)]
            or cfg['dictionary'] != 'DICT_4X4_50'
            or cfg['marker_size_m'] != .04
            or cfg['motor_commands_enabled'] is not False):
        raise ValueError('Unexpected v6 marker binding; review the physical contract')
    result = []
    if value('start_camera') == 'true':
        url = value('camera_info_url')
        if not url.startswith('file://'):
            raise ValueError('Supply the calibrated camera_info_url=file:///...yaml')
        calibration = yaml.safe_load(Path(url[7:]).read_text())
        if (calibration['image_width'], calibration['image_height']) != (320, 240):
            raise ValueError('Camera calibration must match 320x240; no implicit resize')
        result.append(Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            namespace='omx_v6/eef_camera', name='camera', output='screen',
            parameters=[{
                'video_device': value('video_device'),
                'camera_frame_id': 'eef_usb_camera_optical_frame',
                'camera_name': 'eef_usb_camera', 'camera_info_url': url,
                'pixel_format': 'YUYV', 'output_encoding': 'rgb8',
                'image_size': [320, 240],
            }]))
    for physical, policy_object, slot in bindings:
        prefix = f'/omx_v6/perception/marker_{physical}'
        result.append(Node(
            package='aruco_eef_tracker', executable='aruco_eef_tracker_node',
            name=f'v6_marker_{physical}', output='screen',
            parameters=[{
                'image_topic': value('image_topic'),
                'camera_info_topic': value('camera_info_topic'),
                # allowed_marker_ids is what the installed selector actually uses.
                # A separate node per ID prevents selecting the larger other marker.
                'dictionary': cfg['dictionary'], 'marker_size_m': cfg['marker_size_m'],
                'marker_id': physical, 'allowed_marker_ids': [physical],
                'accept_any_marker': False,
                'pose_topic': prefix+'/pose', 'visible_topic': prefix+'/visible',
                'status_topic': prefix+'/status', 'debug_image_topic': prefix+'/debug_image',
                'min_marker_perimeter_px': 48., 'max_reprojection_error_px': 1.5,
                'pose_z_min_m': .04, 'pose_z_max_m': 1.2,
                'publish_debug_image': True, 'draw_axes': True,
            }]))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('video_device', default_value=(
            '/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0')),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/omx_v6/eef_camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/omx_v6/eef_camera/camera_info'),
        OpaqueFunction(function=nodes),
    ])
