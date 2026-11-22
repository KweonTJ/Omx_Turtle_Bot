"""One entry point for calibrated RGB input and the v6 C++ ONNX/OpenCR runtime."""
from functools import partial
from pathlib import Path
import time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable
from omx_turtlebot3_runtime.launch_support import build_actions


def generate_launch_description():
    root = Path(get_package_share_directory('omx_turtlebot3_runtime'))
    policy = Path(get_package_share_directory('omx_turtlebot3_policy'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='read_only', choices=['read_only', 'execute']),
        DeclareLaunchArgument('config', default_value=str(root/'config/runtime.yaml')),
        DeclareLaunchArgument('scene', default_value=str(root/'assets/scene.xml')),
        DeclareLaunchArgument('model_dir', default_value=str(policy/'models/omx_turtlebot3_v6')),
        DeclareLaunchArgument('python_executable', default_value=''),
        DeclareLaunchArgument('start_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('video_device', default_value=(
            '/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0')),
        DeclareLaunchArgument('camera_info_url', default_value='file://'+str(root/'config/eef_usb_camera.yaml')),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='25')),
        DeclareLaunchArgument('duration_s', default_value='0'),
        DeclareLaunchArgument('log_dir', default_value='/tmp/omx_v6_cpp_launch_'+str(time.time_ns())),
        OpaqueFunction(function=partial(build_actions, runtime_share=root,
            policy_share=policy, runtime_prefix=get_package_prefix('omx_turtlebot3_runtime'))),
    ])
