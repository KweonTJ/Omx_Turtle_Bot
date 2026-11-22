"""Explicit empty-gripper, bounded ROS calibration with raw camera evidence."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from omx_turtlebot3_runtime.launch_support import runtime_python,calibrated_camera


def build(context):
    def value(key):return LaunchConfiguration(key).perform(context)
    output=Path(value('log_dir')).absolute()
    if output.exists():raise ValueError('Use a new calibration log directory')
    output.mkdir(parents=True)
    runtime=Path(get_package_share_directory('omx_turtlebot3_runtime'))
    camera_file=runtime/'config/eef_usb_camera.yaml';calibrated_camera(camera_file)
    env={'ROS_DOMAIN_ID':value('ros_domain_id'),'PYTHONNOUSERSITE':'1','ROS_LOG_DIR':str(output/'ros')}
    camera=Node(package='v4l2_camera',executable='v4l2_camera_node',namespace='eef_camera',
        output='screen',additional_env=env,parameters=[{
            'video_device':'/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0',
            'camera_frame_id':'eef_usb_camera_optical_frame','camera_name':'eef_usb_camera',
            'camera_info_url':'file://'+str(camera_file),'pixel_format':'YUYV','output_encoding':'rgb8','image_size':[320,240]}])
    python=runtime_python(get_package_prefix('omx_turtlebot3_runtime'),'')
    recorder=ExecuteProcess(cmd=[python,'-m','omx_turtlebot3_runtime.kinematic_capture',
        '--config',str(runtime/'config/runtime.yaml'),'--output',str(output/'capture'),
        '--reference-id',value('reference_id')],output='screen',additional_env=env)
    motion=Node(package='omx_turtlebot3_control_cpp',executable='ros_kinematic_calibration_cpp',
        output='screen',additional_env=env,parameters=[{'execute':value('execute')=='true','step_rad':float(value('step_rad'))}])
    def finished(event,context):
        if event.returncode and not context.is_shutdown:
            raise RuntimeError('Calibration did not complete; inspect stage and capture logs')
        return [EmitEvent(event=Shutdown(reason='Bounded calibration ended; external hardware keeps hold'))]
    def lost_camera(event,context):
        if not context.is_shutdown:raise RuntimeError('Calibration camera or recorder exited')
    return [RegisterEventHandler(OnProcessExit(target_action=motion,on_exit=finished)),
            RegisterEventHandler(OnProcessExit(target_action=camera,on_exit=lost_camera)),
            camera,recorder,motion]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('reference_id',default_value='0',choices=['0','1']),
        DeclareLaunchArgument('step_rad',default_value='0.12'),
        DeclareLaunchArgument('ros_domain_id',default_value='25'),
        DeclareLaunchArgument('log_dir'),OpaqueFunction(function=build)])
