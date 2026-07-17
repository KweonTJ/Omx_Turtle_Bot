from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 실행 모듈
    start_hardware = LaunchConfiguration('start_hardware')
    start_rl_control = LaunchConfiguration('start_rl_control')
    start_eef_vision = LaunchConfiguration('start_eef_vision')
    start_mux = LaunchConfiguration('start_mux')
    start_coordinator = LaunchConfiguration('start_coordinator')

    # 하드웨어 설정
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_rviz = LaunchConfiguration('start_rviz')
    start_camera = LaunchConfiguration('start_camera')
    start_eef_camera_driver = LaunchConfiguration('start_eef_camera_driver')
    start_state_relays = LaunchConfiguration('start_state_relays')
    start_lidar = LaunchConfiguration('start_lidar')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')

    # EEF 카메라 장착 TF
    use_eef_usb_camera = LaunchConfiguration('use_eef_usb_camera')
    eef_usb_camera_parent = LaunchConfiguration('eef_usb_camera_parent')
    eef_usb_camera_xyz = LaunchConfiguration('eef_usb_camera_xyz')
    eef_usb_camera_rpy = LaunchConfiguration('eef_usb_camera_rpy')

    # EEF 카메라 드라이버
    eef_camera_video_device = LaunchConfiguration('eef_camera_video_device')
    eef_camera_frame_id = LaunchConfiguration('eef_camera_frame_id')
    eef_camera_pixel_format = LaunchConfiguration('eef_camera_pixel_format')
    eef_camera_output_encoding = LaunchConfiguration('eef_camera_output_encoding')
    eef_camera_image_width = LaunchConfiguration('eef_camera_image_width')
    eef_camera_image_height = LaunchConfiguration('eef_camera_image_height')
    eef_camera_name = LaunchConfiguration('eef_camera_name')
    eef_camera_info_url = LaunchConfiguration('eef_camera_info_url')

    # 설정 파일
    rl_control_config_file = LaunchConfiguration('rl_control_config_file')
    eef_vision_config_file = LaunchConfiguration('eef_vision_config_file')
    mux_config_file = LaunchConfiguration('mux_config_file')
    coordinator_config_file = LaunchConfiguration('coordinator_config_file')

    # ---------------------------------------------------------
    # TurtleBot3 + OpenMANIPULATOR-X 실제 하드웨어
    # ---------------------------------------------------------
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(
                    'turtlebot3_manipulation_bringup'
                ),
                'launch',
                'hardware.launch.py',
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'start_rviz': start_rviz,

            # RL 노드가 자체적으로 Stay 자세를 관리한다.
            'move_to_stay_pose': 'true',

            'start_camera': start_camera,
            'start_eef_camera_driver': start_eef_camera_driver,
            'start_state_relays': start_state_relays,
            'start_lidar': start_lidar,
            'lidar_port': lidar_port,
            'lidar_frame_id': lidar_frame_id,

            'use_eef_usb_camera': use_eef_usb_camera,
            'eef_usb_camera_parent': eef_usb_camera_parent,
            'eef_usb_camera_xyz': eef_usb_camera_xyz,
            'eef_usb_camera_rpy': eef_usb_camera_rpy,

            'eef_camera_video_device': eef_camera_video_device,
            'eef_camera_frame_id': eef_camera_frame_id,
            'eef_camera_pixel_format': eef_camera_pixel_format,
            'eef_camera_output_encoding':
                eef_camera_output_encoding,
            'eef_camera_image_width': eef_camera_image_width,
            'eef_camera_image_height': eef_camera_image_height,
            'eef_camera_name': eef_camera_name,
            'eef_camera_info_url': eef_camera_info_url,
        }.items(),
        condition=IfCondition(start_hardware),
    )

    # ---------------------------------------------------------
    # ArUco 기반 EEF 비전
    # ---------------------------------------------------------
    eef_vision = Node(
        package='omx_eef_vision',
        executable='eef_vision_node',
        name='eef_vision_node',
        output='screen',
        parameters=[eef_vision_config_file],
        condition=IfCondition(start_eef_vision),
    )

    # ---------------------------------------------------------
    # PPO residual 기반 OpenMANIPULATOR-X 제어
    # ---------------------------------------------------------
    rl_control = Node(
        package='omx_rl_control',
        executable='rl_control_node',
        name='rl_control_node',
        output='screen',
        parameters=[rl_control_config_file],
        condition=IfCondition(start_rl_control),
    )

    # ---------------------------------------------------------
    # TurtleBot3 cmd_vel 선택기
    # ---------------------------------------------------------
    cmd_vel_mux = Node(
        package='turtlebot3_control',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen',
        parameters=[mux_config_file],
        condition=IfCondition(start_mux),
    )

    # ---------------------------------------------------------
    # 주행 도착 → ArUco 확인 → RL PICK 명령
    #
    # omx_rl_control이 호환 토픽을 제공하므로 기존 Coordinator 사용 가능:
    # /mp_control/start
    # /mp_control/status
    # ---------------------------------------------------------
    coordinator = Node(
        package='turtlebot3_control',
        executable='omx_turtle_node',
        name='omx_turtle_node',
        output='screen',
        parameters=[coordinator_config_file],
        condition=IfCondition(start_coordinator),
    )

    return LaunchDescription([
        # 실행 모듈
        DeclareLaunchArgument(
            'start_hardware',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_rl_control',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_eef_vision',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_mux',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_coordinator',
            default_value='true',
        ),

        # 하드웨어
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='false',
            description='Astra 카메라는 현재 ArUco 파지에 사용하지 않음.',
        ),
        DeclareLaunchArgument(
            'start_eef_camera_driver',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_state_relays',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='base_scan',
        ),

        # EEF 카메라 장착 TF
        DeclareLaunchArgument(
            'use_eef_usb_camera',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'eef_usb_camera_parent',
            default_value='dummy_mimic_fix',
        ),
        DeclareLaunchArgument(
            'eef_usb_camera_xyz',
            default_value='0.02 0.0 0.065',
        ),
        DeclareLaunchArgument(
            'eef_usb_camera_rpy',
            default_value='0.0 0.0 0.0',
        ),

        # EEF 카메라
        DeclareLaunchArgument(
            'eef_camera_video_device',
            default_value='/dev/video0',
        ),
        DeclareLaunchArgument(
            'eef_camera_frame_id',
            default_value='eef_usb_camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'eef_camera_pixel_format',
            default_value='YUYV',
        ),
        DeclareLaunchArgument(
            'eef_camera_output_encoding',
            default_value='rgb8',
        ),
        DeclareLaunchArgument(
            'eef_camera_image_width',
            default_value='320',
        ),
        DeclareLaunchArgument(
            'eef_camera_image_height',
            default_value='240',
        ),
        DeclareLaunchArgument(
            'eef_camera_name',
            default_value='eef_usb_camera',
        ),
        DeclareLaunchArgument(
            'eef_camera_info_url',
            default_value=[
                'file://',
                PathJoinSubstitution([
                    FindPackageShare(
                        'turtlebot3_manipulation_bringup'
                    ),
                    'config',
                    'eef_usb_camera.yaml',
                ]),
            ],
        ),

        # 설정 파일
        DeclareLaunchArgument(
            'rl_control_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_rl_control'),
                'config',
                'rl_control.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'eef_vision_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('omx_eef_vision'),
                'config',
                'eef_vision.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'mux_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'cmd_vel_mux.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'coordinator_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_control'),
                'config',
                'omx_turtle.yaml',
            ]),
        ),

        hardware,
        eef_vision,
        TimerAction(
            period=8.0,
            actions=[rl_control],
        ),
        cmd_vel_mux,
        coordinator,
    ])