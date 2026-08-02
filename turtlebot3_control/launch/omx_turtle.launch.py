from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 실행 모듈
    start_hardware = LaunchConfiguration('start_hardware')
    start_rl_control = LaunchConfiguration('start_rl_control')
    control_runtime = LaunchConfiguration('control_runtime')
    cpp_policy_backend = LaunchConfiguration('cpp_policy_backend')
    cpp_policy_device = LaunchConfiguration('cpp_policy_device')
    start_eef_vision = LaunchConfiguration('start_eef_vision')
    start_mux = LaunchConfiguration('start_mux')
    start_coordinator = LaunchConfiguration('start_coordinator')
    require_base_arrived = LaunchConfiguration('require_base_arrived')
    require_odom_stopped = LaunchConfiguration('require_odom_stopped')
    delivery_pose_required = LaunchConfiguration('delivery_pose_required')

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

            # Bringup이 실제 Stay로 정렬하고 RL 노드는 도달 여부를 검증한다.
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
        parameters=[
            rl_control_config_file,
            {
                'require_base_arrived': require_base_arrived,
                'require_odom_stopped': require_odom_stopped,
                'delivery_pose_required': delivery_pose_required,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", start_rl_control, "'.lower() == 'true' and '",
            control_runtime, "' == 'python'",
        ])),
    )

    rl_control_cpp = Node(
        package='omx_rl_control_cpp',
        executable='rl_control_cpp_node',
        name='rl_control_node',
        output='screen',
        parameters=[
            rl_control_config_file,
            {
                'require_base_arrived': require_base_arrived,
                'require_odom_stopped': require_odom_stopped,
                'delivery_pose_required': delivery_pose_required,
                'cpp_policy_backend': cpp_policy_backend,
                'cpp_policy_device': cpp_policy_device,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", start_rl_control, "'.lower() == 'true' and '",
            control_runtime, "' == 'cpp'",
        ])),
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
            'control_runtime',
            default_value='python',
            choices=['python', 'cpp'],
            description='매니퓰레이터 RL 런타임 구현을 선택한다.',
        ),
        DeclareLaunchArgument(
            'cpp_policy_backend',
            default_value='auto',
            choices=['auto', 'torchscript', 'eigen'],
            description='C++ actor backend를 자동, TorchScript, Eigen 중 선택한다.',
        ),
        DeclareLaunchArgument(
            'cpp_policy_device',
            default_value='auto',
            choices=['auto', 'cuda', 'cpu'],
            description='TorchScript actor 장치를 자동, CUDA, CPU 중 선택한다.',
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
        DeclareLaunchArgument(
            'require_base_arrived',
            default_value='true',
            description='팔 동작 중 fresh base arrival 신호를 요구한다.',
        ),
        DeclareLaunchArgument(
            'require_odom_stopped',
            default_value='true',
            description='팔 동작 중 odometry 정지를 요구한다.',
        ),
        DeclareLaunchArgument(
            'delivery_pose_required',
            default_value='false',
            description=(
                'true이면 PLACE 전에 fresh /target/delivery_pose를 요구한다.'
            ),
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
            default_value='0.0 1.5708 0.0',
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
            actions=[rl_control, rl_control_cpp],
        ),
        cmd_vel_mux,
        coordinator,
    ])
