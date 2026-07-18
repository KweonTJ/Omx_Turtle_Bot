import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('turtlebot3_position'),
        'config',
        'position.yaml',
    )

    return LaunchDescription([
        Node(
            package='turtlebot3_position',
            executable='uwb_serial_node',
            name='uwb_serial_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='turtlebot3_position',
            executable='position_controller_node',
            name='position_controller_node',
            parameters=[config],
            output='screen',
        ),
    ])
