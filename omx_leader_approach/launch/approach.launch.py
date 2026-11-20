"""Launch only the approach proposal node; never enable a hardware mux."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('omx_leader_approach'))/'config/approach.yaml'
    return LaunchDescription([Node(package='omx_leader_approach', executable='approach_node',
                                   name='leader_approach', output='screen', parameters=[str(config)])])
