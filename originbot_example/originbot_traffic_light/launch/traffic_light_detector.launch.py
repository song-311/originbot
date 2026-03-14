import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('originbot_traffic_light')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='originbot_traffic_light',
            executable='traffic_light_detector_node',
            name='traffic_light_detector',
            output='screen',
            parameters=[params_file],
        )
    ])
