from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation')

    params_file = os.path.join(pkg_share, 'params', 'params.yaml')

    return LaunchDescription([
        Node(
            package='navigation',
            executable='navigation',
            name='navigation',
            output='screen',
            parameters=[params_file]
        )
    ])
