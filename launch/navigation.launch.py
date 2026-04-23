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
            parameters=[params_file],
            remappings=[
                ('map_sub', '/map'),
                ('goal_sub', '/goal_pose'),
                ('odom_sub', '/odom'),
                ('cmd_vel', '/cmd_vel'),
                ('path_pub', '/nav/planner/hybrid_astar_path'),
                ('footprint_pub', '/nav/costmap/footprint'),
                ('global_costmap_pub', '/nav/costmap/global_costmap'),
                ('local_costmap_pub', '/nav/costmap/local_costmap'),
                ('lookahead_pub', '/nav/controller/lookahead_pose'),
                ('trajectory_pub', '/nav/controller/trajectory'),
            ]
        )
    ])
