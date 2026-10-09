import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('volleyball_tracker')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')

    node = Node(
        package='volleyball_tracker',
        executable='volleyball_tracker_node',
        name='volleyball_tracker_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([node])
