from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('custom_nav'),
        'config',
        'nav_srv.yaml'
        )
    return LaunchDescription([
        Node(
            package='custom_nav',
            executable='nav_srv_node',
            name='custom_nav',
            parameters=[config],
        ),
    ])