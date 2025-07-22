from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('yolo_follower')
    params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='yolo_follower',
            executable='detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='yolo_follower',
            executable='follower_node',
            name='yolo_follower',
            output='screen',
            parameters=[params],
        ),
    ])