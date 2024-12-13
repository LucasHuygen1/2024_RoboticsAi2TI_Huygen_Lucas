from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_mediapipe_lucas_pkg',
            executable='gesture_control',
            name='gesture_control'
        ),
        Node(
            package='lidar_mediapipe_lucas_pkg',
            executable='lidar_navigation',
            name='lidar_navigation'
        ),
    ])
