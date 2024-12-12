from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_control_pkg',  
            executable='movement',           
            name='movement_node',           
            output='screen',                 
        ),
    ])
