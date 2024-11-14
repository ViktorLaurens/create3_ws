from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='create3_control',
            executable='move_to_goal',
            name='move_to_goal',
            output='screen',
        )
    ])
