from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bug_algorithms',
            executable='bug1_node',
            output='screen'
        )
    ])
