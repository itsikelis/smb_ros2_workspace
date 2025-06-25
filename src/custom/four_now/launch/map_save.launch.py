from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_now',
            executable='map_logger_node',
            name='map_logger_node',
            output='screen'
        )
    ])
