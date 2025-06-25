from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_now',
            executable='save_map_node',
            name='save_map_node',
            output='screen'
        )
    ])