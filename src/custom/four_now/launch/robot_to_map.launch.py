from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_now',
            executable='robot_to_map_node',
            name='robot_to_map_node',
            output='screen'
        )
    ])