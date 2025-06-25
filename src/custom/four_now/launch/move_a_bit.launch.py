from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_now',
            executable='move_a_bit_node',
            name='move_a_bit_node',
            output='screen'
        )
    ])