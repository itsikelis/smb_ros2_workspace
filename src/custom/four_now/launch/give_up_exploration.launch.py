from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_now',
            executable='give_up_exploration_node',
            name='give_up_exploration_node',
            output='screen'
        )
    ])