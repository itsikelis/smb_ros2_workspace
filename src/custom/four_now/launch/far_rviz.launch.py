from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="far_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory("smb_bringup"), "rviz", "debug.rviz"]
            ),
        ],
        respawn=False,
    )

    return LaunchDescription([rviz2])
