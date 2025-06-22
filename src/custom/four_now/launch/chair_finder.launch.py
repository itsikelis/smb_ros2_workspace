from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    exploration = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('smb_bringup'),
                    'launch',
                    'smb_sim_exploration.launch.py'
                ])
            )
        )
    
    return LaunchDescription([
        exploration,
    ])
