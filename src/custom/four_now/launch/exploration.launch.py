from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    far_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("far_planner"), "launch", "far_planner.launch"]
                )
            ]
        ),
    )

    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("tare_planner"), "/explore_robotx.launch"]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "rviz": "true",
        }.items(),
    )

    local_planner_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("local_planner"),
                        "launch",
                        "local_planner.launch.py",
                    ]
                )
            ]
        ),
    )

    return LaunchDescription(
        [
            exploration_launch,
            # far_planner_launch,
            local_planner_launch,
        ]
    )
