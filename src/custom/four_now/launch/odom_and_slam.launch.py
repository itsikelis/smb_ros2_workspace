from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Tfs
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "map"],
        output="log",
    )
    static_tf_map_to_graph_msf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_graph_msf",
        arguments=["0", "0", "0", "0", "0", "0", "world_graph_msf", "odom"],
        output="log",
    )

    # Terrain analysis launch include
    terrain_analysis_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("terrain_analysis"),
                        "launch",
                        "terrain_analysis.launch",
                    ]
                )
            ]
        ),
    )

    # Terrain analysis ext launch include
    terrain_analysis_ext_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("terrain_analysis_ext"),
                        "launch",
                        "terrain_analysis_ext.launch",
                    ]
                )
            ]
        ),
    )

    graph_msf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("smb_estimator_graph_ros2"),
                    "launch",
                    "smb_estimator_graph.launch.py",
                ]
            )
        ),
    )
    open_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("open3d_slam_ros"),
                    "launch",
                    "summer_school_slam_robot_launch.py",
                ]
            )
        ),
    )
    local_odometry = Node(
        package="smb_kinematics",
        executable="odometry_and_pointcloud_conversion_graph_msf",
        name="odometry_and_pointcloud_conversion_graph_msf",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription(
        [
            static_tf_map_to_odom,
            static_tf_map_to_graph_msf,
            ###
            terrain_analysis_launch,
            terrain_analysis_ext_launch,
            ###
            graph_msf_launch,
            open_slam_launch,
            local_odometry,
        ]
    )
