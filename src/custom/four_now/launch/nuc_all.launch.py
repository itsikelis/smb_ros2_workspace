from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_id = SetEnvironmentVariable("ROBOT_ID", "262")

    # Path to the URDF file
    description_file = PathJoinSubstitution(
        [FindPackageShare("smb_description"), "urdf", "smb.urdf.xacro"]
    )
    # Generate the robot description using xacro
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]), value_type=str
    )
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # Lidar node
    rslidar_config_file = (
        get_package_share_directory("smb_bringup") + "/config/rslidar_config.yaml"
    )
    rslidar = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": rslidar_config_file}],
    )

    # Low Level Control
    default_config_topics = os.path.join(
        get_package_share_directory("smb_bringup"), "config", "twist_mux_topics.yaml"
    )
    config_topics = DeclareLaunchArgument(
        "config_topics",
        default_value=default_config_topics,
        description="Default topics config file",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings={("/cmd_vel_out", "/cmd_vel")},
        parameters=[{"use_sim_time": False}, LaunchConfiguration("config_topics")],
    )
    twist_pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("twist_pid_controller"),
                    "launch",
                    "twist_pid_controller.launch.py",
                ]
            )
        ),
    )
    # Kinematics controller node
    kinematics_controller = Node(
        package="smb_kinematics",
        executable="smb_kinematics_node",
        name="smb_kinematics_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    # Low Level Controller node
    low_level_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("smb_low_level_controller"),
                        "launch",
                        "speed_control_node.launch.py",
                    ]
                )
            ]
        ),
    )

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
            robot_state_publisher_node,
            rslidar,
            ###
            config_topics,
            twist_mux,
            twist_pid,
            kinematics_controller,
            low_level_controller,
            ## SLAM - ODOMETRY
            static_tf_map_to_odom,
            static_tf_map_to_graph_msf,
            ###
            # terrain_analysis_launch,
            # terrain_analysis_ext_launch,
            # ###
            # graph_msf_launch,
            # open_slam_launch,
            # local_odometry,
            # ## EXPLORATION
            # exploration_launch,
            # # far_planner_launch,
            # local_planner_launch,
        ]
    )
