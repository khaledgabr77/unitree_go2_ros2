import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _optional_share_directory(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None


def _required_share_directory(*package_names):
    for package_name in package_names:
        share_directory = _optional_share_directory(package_name)
        if share_directory is not None:
            return share_directory
    raise PackageNotFoundError(package_names[0])


def _first_existing_path(*paths):
    for path in paths:
        if path and os.path.exists(path):
            return path
    return next((path for path in paths if path), "")


def _launch_setup(context, *args, **kwargs):
    gazebo_pkg_share = get_package_share_directory("champ_gazebo")
    description_pkg_share = _required_share_directory(
        "unitree_go2_description", "champ_description"
    )

    system_plugin_paths = [
        path
        for path in (
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            os.environ.get("LD_LIBRARY_PATH", ""),
        )
        if path
    ]
    resource_paths = [
        path
        for path in (
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            description_pkg_share,
            gazebo_pkg_share,
        )
        if path
    ]

    paused = LaunchConfiguration("paused").perform(context).lower() == "true"
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"

    gz_sim_cmd = ["gz", "sim"]
    if not paused:
        gz_sim_cmd.append("-r")
    if headless or not gui:
        gz_sim_cmd.append("-s")
    gz_sim_cmd.append(LaunchConfiguration("world"))

    robot_description = Command(
        [
            "xacro ",
            LaunchConfiguration("description_path"),
            " robot_controllers:=",
            LaunchConfiguration("ros_control_file"),
        ]
    )

    start_gazebo = ExecuteProcess(
        cmd=gz_sim_cmd,
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(resource_paths),
            "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(system_plugin_paths),
        },
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/velodyne_points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/rgb_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/odom_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # add /unitree_lidar, /d455/* as needed
        ],
        remappings=[
            ("/odom_tf", "/tf"),
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": ParameterValue(
                    LaunchConfiguration("world_name"), value_type=str
                ),
                "string": ParameterValue(robot_description, value_type=str),
                "name": ParameterValue(
                    LaunchConfiguration("robot_name"), value_type=str
                ),
                "allow_renaming": False,
                "x": ParameterValue(
                    LaunchConfiguration("world_init_x"), value_type=float
                ),
                "y": ParameterValue(
                    LaunchConfiguration("world_init_y"), value_type=float
                ),
                "z": ParameterValue(
                    LaunchConfiguration("world_init_z"), value_type=float
                ),
                "R": 0.0,
                "P": 0.0,
                "Y": ParameterValue(
                    LaunchConfiguration("world_init_heading"), value_type=float
                ),
            }
        ],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_states_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    load_joint_trajectory_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_group_effort_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    return [
        start_gazebo,
        clock_bridge,
        sensor_bridge,
        TimerAction(period=2.0, actions=[spawn_robot]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    load_joint_state_controller,
                    load_joint_trajectory_effort_controller,
                ],
            )
        ),
    ]


def generate_launch_description():
    gazebo_pkg_share = get_package_share_directory("champ_gazebo")
    description_pkg_share = _required_share_directory(
        "unitree_go2_description", "champ_description"
    )

    default_description_path = _first_existing_path(
        os.path.join(description_pkg_share, "urdf/unitree_go2_robot.xacro"),
        os.path.join(description_pkg_share, "urdf/champ.urdf.xacro"),
    )
    default_world_path = _first_existing_path(
        os.path.join(description_pkg_share, "worlds/default.sdf"),
        os.path.join(gazebo_pkg_share, "worlds/default.world"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="champ"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument(
                "description_path",
                default_value=default_description_path,
                description="Absolute path to robot urdf/xacro file",
            ),
            DeclareLaunchArgument(
                "ros_control_file",
                default_value=os.path.join(gazebo_pkg_share, "config/ros_control.yaml"),
            ),
            DeclareLaunchArgument("world", default_value=default_world_path),
            DeclareLaunchArgument(
                "world_name",
                default_value="default",
                description="World name inside the loaded SDF",
            ),
            DeclareLaunchArgument("world_init_x", default_value="0.0"),
            DeclareLaunchArgument("world_init_y", default_value="0.0"),
            DeclareLaunchArgument("world_init_z", default_value="0.6"),
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
