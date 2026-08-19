import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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


def generate_launch_description():
    config_pkg_share = _required_share_directory("champ_config", "unitree_go2_sim")
    description_pkg_share = _required_share_directory(
        "unitree_go2_description", "champ_description"
    )
    rviz_pkg_share = _optional_share_directory("champ_description")
    unitree_sim_pkg_share = _optional_share_directory("unitree_go2_sim")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    default_rviz_path = _first_existing_path(
        os.path.join(rviz_pkg_share, "rviz/urdf_viewer.rviz") if rviz_pkg_share else None,
        os.path.join(unitree_sim_pkg_share, "rviz/rviz.rviz")
        if unitree_sim_pkg_share
        else None,
    )
    default_model_path = _first_existing_path(
        os.path.join(description_pkg_share, "urdf/unitree_go2_robot.xacro"),
        os.path.join(description_pkg_share, "urdf/champ.urdf.xacro"),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_description_path = DeclareLaunchArgument(
        name="description_path",
        default_value=default_model_path,
        description="Absolute path to robot urdf/xacro file",
    )
    declare_rviz_path = DeclareLaunchArgument(
        name="rviz_path",
        default_value=default_rviz_path,
        description="Absolute path to rviz file",
    )
    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path",
        default_value=joints_config,
        description="Absolute path to joints map file",
    )
    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path",
        default_value=links_config,
        description="Absolute path to links map file",
    )
    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path",
        default_value=gait_config,
        description="Absolute path to gait config file",
    )
    declare_orientation_from_imu = DeclareLaunchArgument(
        "orientation_from_imu",
        default_value="false",
        description="Take orientation from IMU data",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_rviz_ref_frame = DeclareLaunchArgument(
        "rviz_ref_frame", default_value="odom", description="Rviz ref frame"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="/", description="Robot name"
    )
    declare_base_link_frame = DeclareLaunchArgument(
        "base_link_frame", default_value="base_link", description="Base link frame"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_gazebo = DeclareLaunchArgument(
        "gazebo", default_value="false", description="If in gazebo"
    )
    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic",
        default_value="joint_group_effort_controller/joint_trajectory",
        description="Joint controller topic",
    )
    declare_hardware_connected = DeclareLaunchArgument(
        "hardware_connected",
        default_value="false",
        description="Whether hardware is connected",
    )
    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control",
        default_value="true",
        description="Publish joint control",
    )
    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states",
        default_value="true",
        description="Publish joint states",
    )
    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts",
        default_value="true",
        description="Publish foot contacts",
    )
    declare_publish_odom_tf = DeclareLaunchArgument(
        "publish_odom_tf",
        default_value="true",
        description="Publish odom tf from cmd_vel estimation",
    )
    declare_close_loop_odom = DeclareLaunchArgument(
        "close_loop_odom", default_value="false", description=""
    )

    # Wrap with ParameterValue(..., value_type=str) so launch_ros doesn't try
    # to YAML-parse the URDF (which fails on Jazzy with: "Unable to parse the
    # value of parameter robot_description as yaml"). The substitution still
    # runs xacro at launch time; we just tell the parameter machinery the
    # result is a string, not arbitrary YAML.
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("description_path")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": robot_description},
            LaunchConfiguration("joints_map_path"),
            LaunchConfiguration("links_map_path"),
            LaunchConfiguration("gait_config_path"),
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"orientation_from_imu": LaunchConfiguration("orientation_from_imu")},
            {"urdf": robot_description},
            LaunchConfiguration("joints_map_path"),
            LaunchConfiguration("links_map_path"),
            LaunchConfiguration("gait_config_path"),
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    rviz2 = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_path")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            declare_rviz_path,
            declare_joints_map_path,
            declare_links_map_path,
            declare_gait_config_path,
            declare_orientation_from_imu,
            declare_rviz,
            declare_rviz_ref_frame,
            declare_robot_name,
            declare_base_link_frame,
            declare_lite,
            declare_gazebo,
            declare_joint_controller_topic,
            declare_hardware_connected,
            declare_publish_joint_control,
            declare_publish_joint_states,
            declare_publish_foot_contacts,
            declare_publish_odom_tf,
            declare_close_loop_odom,
            robot_state_publisher,
            quadruped_controller_node,
            state_estimator_node,
            # base_to_footprint_ekf,
            # footprint_to_odom_ekf,
            rviz2,
        ]
    )
