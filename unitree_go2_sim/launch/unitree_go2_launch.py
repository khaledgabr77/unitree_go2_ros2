# Copyright (c) 2026 Khaled Gabr
import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim"
    ).find("unitree_go2_sim")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description"
    ).find("unitree_go2_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "config/ros_control/ros_control.yaml"  
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(
        descr_pkg_share, "urdf/unitree_go2_robot.xacro"
    )
    default_world_path = os.path.join(
        descr_pkg_share, "worlds/cave_world.sdf"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="default",
        description="World name as declared inside the loaded SDF (<world name=...>)",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    # Spawn just outside the cave-style opening on the +X face of the cave tile,
    # facing −X so the robot looks into the cave.
    # Override on the CLI for the empty/default world, e.g. world_init_x:=0.0 …
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="-30.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value=".0")
    # 0.35 m matches the crouched stand pose set as initial_value on the
    # leg joints in unitree_go2_description/urdf/leg.xacro. With
    # upper_leg=0.8 rad and lower_leg=-1.5 rad each foot sits ~0.31 m below
    # the trunk, so spawning the trunk at z=0.35 puts the feet ~1-2 cm above
    # the floor and the impact is negligible. The previous 0.5 m drop (with
    # legs at angle 0, i.e. straight down 0.43 m poles) was the main cause
    # of the robot flipping onto its back at spawn.
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.35")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    declare_entity = DeclareLaunchArgument("entity", default_value="go2")
    declare_gz_topic = DeclareLaunchArgument("gz_topic", default_value="/model/go2/pose")
    declare_odom_topic = DeclareLaunchArgument("odom_topic", default_value="/odom")
    declare_tf_parent = DeclareLaunchArgument("tf_parent", default_value="odom")
    declare_tf_child = DeclareLaunchArgument("tf_child", default_value="base_link")
    declare_idle_timeout = DeclareLaunchArgument("idle_timeout", default_value="5")
    declare_max_backoff = DeclareLaunchArgument("max_backoff", default_value="10")
    declare_median_window = DeclareLaunchArgument("median_window", default_value="3")
    declare_alpha = DeclareLaunchArgument("alpha", default_value="0.3")
    declare_deadband_lin = DeclareLaunchArgument("deadband_lin", default_value="0.01")
    declare_deadband_ang = DeclareLaunchArgument("deadband_ang", default_value="0.01")
    declare_max_dt = DeclareLaunchArgument("max_dt", default_value="0.5")

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "world_name": LaunchConfiguration("world_name"),
            "description_path": default_model_path,
            "ros_control_file": LaunchConfiguration("ros_control_file"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    gz_pose_to_ros2_proc = Node(
        package="unitree_go2_sim",
        executable="gz_pose_to_ros2.py",
        output="screen",
        arguments=[
            '--entity', LaunchConfiguration('entity'),
            '--gz-topic', LaunchConfiguration('gz_topic'),
            '--odom-topic', LaunchConfiguration('odom_topic'),
            '--tf-parent', LaunchConfiguration('tf_parent'),
            '--tf-child', LaunchConfiguration('tf_child'),
            '--idle-timeout', LaunchConfiguration('idle_timeout'),
            '--max-backoff', LaunchConfiguration('max_backoff'),
            '--median-window', LaunchConfiguration('median_window'),
            '--alpha', LaunchConfiguration('alpha'),
            '--deadband-lin', LaunchConfiguration('deadband_lin'),
            '--deadband-ang', LaunchConfiguration('deadband_ang'),
            '--max-dt', LaunchConfiguration('max_dt'),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_name,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,

            declare_entity,
            declare_gz_topic,
            declare_odom_topic,
            declare_tf_parent,
            declare_tf_child,
            declare_idle_timeout,
            declare_max_backoff,
            declare_median_window,
            declare_alpha,
            declare_deadband_lin,
            declare_deadband_ang,
            declare_max_dt,

            bringup_ld,
            gazebo_ld,
        ]
    )