# Copyright (c) 2026 Khaled Gabr
"""Bring up the Go2 Gazebo simulation and an indoor SLAM backend together.

This is a thin wrapper: the simulation is launched by unitree_go2_sim as-is,
and one of the SLAM launch files in this package is started on top of it once
Gazebo has had time to come up. It deliberately adds no nodes of its own.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    target_frame = LaunchConfiguration("target_frame")
    open_rviz = LaunchConfiguration("rviz")
    slam_backend = LaunchConfiguration("slam")

    slam_share = get_package_share_directory("unitree_indoor_slam")
    sim_share = get_package_share_directory("unitree_go2_sim")
    description_share = get_package_share_directory("unitree_go2_description")

    default_world = os.path.join(description_share, "worlds", "cave_world.sdf")

    declare_world = DeclareLaunchArgument(
        "world", default_value=default_world,
        description="Gazebo world to load; defaults to the indoor cave world")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use the Gazebo simulation clock")
    declare_slam = DeclareLaunchArgument(
        "slam", default_value="slam_toolbox",
        choices=["slam_toolbox", "gmapping", "rtabmap"],
        description="SLAM backend to run on top of the simulation")
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic", default_value="/velodyne_points/points",
        description="Input 3D PointCloud2 topic consumed by the SLAM backend")
    declare_target_frame = DeclareLaunchArgument(
        "target_frame", default_value="velodyne",
        description="LiDAR sensor frame (2D backends only)")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Open RViz with the indoor SLAM config")
    declare_slam_delay = DeclareLaunchArgument(
        "slam_start_delay", default_value="15.0",
        description="Seconds to wait for Gazebo and /clock before starting SLAM")

    # Spawn just outside the cave opening. z=0.35 matches the crouched stand
    # pose set by the joint initial_values in leg.xacro, so the dog settles
    # instead of flipping. These are the poses the shipped maps were recorded
    # from; override on the CLI when using the empty world.
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="-30.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.35")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0")

    # The simulation brings up its own RViz with the general-purpose sim config;
    # suppress it so the only RViz is the one using indoor_slam.rviz.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "unitree_go2_launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
        }.items(),
    )

    def slam_include(launch_file, backend, extra_args=None):
        args = {
            "use_sim_time": use_sim_time,
            "cloud_topic": cloud_topic,
            "rviz": open_rviz,
        }
        if extra_args:
            args.update(extra_args)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", launch_file)
            ),
            launch_arguments=args.items(),
            condition=IfCondition(EqualsSubstitution(slam_backend, backend)),
        )

    # slam_toolbox and gmapping both convert the cloud to a 2D scan and so take
    # target_frame; rtabmap consumes the cloud directly and does not.
    slam_toolbox = slam_include(
        "unitree_slam_toolbox.launch.py", "slam_toolbox",
        {"target_frame": target_frame},
    )
    gmapping = slam_include(
        "unitree_gmapping.launch.py", "gmapping",
        {"target_frame": target_frame},
    )
    rtabmap = slam_include("unitree_rtabmap.launch.py", "rtabmap")

    # slam_toolbox configures and activates its lifecycle node as soon as it is
    # launched, so it needs /clock to already be published when use_sim_time is
    # set. Give Gazebo a head start rather than racing it.
    delayed_slam = TimerAction(
        period=LaunchConfiguration("slam_start_delay"),
        actions=[slam_toolbox, gmapping, rtabmap],
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        declare_slam,
        declare_cloud_topic,
        declare_target_frame,
        declare_rviz,
        declare_slam_delay,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        sim,
        delayed_slam,
    ])
