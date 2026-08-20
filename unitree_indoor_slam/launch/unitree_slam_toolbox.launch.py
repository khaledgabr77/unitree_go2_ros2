# Copyright (c) 2026 Khaled Gabr
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
import launch.events


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    target_frame = LaunchConfiguration("target_frame")
    open_rviz = LaunchConfiguration("rviz")

    pkg_share = get_package_share_directory("unitree_indoor_slam")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_cfg = os.path.join(pkg_share, "rviz", "indoor_slam.rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation clock")
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic", default_value="/velodyne_points/points",
        description="Input 3D PointCloud2 topic to convert into a 2D LaserScan")
    declare_target_frame = DeclareLaunchArgument(
        "target_frame", default_value="velodyne",
        description="LiDAR sensor frame used as the LaserScan frame")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Open RViz with the indoor SLAM config")

    pc_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": target_frame,
            "transform_tolerance": 0.01,
            "min_height": -0.2,
            "max_height": 1.5,
            "angle_min": -3.141592,
            "angle_max":  3.141592,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.5,
            "range_max": 20.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    # async_slam_toolbox_node is a lifecycle node and does NOT self-activate:
    # `autostart` is a nav2_lifecycle_manager parameter, not a slam_toolbox one
    # (the string does not appear anywhere in the slam_toolbox binaries), so
    # putting it in slam_toolbox_params.yaml has no effect. The configure /
    # activate events at the bottom of this file are the only thing that drives
    # unconfigured -> inactive -> active. Without them the node sits in
    # `unconfigured`, never subscribes to /scan and never publishes map->odom.
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(open_rviz),
        output="screen",
    )
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(slam_toolbox_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    # Fires once, on the configuring -> inactive transition, and then
    # unregisters itself; re-firing would try to ACTIVATE an already-active
    # node and log "Unable to start transition 3 from current state active".
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            handle_once=True,
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cloud_topic,
        declare_target_frame,
        declare_rviz,
        pc_to_scan,
        slam_toolbox_node,
        rviz_node,
        configure_event,
        activate_after_configure,
    ])
