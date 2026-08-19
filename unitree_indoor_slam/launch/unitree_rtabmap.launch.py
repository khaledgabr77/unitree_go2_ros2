# Copyright (c) 2026 Khaled Gabr
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    base_frame = LaunchConfiguration("frame_id")
    odom_frame = LaunchConfiguration("odom_frame_id")
    open_rviz = LaunchConfiguration("rviz")
    delete_db = LaunchConfiguration("delete_db_on_start")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic", default_value="/velodyne_points/points",
        description="3D LiDAR PointCloud2 topic")
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic", default_value="/odom",
        description="Wheel/EKF odometry topic to fuse with the LiDAR")
    declare_frame_id = DeclareLaunchArgument(
        "frame_id", default_value="base_link",
        description="Robot base frame")
    declare_odom_frame_id = DeclareLaunchArgument(
        "odom_frame_id", default_value="odom",
        description="Odometry frame")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false")
    declare_delete_db = DeclareLaunchArgument(
        "delete_db_on_start", default_value="true",
        description="Delete the previous rtabmap.db on start (fresh map)")

    # LiDAR-only RTAB-Map (uses external odometry from EKF and 3D point cloud).
    # Reg/Force3DoF + Grid/3D=false produce a 2D occupancy grid for indoor use.
    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "frame_id": base_frame,
        "odom_frame_id": odom_frame,
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_rgbd": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud": True,
        "approx_sync": True,
        "queue_size": 30,
        "wait_for_transform": 0.2,
        # Registration / ICP for lidar-only mapping
        "Reg/Strategy": "1",          # 0=Vis, 1=ICP, 2=VisICP
        "Reg/Force3DoF": "true",      # planar indoor robot
        "Icp/PointToPlane": "true",
        "Icp/PointToPlaneK": "20",
        "Icp/PointToPlaneRadius": "0",
        "Icp/VoxelSize": "0.1",
        "Icp/Iterations": "10",
        "Icp/Epsilon": "0.001",
        "Icp/MaxCorrespondenceDistance": "1.0",
        "Icp/PMOutlierRatio": "0.7",
        "Icp/CorrespondenceRatio": "0.2",
        # Loop closure / graph
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/ProximityPathMaxNeighbors": "1",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "RGBD/CreateOccupancyGrid": "true",
        # 2D occupancy grid output
        "Grid/Sensor": "0",           # 0=lidar, 1=depth, 2=both
        "Grid/3D": "false",
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "20.0",
        "Grid/RayTracing": "true",
        "Grid/ClusterRadius": "0.5",
        "Grid/MinClusterSize": "10",
        "Grid/NormalsSegmentation": "false",
        # Memory
        "Mem/NotLinkedNodesKept": "false",
        "Mem/STMSize": "30",
        "Optimizer/Strategy": "1",    # g2o
    }

    rtabmap_remap = [
        ("scan_cloud", cloud_topic),
        ("odom", odom_topic),
    ]

    rtabmap_fresh = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=[rtabmap_params],
        remappings=rtabmap_remap,
        arguments=["--delete_db_on_start"],
        condition=IfCondition(delete_db),
        output="screen",
    )

    rtabmap_keepdb = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=[rtabmap_params],
        remappings=rtabmap_remap,
        condition=UnlessCondition(delete_db),
        output="screen",
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": base_frame,
            "odom_frame_id": odom_frame,
            "subscribe_scan_cloud": True,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "approx_sync": True,
        }],
        remappings=rtabmap_remap,
        condition=IfCondition(open_rviz),
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cloud_topic,
        declare_odom_topic,
        declare_frame_id,
        declare_odom_frame_id,
        declare_rviz,
        declare_delete_db,
        rtabmap_fresh,
        rtabmap_keepdb,
        rtabmap_viz,
    ])
