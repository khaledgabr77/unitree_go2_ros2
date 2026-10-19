cmake_minimum_required(VERSION 3.5)
project(unitree_application)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  champ_msgs
  ros_gz_interfaces
)

add_executable(foot_contacts_bridge src/foot_contacts_bridge.cpp)
ament_target_dependencies(foot_contacts_bridge ${dependencies})

add_executable(tf_to_odom_node src/tf_to_odom_node.cpp)
ament_target_dependencies(tf_to_odom_node rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS
    foot_contacts_bridge
    tf_to_odom_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
