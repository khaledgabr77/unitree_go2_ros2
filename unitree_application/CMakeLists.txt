cmake_minimum_required(VERSION 3.5)
project(unitree_application)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

set(dependencies
  rclcpp
  champ_msgs
  ros_gz_interfaces
)

add_executable(foot_contacts_bridge src/foot_contacts_bridge.cpp)
ament_target_dependencies(foot_contacts_bridge ${dependencies})

install(TARGETS
    foot_contacts_bridge
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
