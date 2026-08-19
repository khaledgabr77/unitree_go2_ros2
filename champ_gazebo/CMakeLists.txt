cmake_minimum_required(VERSION 3.8)
project(champ_gazebo)

find_package(ament_cmake REQUIRED)

option(
  CHAMP_GAZEBO_BUILD_CLASSIC_CONTACT_SENSOR
  "Build the legacy Gazebo Classic contact sensor executable"
  OFF
)

install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

if(CHAMP_GAZEBO_BUILD_CLASSIC_CONTACT_SENSOR)
  add_compile_options(-std=c++17)

  find_package(gazebo REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(urdf REQUIRED)
  find_package(champ REQUIRED)
  find_package(champ_msgs REQUIRED)

  include_directories(
    include
    ${champ_INCLUDE_DIRS}
    ${champ_INCLUDE_DIRS}/champ/
    ${champ_INCLUDE_DIRS}/champ/champ/
    ${GAZEBO_INCLUDE_DIRS}
  )

  add_executable(contact_sensor src/contact_sensor.cpp)
  target_link_libraries(contact_sensor ${GAZEBO_LIBRARIES})
  ament_target_dependencies(contact_sensor rclcpp urdf champ champ_msgs)

  install(
    TARGETS contact_sensor
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
