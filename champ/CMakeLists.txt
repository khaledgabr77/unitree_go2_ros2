
cmake_minimum_required(VERSION 3.5)
project(champ)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)
set(LIBRARY_NAME "${PROJECT_NAME}_lib")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIBRARY_NAME} INTERFACE
  ${rclcpp_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_package()
