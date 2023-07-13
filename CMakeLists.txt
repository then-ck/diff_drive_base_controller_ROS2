cmake_minimum_required(VERSION 3.5)
project(diffdrive_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Most important code - for <rclcpp/rclcpp.h> to work
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Declare C++ library
add_library(base_controller
  src/base_controller.cpp
)

include_directories(
  include include/
)

# Main base_controller executable 
add_executable(base_controller_node 
    src/base_controller.cpp   
)
ament_target_dependencies(base_controller_node rclcpp std_msgs trajectory_msgs geometry_msgs)

target_link_libraries(base_controller_node 
    base_controller
)


install(TARGETS
  base_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
