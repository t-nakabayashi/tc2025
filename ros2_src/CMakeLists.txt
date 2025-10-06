cmake_minimum_required(VERSION 3.8)
project(tc2025_ros2)

# Default to C++17 to match modern ROS 2 templates.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

ament_package()
