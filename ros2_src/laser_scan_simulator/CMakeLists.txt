cmake_minimum_required(VERSION 3.5)
project(laser_scan_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_package()
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
