cmake_minimum_required(VERSION 3.5)
project(route_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FollowerState.msg"
  "msg/ManagerStatus.msg"
  "msg/MissionInfo.msg"
  "msg/Route.msg"
  "msg/RouteState.msg"
  "msg/Waypoint.msg"
  "msg/ObstacleAvoidanceHint.msg"
  "srv/GetRoute.srv"
  "srv/UpdateRoute.srv"
  "srv/ReportStuck.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
