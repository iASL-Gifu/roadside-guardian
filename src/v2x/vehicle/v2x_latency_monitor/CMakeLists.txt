cmake_minimum_required(VERSION 3.8)
project(v2x_latency_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

add_executable(latency_monitor_node src/latency_monitor_node.cpp)
ament_target_dependencies(latency_monitor_node
  rclcpp
  std_msgs
  autoware_auto_planning_msgs
)

install(TARGETS latency_monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()