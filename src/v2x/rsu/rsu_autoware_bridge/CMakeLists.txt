cmake_minimum_required(VERSION 3.8)
project(rsu_autoware_bridge)

# C++ standard (C++17 recommended for ROS 2 Humble/Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)

# Include directories setting
include_directories(include)

# ========================================
#  1. Vehicle Bridge Node
# ========================================
# Build as a library (Component)
add_library(vehicle_bridge_node_lib SHARED
  src/vehicle_bridge_node.cpp
)
target_include_directories(vehicle_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vehicle_bridge_node_lib
  rclcpp
  rclcpp_components
  autoware_auto_planning_msgs
  geometry_msgs
  tf2_msgs
  autoware_universe_utils
)

# Build executable (link file with main function)
add_executable(vehicle_bridge_node src/vehicle_bridge_node.cpp)
target_link_libraries(vehicle_bridge_node vehicle_bridge_node_lib)
ament_target_dependencies(vehicle_bridge_node
  rclcpp
  autoware_auto_planning_msgs
  geometry_msgs
  autoware_universe_utils
)

# Register as a component (for easy launching)
rclcpp_components_register_node(vehicle_bridge_node_lib
  PLUGIN "rsu_autoware_bridge::VehicleBridgeNode"
  EXECUTABLE vehicle_bridge_node_exe
)

# ========================================
#  2. RSU Bridge Node
# ========================================
# Build as a library
add_library(rsu_bridge_node_lib SHARED
  src/rsu_bridge_node.cpp
)
target_include_directories(rsu_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rsu_bridge_node_lib
  rclcpp
  rclcpp_components
  autoware_auto_planning_msgs
  autoware_planning_msgs
  geometry_msgs
  tf2_msgs
  autoware_universe_utils
)

# Build executable
add_executable(rsu_bridge_node src/rsu_bridge_node.cpp)
target_link_libraries(rsu_bridge_node rsu_bridge_node_lib)
ament_target_dependencies(rsu_bridge_node rclcpp
  autoware_auto_planning_msgs
  autoware_planning_msgs
  geometry_msgs
  tf2_msgs
  autoware_universe_utils
)

# Register as a component
rclcpp_components_register_node(rsu_bridge_node_lib
  PLUGIN "rsu_autoware_bridge::RsuBridgeNode"
  EXECUTABLE rsu_bridge_node_exe
)

# ========================================
#  3. Arbiter Node
# ========================================
add_library(vehicle_trajectory_arbiter_node_lib SHARED
  src/vehicle_trajectory_arbiter_node.cpp
)
target_include_directories(vehicle_trajectory_arbiter_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vehicle_trajectory_arbiter_node_lib
  rclcpp
  rclcpp_components
  autoware_auto_planning_msgs
)

add_executable(vehicle_trajectory_arbiter_node src/vehicle_trajectory_arbiter_node.cpp)
target_link_libraries(vehicle_trajectory_arbiter_node vehicle_trajectory_arbiter_node_lib)
ament_target_dependencies(vehicle_trajectory_arbiter_node rclcpp autoware_auto_planning_msgs)

rclcpp_components_register_node(vehicle_trajectory_arbiter_node_lib
  PLUGIN "rsu_autoware_bridge::TrajectoryArbiterNode"
  EXECUTABLE vehicle_trajectory_arbiter_node_exe
)

# ========================================
#  Install settings
# ========================================
install(TARGETS
  vehicle_bridge_node
  rsu_bridge_node
  vehicle_trajectory_arbiter_node
  vehicle_bridge_node_lib
  rsu_bridge_node_lib
  vehicle_trajectory_arbiter_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
