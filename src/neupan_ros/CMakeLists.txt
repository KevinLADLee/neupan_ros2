cmake_minimum_required(VERSION 3.16)
project(neupan_ros LANGUAGES CXX)

include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(neupan_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(neupan_obstacle_preprocessing STATIC
  src/obstacle_observation.cpp
  src/laser_scan_preprocessor.cpp
  src/pointcloud_preprocessor.cpp)
target_include_directories(neupan_obstacle_preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(neupan_obstacle_preprocessing neupan::neupan)
ament_target_dependencies(neupan_obstacle_preprocessing sensor_msgs)

add_library(neupan_node_component SHARED src/neupan_node.cpp)
target_link_libraries(neupan_node_component
  neupan::neupan neupan_obstacle_preprocessing)
ament_target_dependencies(neupan_node_component
  rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs
  visualization_msgs std_msgs diagnostic_msgs tf2 tf2_ros)
rclcpp_components_register_node(neupan_node_component
  PLUGIN "neupan_ros::NeuPANNode"
  EXECUTABLE neupan_node
  NO_UNDEFINED_SYMBOLS)

# Minimal reference global planner (A* on map -> initial_path) for demos.
add_executable(astar_global_node src/astar_global_node.cpp)
ament_target_dependencies(astar_global_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS neupan_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS astar_global_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pointcloud_preprocessor test/test_pointcloud_preprocessor.cpp)
  target_link_libraries(test_pointcloud_preprocessor neupan_obstacle_preprocessing)
  ament_target_dependencies(test_pointcloud_preprocessor sensor_msgs)

  ament_add_gtest(test_laser_scan_preprocessor test/test_laser_scan_preprocessor.cpp)
  target_link_libraries(test_laser_scan_preprocessor neupan_obstacle_preprocessing)
  ament_target_dependencies(test_laser_scan_preprocessor sensor_msgs)
endif()

# Vendor the DUNE model from neupan_core into our own share, so the launch file
# only depends on this (ament) package's share directory. neupan_core is a
# plain-CMake package and is therefore not discoverable via the ament index.
install(DIRECTORY ${neupan_core_MODELS_DIR}/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
