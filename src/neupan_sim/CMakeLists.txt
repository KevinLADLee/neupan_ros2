cmake_minimum_required(VERSION 3.16)
project(neupan_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(neupan_simulation STATIC src/simulation.cpp)
target_include_directories(neupan_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(neupan_simulation PRIVATE -O3 -Wall -Wextra -Wpedantic)

add_executable(neupan_sim_node src/neupan_sim_node.cpp)
target_link_libraries(neupan_sim_node neupan_simulation)
ament_target_dependencies(neupan_sim_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_srvs tf2_ros
  visualization_msgs)
target_compile_options(neupan_sim_node PRIVATE -O3 -Wall -Wextra -Wpedantic)

install(TARGETS neupan_simulation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
add_executable(neupan_fleet_sim src/neupan_sim_node.cpp)
target_compile_definitions(neupan_fleet_sim PRIVATE NEUPAN_FLEET_MAIN)
target_link_libraries(neupan_fleet_sim neupan_simulation)
ament_target_dependencies(neupan_fleet_sim
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs std_srvs tf2_ros visualization_msgs)
target_compile_options(neupan_fleet_sim PRIVATE -O3 -Wall -Wextra -Wpedantic)

install(TARGETS neupan_sim_node neupan_fleet_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/validate.py DESTINATION lib/${PROJECT_NAME}
  RENAME neupan_validate)
install(PROGRAMS scripts/start_fleet.py DESTINATION lib/${PROJECT_NAME}
  RENAME neupan_start_fleet)
install(FILES scripts/validation_config.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_validation_config COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_validation_config.py)
  ament_add_gtest(test_fleet test/test_fleet.cpp)
  target_link_libraries(test_fleet neupan_simulation)
  ament_add_gtest(test_simulation test/test_simulation.cpp)
  target_link_libraries(test_simulation neupan_simulation)
endif()

ament_package()
