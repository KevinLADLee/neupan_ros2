cmake_minimum_required(VERSION 3.16)
project(neupan_core VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_TESTING "Build unit tests" OFF)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(NEUPAN_NATIVE "Enable -march=native tuning" OFF)
option(NEUPAN_BUILD_TOOLS "Build benchmarks and developer tools" OFF)

find_package(Eigen3 3.3 REQUIRED)
find_package(osqp 1.0.0 EXACT REQUIRED)
find_package(OsqpEigen 0.11.2 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NEUPAN_NATIVE)
  message(WARNING "NEUPAN_NATIVE requires an osqp-eigen built with the same "
                  "-march flags; use with care.")
  add_compile_options(-march=native -mtune=native)
endif()

add_library(neupan
  src/mlp.cpp
  src/robot.cpp
  src/dune.cpp
  src/nrmp.cpp
  src/initial_path.cpp
  src/obstacle_prediction.cpp
  src/pan.cpp
  src/neupan_planner.cpp
  src/tensor_io.cpp
)
add_library(neupan::neupan ALIAS neupan)

target_include_directories(neupan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(neupan PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen
                                    yaml-cpp)
target_compile_options(neupan PRIVATE -O3)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  foreach(t test_dune test_nrmp test_equivalence test_replay)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE neupan GTest::gtest GTest::gtest_main)
    target_compile_definitions(${t} PRIVATE
      NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
      NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

if(NEUPAN_BUILD_TOOLS)
  add_executable(neupan_verify_model tools/verify_model.cpp)
  target_link_libraries(neupan_verify_model PRIVATE neupan)
  install(TARGETS neupan_verify_model RUNTIME DESTINATION bin)

  add_executable(neupan_bench tools/bench.cpp)
  target_link_libraries(neupan_bench PRIVATE neupan)
  target_compile_definitions(neupan_bench PRIVATE
    NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
    NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")

  add_executable(neupan_ab_safety_margin tools/ab_safety_margin.cpp)
  target_link_libraries(neupan_ab_safety_margin PRIVATE neupan)
  if(BUILD_TESTING)
    # Asserting tool: config A must be rejected, config B must keep passing.
    add_test(NAME ab_safety_margin COMMAND neupan_ab_safety_margin
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/ab_configs/season_d0.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/ab_configs/upstream_margins.yaml
      ${CMAKE_CURRENT_SOURCE_DIR}/models/diff_sentry.bin)
  endif()
endif()

install(TARGETS neupan EXPORT neupanTargets)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION share/neupan_core/models)

install(FILES package.xml DESTINATION share/neupan_core)

install(EXPORT neupanTargets
  FILE neupan_coreTargets.cmake
  NAMESPACE neupan::
  DESTINATION lib/cmake/neupan_core)

# Export the installed models directory so dependents (e.g. neupan_ros)
# can vendor the DUNE model into their own share.
include(CMakePackageConfigHelpers)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/neupan_coreConfig.cmake" "\
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(osqp 1.0.0 EXACT)
find_dependency(OsqpEigen 0.11.2 EXACT)
find_dependency(yaml-cpp)
get_filename_component(neupan_core_MODELS_DIR
  \"\${CMAKE_CURRENT_LIST_DIR}/../../../share/neupan_core/models\" ABSOLUTE)
include(\"\${CMAKE_CURRENT_LIST_DIR}/neupan_coreTargets.cmake\")
")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/neupan_coreConfig.cmake"
  DESTINATION lib/cmake/neupan_core)
