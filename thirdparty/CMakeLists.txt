cmake_minimum_required(VERSION 3.16)
project(neupan_thirdparty LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 3.3 REQUIRED)

# OSQP v1.0.0 declares QDLDL through FetchContent even though its version is
# fixed upstream. Redirect that declaration to the vendored source and reject
# every network fallback.
set(FETCHCONTENT_SOURCE_DIR_QDLDL "${CMAKE_CURRENT_SOURCE_DIR}/qdldl"
    CACHE PATH "Vendored QDLDL source" FORCE)
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL
    "Disallow third-party downloads while configuring" FORCE)

# NeuPAN uses one deterministic, CPU-only solver stack. Keep the upstream
# sources unmodified and constrain their options from this superbuild.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build third-party shared libraries" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build third-party tests" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "Build OSQP shared library" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "Build OSQP static library" FORCE)
set(OSQP_VERSION "1.0.0" CACHE STRING "Bundled OSQP version" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "Build OSQP demo" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "Build OSQP tests" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "Enable OSQP code generation" FORCE)
set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "Enable OSQP derivatives" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "Enable OSQP signal handling" FORCE)
set(OSQP_ENABLE_PRINTING ON CACHE BOOL "Enable OSQP diagnostic output" FORCE)
set(OSQP_ENABLE_PROFILING OFF CACHE BOOL "Enable OSQP profiling" FORCE)
set(OSQP_USE_LONG OFF CACHE BOOL "Use 64-bit OSQP indices" FORCE)
set(QDLDL_LONG OFF CACHE BOOL "Use 64-bit QDLDL indices" FORCE)

add_subdirectory(osqp)

# osqp creates the namespace only for its installed package. osqp-eigen also
# supports an existing in-tree target, so expose the same name in this build.
if(NOT TARGET osqp::osqpstatic)
  add_library(osqp::osqpstatic ALIAS osqpstatic)
endif()

set(OSQP_EIGEN_OSQP_TARGET_TO_LINK osqp::osqpstatic CACHE STRING
    "OSQP target used by osqp-eigen" FORCE)
set(OSQP_IS_V1 ON CACHE BOOL "Bundled OSQP uses the v1 API" FORCE)
set(OSQP_IS_V1_FINAL ON CACHE BOOL "Bundled OSQP uses the final v1 API" FORCE)
set(OSQP_EIGEN_DEBUG_OUTPUT OFF CACHE BOOL "Enable osqp-eigen debug output" FORCE)
set(ENABLE_RPATH OFF CACHE BOOL "Enable osqp-eigen install RPATH" FORCE)

add_subdirectory(osqp-eigen)
