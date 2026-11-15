cmake_minimum_required(VERSION 3.28)

project(
  Stapes
  VERSION 0.1.0
  DESCRIPTION "Deterministic simulation environment for a 6-DOF surgical robot arm"
  LANGUAGES CXX)

option(STAPES_BUILD_TESTS "Build the test suite" ON)
option(STAPES_BUILD_BENCHMARKS "Build the benchmarks" ON)
option(STAPES_BUILD_PYTHON "Build the Python bindings" OFF)
option(STAPES_WERROR "Treat compiler warnings as errors" OFF)
option(STAPES_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(STAPES_COVERAGE "Instrument Stapes targets for coverage" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(STAPES_BUILD_PYTHON)
  # The core is built as static libraries, and a Python module is a shared object.
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(StapesOptions)
include(StapesDependencies)

add_subdirectory(core)

if(STAPES_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(STAPES_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

if(STAPES_BUILD_PYTHON)
  add_subdirectory(bindings)
endif()
