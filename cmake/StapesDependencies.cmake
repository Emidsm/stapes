# Third-party code, pinned by version and hash so every build uses the same bytes.
# SYSTEM keeps warnings from these headers out of Stapes's warnings-as-errors build.

include(FetchContent)

# Linear algebra. Header-only, so CMake only needs the include path; SOURCE_SUBDIR
# points at nothing to skip Eigen's own CMake project.
FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
  URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
  SOURCE_SUBDIR do-not-configure
  SYSTEM)
FetchContent_MakeAvailable(eigen)

add_library(stapes_eigen INTERFACE)
add_library(stapes::eigen ALIAS stapes_eigen)
target_include_directories(stapes_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")

if(STAPES_BUILD_TESTS)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    SYSTEM
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(googletest)
endif()

if(STAPES_BUILD_BENCHMARKS)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_WERROR OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    benchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.5.tar.gz
    URL_HASH SHA256=9631341c82bac4a288bef951f8b26b41f69021794184ece969f8473977eaa340
    SYSTEM
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(benchmark)
endif()

if(STAPES_BUILD_PYTHON)
  # Finds the interpreter that is first on the path, so an activated virtual environment wins.
  set(Python_FIND_VIRTUALENV FIRST)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  set(PYBIND11_FINDPYTHON ON)
  FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v3.0.4.tar.gz
    URL_HASH SHA256=74b6a2c2b4573a400cafb6ecbf60c98df300cd3d0041296b913d02b2cbbb2676
    SYSTEM
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(pybind11)
endif()
