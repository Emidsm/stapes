# The Python layer: scenarios and analysis. It depends on the core; the core never depends on
# it, and no preset other than `python` configures this directory.
#
# The module is a thin translation of types. Anything with behaviour belongs in `core`, where
# the C++ tests and the mutation check can reach it.
pybind11_add_module(stapes_core MODULE src/stapes_core.cpp)
target_link_libraries(stapes_core PRIVATE stapes::sim stapes::options)
target_compile_definitions(stapes_core PRIVATE STAPES_VERSION="${PROJECT_VERSION}")
set_target_properties(stapes_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                             "${CMAKE_BINARY_DIR}/python")

if(STAPES_BUILD_TESTS)
  # Runs under ctest with the C++ suites, and writes its results in the same JSON layout, so
  # one traceability matrix covers both languages.
  add_test(
    NAME test_python
    COMMAND
      ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_BINARY_DIR}/python:${PROJECT_SOURCE_DIR}/python"
      "STAPES_DETERMINISM_PROBE=$<TARGET_FILE:determinism_probe>"
      "STAPES_TEST_RESULTS=${CMAKE_BINARY_DIR}/test-results/test_python.json"
      ${Python_EXECUTABLE} -m pytest ${PROJECT_SOURCE_DIR}/python/tests -q -p no:cacheprovider)
endif()
