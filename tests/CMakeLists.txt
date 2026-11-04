add_library(stapes_test_support INTERFACE)
target_include_directories(stapes_test_support INTERFACE support)
target_link_libraries(stapes_test_support INTERFACE stapes::base stapes::kinematics
                                                     GTest::gtest stapes::options)

# Each binary writes its results as JSON, which tools/traceability.py reads to tie
# every requirement to the tests that verify it.
function(stapes_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE stapes_test_support GTest::gtest_main)
  add_test(NAME ${name}
           COMMAND ${name} --gtest_output=json:${CMAKE_BINARY_DIR}/test-results/${name}.json)
endfunction()

stapes_add_test(test_base unit/base/test_random.cpp unit/base/test_sim_clock.cpp
                 unit/base/test_state_hash.cpp)
stapes_add_test(test_math unit/math/test_rotation.cpp)
stapes_add_test(test_kinematics unit/kinematics/test_arm_model.cpp
                 unit/kinematics/test_forward_kinematics.cpp
                 unit/kinematics/test_inverse_kinematics.cpp)
# This test replaces the global operator new to count calls, and ASan has to replace it
# too; the two conflict. Sanitizer builds check memory safety, and every other build
# still runs this test.
if(NOT STAPES_SANITIZE)
  stapes_add_test(test_realtime unit/realtime/test_no_allocation.cpp)
endif()
stapes_add_test(test_determinism unit/determinism/test_same_seed.cpp)

# In-process tests cannot catch what changes between processes, such as reads of
# uninitialised memory or a dependence on addresses. This runs a workload in two
# separate processes and compares the hashes.
add_executable(determinism_probe determinism/determinism_probe.cpp)
target_link_libraries(determinism_probe PRIVATE stapes_test_support)
add_test(NAME determinism_across_processes
         COMMAND ${CMAKE_COMMAND} -DPROBE=$<TARGET_FILE:determinism_probe> -P
                 ${CMAKE_CURRENT_SOURCE_DIR}/determinism/compare_runs.cmake)
