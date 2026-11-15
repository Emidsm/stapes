add_library(stapes_test_support INTERFACE)
target_include_directories(stapes_test_support INTERFACE support)
target_link_libraries(stapes_test_support INTERFACE stapes::base stapes::kinematics
                                                    stapes::dynamics stapes::plant stapes::control
                                                    stapes::sim GTest::gtest stapes::options)

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

stapes_add_test(test_dynamics unit/dynamics/test_dynamic_model.cpp
                unit/dynamics/test_rigid_body_dynamics.cpp)
stapes_add_test(test_plant unit/plant/test_actuator.cpp unit/plant/test_encoder.cpp
                unit/plant/test_force_sensor.cpp unit/plant/test_optical_tracker.cpp
                unit/plant/test_safety_channel.cpp unit/plant/test_simulated_arm.cpp)
stapes_add_test(test_control unit/control/test_pid_controller.cpp
                unit/control/test_joint_controller.cpp)
stapes_add_test(test_closed_loop integration/test_closed_loop.cpp)

stapes_add_test(test_task unit/task/test_keyhole.cpp)
stapes_add_test(test_faults unit/faults/test_fault_injector.cpp)
stapes_add_test(test_safety unit/safety/test_safety_monitor.cpp)
target_compile_definitions(test_safety PRIVATE STAPES_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
stapes_add_test(test_sim unit/sim/test_trace.cpp)
stapes_add_test(test_keyhole integration/test_keyhole.cpp)
stapes_add_test(test_fault_campaign integration/test_fault_campaign.cpp)

# In-process tests cannot catch what changes between processes, such as reads of
# uninitialised memory or a dependence on addresses. This runs a workload in two
# separate processes and compares the hashes.
add_executable(determinism_probe determinism/determinism_probe.cpp)
target_link_libraries(determinism_probe PRIVATE stapes_test_support)
add_test(NAME determinism_across_processes
         COMMAND ${CMAKE_COMMAND} -DPROBE=$<TARGET_FILE:determinism_probe> -P
                 ${CMAKE_CURRENT_SOURCE_DIR}/determinism/compare_runs.cmake)
