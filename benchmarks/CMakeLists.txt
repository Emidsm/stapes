add_executable(bench_kinematics bench_kinematics.cpp)
target_link_libraries(bench_kinematics PRIVATE stapes::kinematics benchmark::benchmark
                                               stapes::options)
