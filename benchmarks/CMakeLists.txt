add_executable(bench_kinematics bench_kinematics.cpp)
target_link_libraries(bench_kinematics PRIVATE stapes::kinematics benchmark::benchmark
                                               stapes::options)

add_executable(bench_simulation bench_simulation.cpp)
target_link_libraries(bench_simulation PRIVATE stapes::plant stapes::control benchmark::benchmark
                                               stapes::options)
