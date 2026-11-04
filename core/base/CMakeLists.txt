add_library(stapes_base src/random.cpp src/sim_clock.cpp)
add_library(stapes::base ALIAS stapes_base)

target_include_directories(stapes_base PUBLIC include)
target_link_libraries(stapes_base PRIVATE stapes::options)
