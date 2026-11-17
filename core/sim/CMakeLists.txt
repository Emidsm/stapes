# The only module that sees both sides of the hardware boundary, because it assembles them.
add_library(stapes_sim src/keyhole_simulation.cpp src/jog_simulation.cpp src/planning.cpp
                       src/trace.cpp)
add_library(stapes::sim ALIAS stapes_sim)

target_include_directories(stapes_sim PUBLIC include)
target_link_libraries(
  stapes_sim
  PUBLIC stapes::control stapes::faults stapes::plant stapes::safety stapes::task
  PRIVATE stapes::options)
