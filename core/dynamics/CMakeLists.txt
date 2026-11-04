add_library(stapes_dynamics src/dynamic_model.cpp src/rigid_body_dynamics.cpp)
add_library(stapes::dynamics ALIAS stapes_dynamics)

target_include_directories(stapes_dynamics PUBLIC include)
target_link_libraries(
  stapes_dynamics
  PUBLIC stapes::kinematics
  PRIVATE stapes::options)
