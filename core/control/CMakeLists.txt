add_library(stapes_control src/joint_controller.cpp src/pid_controller.cpp)
add_library(stapes::control ALIAS stapes_control)

target_include_directories(stapes_control PUBLIC include)
target_link_libraries(
  stapes_control
  PUBLIC stapes::dynamics stapes::hardware
  PRIVATE stapes::options)
