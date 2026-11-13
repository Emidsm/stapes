# Links neither `control` nor `plant`. The monitor cannot read the controller's state or the
# simulator's true state, because the build gives it no way to reach them.
add_library(stapes_safety src/reference_config.cpp src/safety_monitor.cpp)
add_library(stapes::safety ALIAS stapes_safety)

target_include_directories(stapes_safety PUBLIC include)
target_link_libraries(
  stapes_safety
  PUBLIC stapes::hardware
  PRIVATE stapes::options)
