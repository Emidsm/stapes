add_library(stapes_plant src/actuator.cpp src/encoder.cpp src/force_sensor.cpp
                         src/optical_tracker.cpp src/simulated_arm.cpp)
add_library(stapes::plant ALIAS stapes_plant)

target_include_directories(stapes_plant PUBLIC include)
target_link_libraries(
  stapes_plant
  PUBLIC stapes::base stapes::dynamics stapes::hardware
  PRIVATE stapes::options)
