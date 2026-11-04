add_library(stapes_kinematics src/arm_model.cpp src/forward_kinematics.cpp
                               src/inverse_kinematics.cpp)
add_library(stapes::kinematics ALIAS stapes_kinematics)

target_include_directories(stapes_kinematics PUBLIC include)
target_link_libraries(
  stapes_kinematics
  PUBLIC stapes::base stapes::math
  PRIVATE stapes::options)
