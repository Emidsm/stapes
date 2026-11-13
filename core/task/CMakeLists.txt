add_library(stapes_task src/keyhole.cpp)
add_library(stapes::task ALIAS stapes_task)

target_include_directories(stapes_task PUBLIC include)
target_link_libraries(
  stapes_task
  PUBLIC stapes::kinematics
  PRIVATE stapes::options)
