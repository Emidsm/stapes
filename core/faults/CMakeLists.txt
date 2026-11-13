add_library(stapes_faults src/fault_injector.cpp)
add_library(stapes::faults ALIAS stapes_faults)

target_include_directories(stapes_faults PUBLIC include)
target_link_libraries(
  stapes_faults
  PUBLIC stapes::hardware
  PRIVATE stapes::options)
