add_library(stapes_hardware INTERFACE)
add_library(stapes::hardware ALIAS stapes_hardware)

target_include_directories(stapes_hardware INTERFACE include)
target_link_libraries(stapes_hardware INTERFACE stapes::kinematics)
