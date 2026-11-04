add_library(stapes_math src/rotation.cpp)
add_library(stapes::math ALIAS stapes_math)

target_include_directories(stapes_math PUBLIC include)
target_link_libraries(
  stapes_math
  PUBLIC stapes::eigen
  PRIVATE stapes::options)
