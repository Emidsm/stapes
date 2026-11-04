# Each module is its own target, and a module can only include what it links.
# That makes the dependency direction a build error to violate:
#   base -> math -> kinematics
add_subdirectory(base)
add_subdirectory(math)
add_subdirectory(kinematics)
