# Each module is its own target, and a module can only include what it links.
# That makes the dependency direction a build error to violate:
#   base -> math -> kinematics -> dynamics
#   hardware <- plant (the simulated robot)
#   hardware <- control (cannot see plant, so it cannot tell it is simulated)
add_subdirectory(base)
add_subdirectory(math)
add_subdirectory(kinematics)
add_subdirectory(dynamics)
add_subdirectory(hardware)
add_subdirectory(plant)
add_subdirectory(control)
