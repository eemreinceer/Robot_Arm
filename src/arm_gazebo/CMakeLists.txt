cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  worlds
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/spawn_object.py
  scripts/sim_grasp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
