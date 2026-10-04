cmake_minimum_required(VERSION 3.8)
project(arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build library
add_library(arm_kinematics_lib
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
  src/jacobian.cpp
)
target_link_libraries(arm_kinematics_lib
  Eigen3::Eigen
)
ament_target_dependencies(arm_kinematics_lib
  rclcpp
  geometry_msgs
  arm_interfaces
)

# Build service node executable
add_executable(kinematics_node
  src/kinematics_node.cpp
)
target_link_libraries(kinematics_node
  arm_kinematics_lib
)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  arm_interfaces
)

# Install targets
install(
  TARGETS arm_kinematics_lib kinematics_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/arm_kinematics
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependency interfaces
ament_export_include_directories(include)
ament_export_libraries(arm_kinematics_lib)
ament_export_dependencies(rclcpp geometry_msgs arm_interfaces Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
