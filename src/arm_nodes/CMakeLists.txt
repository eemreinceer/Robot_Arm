cmake_minimum_required(VERSION 3.8)
project(arm_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(ompl REQUIRED)

include_directories(include)

# OMPL deterministic-seed preload library. LD_PRELOADed into move_group (see
# arm_moveit_config/launch/move_group.launch.py); its constructor pins the OMPL
# RNG seed at process start, before OMPL creates any generator, so planning is
# reproducible run-to-run. See src/ompl_seed_preload.cpp.
add_library(ompl_seed_preload SHARED src/ompl_seed_preload.cpp)
target_include_directories(ompl_seed_preload PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(ompl_seed_preload ${OMPL_LIBRARIES})

add_library(gripper_controller src/gripper_controller.cpp)
ament_target_dependencies(gripper_controller
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(pick_place_node src/pick_place_node.cpp)
target_link_libraries(pick_place_node gripper_controller)
ament_target_dependencies(pick_place_node
  rclcpp
  rclcpp_action
  geometry_msgs
  shape_msgs
  control_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  arm_interfaces
)

add_executable(robot_arm_top_down_pick_demo src/robot_arm_top_down_pick_demo.cpp)
ament_target_dependencies(robot_arm_top_down_pick_demo
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
)

add_executable(robot_arm_pick_place_node src/robot_arm_pick_place_node.cpp)
ament_target_dependencies(robot_arm_pick_place_node
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  arm_interfaces
)

install(TARGETS
  gripper_controller
  pick_place_node
  robot_arm_top_down_pick_demo
  robot_arm_pick_place_node
  ompl_seed_preload
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/demo_pick_place.py
  scripts/analyze_pwm_capture.py
  scripts/analyze_rail_dump.py
  scripts/measurement_analysis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    measurement_analysis
    test/test_measurement_analysis.py
    TIMEOUT 30
  )
endif()

ament_package()
