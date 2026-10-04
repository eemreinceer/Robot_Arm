cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/ObjectPose.msg"
  "msg/ObjectArray.msg"
  "msg/ArmStatus.msg"
  "msg/BoardObservation.msg"
  # Services
  "srv/SpawnObject.srv"
  "srv/GetPickPose.srv"
  "srv/SolveFk.srv"
  "srv/SolveIk.srv"
  # Actions
  "action/PickAndPlace.action"
  "action/MoveToPose.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
