cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(stm32_system_interface SHARED
  src/activation_safety.cpp
  src/stm32_system_interface.cpp
)
target_compile_features(stm32_system_interface PUBLIC cxx_std_17)
target_include_directories(stm32_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stm32_system_interface
  hardware_interface
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(stm32_system_interface yaml-cpp Threads::Threads)

pluginlib_export_plugin_description_file(hardware_interface arm_hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS stm32_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES arm_hardware_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch test DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(stm32_system_interface)
ament_export_dependencies(hardware_interface pluginlib rcl_interfaces rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_activation_safety test/test_activation_safety.cpp)
  if(TARGET test_activation_safety)
    target_link_libraries(test_activation_safety stm32_system_interface)
    ament_target_dependencies(test_activation_safety rclcpp)
  endif()
  # Runs on Humble and Jazzy alike: it drives the interface's own lifecycle
  # calls instead of hardware_interface::HardwareComponent, so it needs no
  # guard like test_error_transition below.
  ament_add_gtest(test_commanded_state test/test_commanded_state.cpp)
  if(TARGET test_commanded_state)
    target_link_libraries(test_commanded_state stm32_system_interface)
    ament_target_dependencies(test_commanded_state
      hardware_interface rclcpp rclcpp_lifecycle)
    target_compile_definitions(test_commanded_state PRIVATE
      ARM_HARDWARE_TEST_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/test/mock_calibration.yaml")
  endif()
  # Also distro-portable: pure conversion checks, no HardwareComponent.
  ament_add_gtest(test_calibration_mapping test/test_calibration_mapping.cpp)
  if(TARGET test_calibration_mapping)
    target_link_libraries(test_calibration_mapping stm32_system_interface)
    ament_target_dependencies(test_calibration_mapping
      hardware_interface rclcpp rclcpp_lifecycle)
    target_compile_definitions(test_calibration_mapping PRIVATE
      ARM_HARDWARE_TEST_MAPPING_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/test/mock_calibration_mapping.yaml")
  endif()
  ament_add_gtest(test_serial_ack test/test_serial_ack.cpp)
  if(TARGET test_serial_ack)
    target_link_libraries(test_serial_ack stm32_system_interface)
    ament_target_dependencies(test_serial_ack hardware_interface rclcpp rclcpp_lifecycle)
  endif()
  ament_add_gtest(test_stm32_system_interface test/test_stm32_system_interface.cpp)
  if(TARGET test_stm32_system_interface)
    target_link_libraries(test_stm32_system_interface
      stm32_system_interface
      Threads::Threads
      util
    )
    ament_target_dependencies(test_stm32_system_interface
      hardware_interface
      lifecycle_msgs
      rcl_interfaces
      rclcpp
      rclcpp_lifecycle
    )
    target_compile_definitions(test_stm32_system_interface PRIVATE
      ARM_HARDWARE_TEST_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/test/mock_calibration.yaml")
  endif()
  # test_error_transition drives the lifecycle through HardwareComponent, which
  # only exists in Jazzy-era hardware_interface. The Jetson runs Humble, where
  # that header is absent (measured 2026-07-29: system.hpp present,
  # hardware_component.hpp not). Without this guard the Nano build breaks as
  # soon as anyone enables testing there -- and the deploy path is exactly
  # where a build break costs the most.
  set(_hi_include_hints ${hardware_interface_INCLUDE_DIRS})
  if(TARGET hardware_interface::hardware_interface)
    get_target_property(_hi_target_includes
      hardware_interface::hardware_interface INTERFACE_INCLUDE_DIRECTORIES)
    if(_hi_target_includes)
      list(APPEND _hi_include_hints ${_hi_target_includes})
    endif()
  endif()
  find_file(ARM_HARDWARE_HAS_COMPONENT_API
    NAMES hardware_interface/hardware_component.hpp
    HINTS ${_hi_include_hints})
  if(ARM_HARDWARE_HAS_COMPONENT_API)
    ament_add_gtest(test_error_transition test/test_error_transition.cpp)
    if(TARGET test_error_transition)
      target_link_libraries(test_error_transition stm32_system_interface)
      ament_target_dependencies(test_error_transition
        hardware_interface rclcpp rclcpp_lifecycle)
      target_compile_definitions(test_error_transition PRIVATE
        ARM_HARDWARE_TEST_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/test/mock_calibration.yaml")
    endif()
  else()
    message(STATUS
      "arm_hardware: skipping test_error_transition -- hardware_component.hpp not found. "
      "The on_error fail-closed behaviour it covers is still compiled into the library; "
      "only its regression test is unavailable on this distro.")
  endif()
endif()

ament_package()
