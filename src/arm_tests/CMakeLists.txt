cmake_minimum_required(VERSION 3.8)
project(arm_tests)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Python test scriptlerini install et
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR integration
)

# Install programs so they can be run via ros2 run
install(PROGRAMS
  integration/test_moveit_planning.py
  integration/test_controllers.py
  integration/test_pick_place_e2e.py
  integration/test_gripper.py
  integration/test_system_health.py
  unit/test_forward_kinematics.py
  unit/test_inverse_kinematics.py
  unit/test_dl_ik_robustness.py
  unit/test_yolo_detection.py
  unit/test_3d_pose_estimation.py
  integration/test_full_pipeline.py
  benchmarks/benchmark_kinematics.py
  benchmarks/benchmark_ik_comparison.py
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies (GTest)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # Unit Test 1: test_interfaces
  ament_add_gtest(test_interfaces unit/test_interfaces.cpp)
  if(TARGET test_interfaces)
    ament_target_dependencies(test_interfaces
      rclcpp
      arm_interfaces
    )
  endif()

  # Unit Test 2: test_kinematics
  ament_add_gtest(test_kinematics unit/test_kinematics.cpp)
  if(TARGET test_kinematics)
    ament_target_dependencies(test_kinematics
      rclcpp
      arm_interfaces
      sensor_msgs
      geometry_msgs
    )
  endif()

  # The M3 shutdown contract is only observable with debuginfod disabled: a
  # crashing move_group otherwise blocks in libdw talking to
  # debuginfod.ubuntu.com and the launch never reaches its post-shutdown phase.
  # Ubuntu populates DEBUGINFOD_URLS from /etc/profile.d, so the test cannot
  # rely on the caller's shell. See docs/m3_shutdown_policy.md.
  add_launch_test(integration/test_m3_launch_smoke.py
    TARGET test_m3_launch_smoke
    TIMEOUT 60
  )
  set_property(TEST test_m3_launch_smoke APPEND PROPERTY ENVIRONMENT "DEBUGINFOD_URLS=")
  ament_add_pytest_test(test_m3_negative
    integration/test_m3_negative.py
    TIMEOUT 45
  )
  ament_add_pytest_test(test_hand_eye_pose_search
    unit/test_hand_eye_pose_search.py
    TIMEOUT 45
  )
  ament_add_pytest_test(test_camera_first_frame_watchdog
    unit/test_camera_first_frame_watchdog.py
    TIMEOUT 30
  )

  # Issue #14 ClassLoader nesne ömrü regresyonu. Bilinçli olarak M3 smoke
  # kaydından uzağa konuldu: o bölge başka test değişikliklerinde yeniden
  # yazılıyor ve bitişik olsaydı her merge'de çakışırdı.
  # Bu test DEBUGINFOD_URLS'i kendi içinde sabitlediği için CTest ENV'i gerekmiyor.
  add_launch_test(integration/test_pick_place_shutdown_clean.py
    TARGET test_pick_place_shutdown_clean
    TIMEOUT 120
  )
endif()

# Install directories
install(DIRECTORY
  launch
  integration
  benchmarks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
