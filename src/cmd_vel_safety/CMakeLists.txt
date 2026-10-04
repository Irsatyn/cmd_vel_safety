cmake_minimum_required(VERSION 3.8)
project(cmd_vel_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cmd_vel_safety_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Safety pipeline as a standalone library with no ROS dependency, so that the
# algorithm can be unit tested without spinning up a ROS graph.
# ---------------------------------------------------------------------------
add_library(safety_limiter STATIC src/safety_limiter.cpp)
target_include_directories(safety_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_property(TARGET safety_limiter PROPERTY POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------
add_executable(velocity_guard src/velocity_guard_node.cpp)
target_link_libraries(velocity_guard safety_limiter)
ament_target_dependencies(velocity_guard
  rclcpp std_msgs geometry_msgs rcl_interfaces cmd_vel_safety_msgs)

add_executable(motion_state_monitor src/motion_state_monitor_node.cpp)
target_link_libraries(motion_state_monitor safety_limiter)
ament_target_dependencies(motion_state_monitor
  rclcpp std_msgs geometry_msgs nav_msgs diagnostic_msgs rcl_interfaces
  cmd_vel_safety_msgs)

add_executable(virtual_robot src/virtual_robot_node.cpp)
ament_target_dependencies(virtual_robot
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS velocity_guard motion_state_monitor virtual_robot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config bags
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/check_bag.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_limiter test/test_safety_limiter.cpp)
  target_link_libraries(test_safety_limiter safety_limiter)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_check_bag test/test_check_bag.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_node_contracts.py TIMEOUT 180)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
