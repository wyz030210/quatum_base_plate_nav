cmake_minimum_required(VERSION 3.8)
project(vehicle_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- message generation -----------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  DEPENDENCIES std_msgs
)

# --- executables -------------------------------------------------------------
# Task 1 (this file) stood up the ament/rosidl skeleton. Planned executables
# (wired up in later tasks, one per task):
#   can_bridge         -> src/can_bridge_node.cpp       (Task 3, done)
#   cmd_vel_to_can      -> src/cmd_vel_to_can_node.cpp  (Task 4, done)
#   speed_query         -> src/speed_query_node.cpp     (Task 5, done)
#
# When a C++ target in THIS package needs the Frame message generated above,
# Humble requires linking against this package's own generated typesupport
# target (rather than depending on it like an external package).

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(can_bridge src/can_bridge_node.cpp)
target_include_directories(can_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(can_bridge rclcpp std_msgs geometry_msgs)
target_link_libraries(can_bridge "${cpp_typesupport_target}")
install(TARGETS can_bridge DESTINATION lib/${PROJECT_NAME})

add_executable(cmd_vel_to_can src/cmd_vel_to_can_node.cpp)
target_include_directories(cmd_vel_to_can PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(cmd_vel_to_can rclcpp std_msgs geometry_msgs nav_msgs std_srvs)
target_link_libraries(cmd_vel_to_can "${cpp_typesupport_target}")
install(TARGETS cmd_vel_to_can DESTINATION lib/${PROJECT_NAME})

add_executable(speed_query src/speed_query_node.cpp)
target_include_directories(speed_query PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(speed_query rclcpp std_msgs)
target_link_libraries(speed_query "${cpp_typesupport_target}")
install(TARGETS speed_query DESTINATION lib/${PROJECT_NAME})

# keyboard_control -> scripts/keyboard_control.py (Task 8, rospy -> rclpy port)
install(PROGRAMS
  scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ackermann_steering test/test_ackermann_steering.cpp)
  target_include_directories(test_ackermann_steering PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_add_pytest_test(test_command_safety test/test_command_safety.py
    TIMEOUT 90
    ENV "COMMAND_ENTRY_EXECUTABLE=$<TARGET_FILE:cmd_vel_to_can>"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py")
  set_tests_properties(test_command_safety PROPERTIES RUN_SERIAL TRUE)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
