cmake_minimum_required(VERSION 3.10)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_indoor_launch test/test_indoor_launch.py)
  ament_add_pytest_test(test_outdoor_launch test/test_outdoor_launch.py)
  ament_add_pytest_test(test_task_vehicle_launch test/test_task_vehicle_launch.py)
endif()

ament_package()
