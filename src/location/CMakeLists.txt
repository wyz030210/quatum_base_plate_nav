cmake_minimum_required(VERSION 3.8)
project(location)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOLIB REQUIRED geographiclib)

# # 打印找到的信息用于调试
# message(STATUS "GeographicLib found: ${GeographicLib_FOUND}")
# message(STATUS "GeographicLib include dirs: ${GeographicLib_INCLUDE_DIRS}")
# message(STATUS "GeographicLib libraries: ${GeographicLib_LIBRARIES}")

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RTKData.msg"
  DEPENDENCIES std_msgs geometry_msgs
)


# 添加可执行文件
add_executable(rtk_localization_node 
  src/car_location.cpp
  src/nmea_parser.cpp
  src/serial_reader.cpp
  src/coordinate_converter.cpp
)


# 包含目录
target_include_directories(rtk_localization_node PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${GEOLIB_INCLUDE_DIRS}
)

# 链接依赖
ament_target_dependencies(rtk_localization_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
)

target_link_libraries(rtk_localization_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${GEOLIB_LIBRARIES}
)

target_compile_definitions(rtk_localization_node PRIVATE ${GeographicLib_CFLAGS_OTHER})

# 安装目标
install(TARGETS rtk_localization_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rtk_data test/test_rtk_data.cpp
    src/nmea_parser.cpp src/serial_reader.cpp)
  target_include_directories(test_rtk_data PRIVATE include)
  ament_target_dependencies(test_rtk_data rclcpp)
  target_link_libraries(test_rtk_data util)
endif()

ament_package()
