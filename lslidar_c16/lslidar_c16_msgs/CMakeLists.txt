cmake_minimum_required(VERSION 3.5)
project(lslidar_c16_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(lslidar_c16_msgs
  "msg/LslidarC16Layer.msg"
  "msg/LslidarC16Packet.msg"
  "msg/LslidarC16Point.msg"
  "msg/LslidarC16Scan.msg"
  "msg/LslidarC16Sweep.msg"
  "msg/LslidarC16ScanUnified.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  )

ament_package()
