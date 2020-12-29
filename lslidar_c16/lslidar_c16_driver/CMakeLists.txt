cmake_minimum_required(VERSION 3.5)
project(lslidar_c16_driver)

#add_compile_options(-std=c++11)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)       
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(angles REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lslidar_c16_decoder REQUIRED)
find_package(PkgConfig REQUIRED)

# Node
add_executable(lslidar_node src/lslidar_c16_driver_node.cpp src/lslidar_c16_driver.cpp src/input.cc)
target_link_libraries(lslidar_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(lslidar_node rclcpp std_msgs lslidar_c16_msgs sensor_msgs diagnostic_updater)

# Composition (driver + pointcloud)
add_executable(lslidar_c16_executor src/lslidar_c16_executor.cc src/lslidar_c16_driver.cpp src/input.cc)
target_link_libraries(lslidar_c16_executor ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(lslidar_c16_executor rclcpp std_msgs lslidar_c16_msgs sensor_msgs diagnostic_updater lslidar_c16_decoder)
#ament_target_dependencies(lslidar_c16_executor rclcpp std_msgs lslidar_c16_msgs sensor_msgs diagnostic_updater)

include_directories(
		include
)

install(TARGETS
  lslidar_node
  lslidar_c16_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs lslidar_c16_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()
