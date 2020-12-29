cmake_minimum_required(VERSION 3.5)
project(lslidar_c16_decoder)
#add_compile_options(-std=c++11)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lslidar_c16_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(angles REQUIRED)
find_package(PkgConfig REQUIRED)

add_library(pointcloud_converter src/convert.cpp src/rawdata.cpp)
target_link_libraries(pointcloud_converter ${rclcpp_LIBRARIES})
ament_target_dependencies(pointcloud_converter rclcpp std_msgs lslidar_c16_msgs sensor_msgs pcl_conversions)

add_executable(cloud_node src/cloud_node.cpp)
target_link_libraries(cloud_node ${rclcpp_LIBRARIES} pointcloud_converter)
ament_target_dependencies(cloud_node rclcpp std_msgs lslidar_c16_msgs sensor_msgs pcl_conversions)

include_directories(
        include
        ${PCL_COMMON_INCLUDE_DIRS}
)

install(TARGETS
  cloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION
  include
)

install(DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp std_msgs lslidar_c16_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()

