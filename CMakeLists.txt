cmake_minimum_required(VERSION 3.5)

project(cspc_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/cspc_lidar
  ${PROJECT_SOURCE_DIR}/sdk
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cspc_lidar sdk/mtime.cpp sdk/node_lidar.cpp sdk/serial_port.cpp sdk/timer.cpp sdk/lidar_data_processing.cpp sdk/point_cloud_optimize.cpp sdk/calibration.cpp src/node_lidar_ros.cpp)

ament_target_dependencies(cspc_lidar
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
)

install(TARGETS
  cspc_lidar
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
