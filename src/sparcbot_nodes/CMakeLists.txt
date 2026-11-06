cmake_minimum_required(VERSION 3.16)
project(sparcbot_nodes LANGUAGES CXX)

# Thin ROS 2 wrappers around sparcbot_core. Only ROS plumbing lives here:
# parameters, publishers/subscribers, services, timers.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sparcbot_interfaces REQUIRED)
find_package(sparcbot_core REQUIRED)

add_executable(sparcbot_driver_node src/driver_node.cpp)
target_link_libraries(sparcbot_driver_node
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${sparcbot_interfaces_TARGETS}
  sparcbot_core::sparcbot_core
)

add_executable(diff_drive_node src/diff_drive_node.cpp)
target_link_libraries(diff_drive_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sparcbot_interfaces_TARGETS}
  sparcbot_core::sparcbot_core
)

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sparcbot_interfaces_TARGETS}
  tf2_ros::tf2_ros
  sparcbot_core::sparcbot_core
)

foreach(target sparcbot_driver_node diff_drive_node odometry_node)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS sparcbot_driver_node diff_drive_node odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
