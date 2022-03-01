cmake_minimum_required(VERSION 3.5)
project(iii_drone)

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

include_directories(include/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src)
link_directories(include/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(hough_interfacer
  src/hough_interfacer_node.cpp
  include/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src/xhoughlines_accel_linux.c
  include/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src/xhoughlines_accel.c
)
ament_target_dependencies(hough_interfacer
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  iii_interfaces
)
install(TARGETS hough_interfacer DESTINATION lib/${PROJECT_NAME})

add_executable(pl_mapper
  src/pl_mapper_node.cpp
  src/geometry.cpp
  src/powerline_class.cpp
  src/single_line_class.cpp
)
ament_target_dependencies(pl_mapper
  rclcpp
  Eigen3
  iii_interfaces
  sensor_msgs
  tf2
  tf2_ros
)
install(TARGETS pl_mapper DESTINATION lib/${PROJECT_NAME})

add_executable(drone_frame_broadcaster
  src/drone_frame_broadcaster_node.cpp
  src/geometry.cpp
)
ament_target_dependencies(drone_frame_broadcaster
  rclcpp
  Eigen3
  geometry_msgs
  px4_msgs
  tf2
  tf2_ros
)
install(TARGETS drone_frame_broadcaster DESTINATION lib/${PROJECT_NAME})

add_executable(logger
  src/logger_node.cpp
  src/geometry.cpp
)
ament_target_dependencies(logger
  rclcpp
  geometry_msgs
  sensor_msgs
  px4_msgs
  tf2
  tf2_ros
  OpenCV
  cv_bridge
  iii_interfaces
)
install(TARGETS logger DESTINATION lib/${PROJECT_NAME})

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    rclcpp
    px4_msgs
    sensor_msgs
    std_msgs
    OpenCV
    cv_bridge
    iii_interfaces
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()


# Add examples
custom_executable(offboard_control)
custom_executable(vel_ctrl_vec_pub)
custom_executable(img_3d_to_2d_proj)
custom_executable(lidar_to_mmwave)
custom_executable(hough_pub)


############
# Install ##
############

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs OpenCV)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
