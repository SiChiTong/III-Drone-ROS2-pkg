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

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include_directories(src/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src)
link_directories(src/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_executable(hough_interfacer
  src/hough_interfacer_node.cpp
  src/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src/xhoughlines_accel_linux.c
  src/hough-accel-ip/ip/drivers/houghlines_accel_v1_0/src/xhoughlines_accel.c
)
ament_target_dependencies(hough_interfacer
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  iii_interfaces
)
install(TARGETS hough_interfacer DESTINATION lib/${PROJECT_NAME})

add_executable(pl_dir_computer
  src/pl_dir_computer_node.cpp
  src/geometry.cpp
)
ament_target_dependencies(pl_dir_computer
  rclcpp
  Eigen3
  iii_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS pl_dir_computer DESTINATION lib/${PROJECT_NAME})

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
  tf2_geometry_msgs
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
    image_transport
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()


custom_executable(img_3d_to_2d_proj)
custom_executable(hough_pub)
custom_executable(offboard_control_fly_to_line)
custom_executable(img_decompress)


add_executable(depth_cam_to_mmwave src/depth_cam_to_mmwave.cpp)
ament_target_dependencies(depth_cam_to_mmwave
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)
install(TARGETS depth_cam_to_mmwave DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/image_drawer.py
  scripts/image_decompress.py
  scripts/draw_PL_IDs.py
  DESTINATION lib/${PROJECT_NAME}
)

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

# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
