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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

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
  px4_msgs
)

install(TARGETS
  pl_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
