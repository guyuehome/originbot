cmake_minimum_required(VERSION 3.5)
project(utils)

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
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)

link_directories(
  /opt/tros/lib/
)

add_executable(image_transport_node
  src/image_transport_node.cpp
)

ament_target_dependencies(
  image_transport_node
  rclcpp
  sensor_msgs
  cv_bridge
  hbm_img_msgs
)

# Install executables
install(
  TARGETS image_transport_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()