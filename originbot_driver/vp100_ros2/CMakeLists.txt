cmake_minimum_required(VERSION 3.5)
project(vp100_ros2 C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/src/impl/include
  ${PROJECT_SOURCE_DIR}/sdk/src/nvilidar
)

#---------------------------------------------------------------------------------------
# Generate executable and add libraries
#---------------------------------------------------------------------------------------
add_executable(vp100_ros2_node
    src/vp100_ros2_node.cpp
    sdk/src/impl/src/serial/nvilidar_serial_unix.cpp
    sdk/src/impl/src/serial/nvilidar_serial_win.cpp
    sdk/src/nvilidar/nvilidar_driver_serialport.cpp
    sdk/src/nvilidar/nvilidar_process.cpp
    sdk/src/nvilidar/nvilidar_filter.cpp
)

# Link libraries
ament_target_dependencies(vp100_ros2_node
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
)

#---------------------------------------------------------------------------------------
# Generate executable and add libraries
#---------------------------------------------------------------------------------------
add_executable(vp100_ros2_client
    src/vp100_ros2_client.cpp
    sdk/src/impl/src/serial/nvilidar_serial_unix.cpp
    sdk/src/impl/src/serial/nvilidar_serial_win.cpp
    sdk/src/nvilidar/nvilidar_driver_serialport.cpp
    sdk/src/nvilidar/nvilidar_process.cpp
    sdk/src/nvilidar/nvilidar_filter.cpp
)

# Link libraries
ament_target_dependencies(vp100_ros2_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    vp100_ros2_node
    vp100_ros2_client
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params startup
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()