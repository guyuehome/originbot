cmake_minimum_required(VERSION 3.5)
project(originbot_teleop)

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
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread program_options)

if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost library not found")
endif()

add_executable(originbot_teleop src/originbot_teleop.cc)
ament_target_dependencies(originbot_teleop rclcpp geometry_msgs)
target_include_directories(originbot_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(originbot_teleop ${Boost_LIBRARIES})

install(TARGETS originbot_teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()