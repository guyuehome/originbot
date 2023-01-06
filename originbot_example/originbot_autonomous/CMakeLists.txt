cmake_minimum_required(VERSION 3.5)
project(originbot_autonomous)

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

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
include_directories(${QPOASES_INLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(purepursuit_node
  Purepursuit/src/originbot_purepursuit.cc
  Purepursuit/src/loadPath.cc
)

add_executable(mpc_node
  MPC/src/originbot_mpc.cc
  MPC/src/loadPath.cc
)

include_directories(MPC/include)
include_directories(Purepursuit/include)

target_link_libraries(mpc_node /usr/local/lib/libqpOASES.a)

ament_target_dependencies(
  purepursuit_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

ament_target_dependencies(
  mpc_node
  geometry_msgs
  nav_msgs
  rclcpp
  ament_index_cpp
)

install(TARGETS
  purepursuit_node
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  MPC/launch
  DESTINATION share/${PROJECT_NAME}
)



