cmake_minimum_required(VERSION 3.5)
project(originbot_qrcode_detect)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(originbot_bringup REQUIRED)

find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(qr_decoder src/qr_decoder.cpp)
ament_target_dependencies(qr_decoder rclcpp sensor_msgs originbot_bringup cv_bridge image_transport)

add_executable(qrcode_control src/qrcode_control.cpp)
ament_target_dependencies(qrcode_control rclcpp sensor_msgs originbot_bringup cv_bridge image_transport geometry_msgs)

# 链接zbar库
find_package(PkgConfig REQUIRED)
pkg_search_module(ZBAR REQUIRED zbar)
target_link_libraries(qr_decoder ${ZBAR_LIBRARIES})
target_include_directories(qr_decoder PUBLIC ${ZBAR_INCLUDE_DIRS})

target_link_libraries(qr_decoder 
  ${OpenCV_LIBS}
)

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

install(TARGETS
  qr_decoder
  qrcode_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()