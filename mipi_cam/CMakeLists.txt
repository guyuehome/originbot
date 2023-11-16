# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(mipi_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-w)

add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include
  src/x3
  src/x3_sdk_swap/include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
)

## Build the mipi camera library
add_executable(${PROJECT_NAME}
  src/camera.cpp
  src/mipi_cam_node.cpp
  src/mipi_cam.cpp
  src/mipi_cap.cpp
#  src/x3/*.c
  src/x3/sensor_f37_config.c
  src/x3/sensor_imx415_config.c
  src/x3/sensor_gc4663_config.c
  src/x3/sensor_imx586_config.c
  src/x3/sensor_gc4c33_config.c
  src/x3/sensor_imx219_config.c
  src/x3/sensor_imx477_config.c
  src/x3/sensor_ov5647_config.c
  src/x3/sensor_sc132gs_config.c 
  src/x3/x3_preparam.c
  src/x3/x3_utils.c
  src/x3_sdk_swap/src/x3_sdk_wrap.c
  src/x3_sdk_swap/src/x3_vio_bind.c
  src/x3_sdk_swap/src/x3_vio_rgn.c
  src/x3_sdk_swap/src/x3_vio_utils.c
  src/x3_sdk_swap/src/x3_vio_vdec.c
  src/x3_sdk_swap/src/x3_vio_vin.c
  src/x3_sdk_swap/src/x3_vio_vot.c
  src/x3_sdk_swap/src/x3_vio_vps.c
  src/x3_sdk_swap/src/x3_vio_venc.cpp
  src/x3_sdk_swap/src/x3_vio_vp.cpp
)

set(BASE_LIBRARIES pthread vio cam dl)
set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson diag gdcbin)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES} ${X3_HAPI_LIB} yaml-cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
  )

target_link_libraries(${PROJECT_NAME}
  hbmem
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  hbm_img_msgs
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
