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
project(hobot_audio)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(audio_msg REQUIRED) 

include_directories(include
${PROJECT_SOURCE_DIR}
)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(
  ${SYS_ROOT}/usr/include/
)

link_directories(
  ${PROJECT_SOURCE_DIR}/include/horizon_speech_sdk/lib
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/lib/hobot/hobotlog
)

add_executable(hobot_audio
  src/utils/alsa_device.cpp
  src/audio_capture/hb_audio_capture.cpp
  src/audio_engine/audioengine.cpp
  src/audio_example.cpp
)

set(BASE_LIBRARIES pthread dl rt m)
set(X3_HAPI_LIB  hbmedia cam isp vio multimedia iar)
set(X3_AUDIO_LIB asound dynamicwfst log_manage cnn_intf ion hisf hrsc)

target_link_libraries(${PROJECT_NAME}
   ${X3_AUDIO_LIB} ${X3_HAPI_LIB} ${BASE_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
"rclcpp"
"geometry_msgs"
"audio_msg")

ament_target_dependencies(${PROJECT_NAME} audio_msg)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install libraries
install(DIRECTORY include/horizon_speech_sdk/lib/
DESTINATION lib/)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/hobot_audio/)

# Install include directories
install(
  DIRECTORY config
  DESTINATION lib/hobot_audio/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
