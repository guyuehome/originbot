cmake_minimum_required(VERSION 3.5)
project(kruisee_lidar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 开启TEST模式将向外发送一个额外的原始数据节点，用于进行点云显示对比
# add_definitions(-DSDK_TEST)

# 定义配置文件所在目录以及配置文件,默认用户配置文件和SDK配置文件都放在cfg目录下, 如果用户需要分离, 则需要设置SDK配置文件目录
# 和用户配置文件路径, 如下针对多目录的设置
# add_definitions(-DSDK_CFG_FILES_DIR=\"${PROJECT_SOURCE_DIR}/sdk_core/config\", \"${CMAKE_INSTALL_PREFIX}/config\")
add_definitions(-DSDK_CFG_FILES_DIR=\"${PROJECT_SOURCE_DIR}/sdk_core/config\")

# 选择配置文件, 不含路径
add_definitions(-DCFG_FILE=\"kruisee_lidar.lua\")

# add ros header files and third_party include
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/third_party/eigen3
)

ament_package()

# add ros node sources
include_directories(${PROJECT_SOURCE_DIR}/top/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/top node_sources)

include_directories(${PROJECT_SOURCE_DIR}/sdk_core/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/sdk_core)

# set(SDK_CORE_PATH /home/zhidou/catkin_workspace/src/kruisee_lidar/sdk_core/out/sdk_core)
# include_directories(${SDK_CORE_PATH}/include)
# link_directories(${SDK_CORE_PATH}/lib)

add_executable(${PROJECT_NAME}_node ${node_sources})

# add third_party dependency library
target_link_libraries(${PROJECT_NAME}_node sdk_core)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs)

install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION bin/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY sdk_core/config
    DESTINATION share/${PROJECT_NAME}
)

