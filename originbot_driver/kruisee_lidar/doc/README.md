# KRUISEE SDK

## 1 简介

KRUISEE SDK 是为 KRUISEE 公司旗下所有产品设计的软件开发套件。它是基于C++14进行开发，对底层差异化通信进行封装和点云处理进行抽象，并为用户提供易用的接口。通过KRUISEE SDK，用户可以快速地连接 KRUISEE 产品并接收点云。

本程序适用于：家用DTOF ROS2

## 2 开发环境
+ Ubuntu 22.04 ROS2 foxy
+ C++14 compiler
+ cmake (version >= 3.5)

## 3 通信协议
| 字段 | 偏移（字节）| 大小（字节）| 描述 |
|:--------:|:--------:|:--------:|:--------:|
| header | 0 | 1 | 帧头，固定为0xFA|
| angle | 1 | 1 | 起始角度(实际角度=angle-0xA0) |
| speed | 2 | 2 | 雷达转速 |
| ranges | 4 | 16 * 2 | 测距值(单个值占2bytes)|
| timestamp | 36 | 4 | 时间戳 |
| intensities | 40 | 16 | 信号强度 |
| reserved | 56 | 4 | 保留 |
| cksum | 60 | 2 | 校验值 |

## 4 使用
### 4.1 Ubuntu 22.04 ROS2 foxy
#### 依赖
KRUISEE SDK 需要一些编译工具和相关库。你可以通过 apt 工具安装这些依赖：
```shell
# [1] 安装cmake
sudo apt install cmake
```

#### ROS2构建 & 编译
```shell

# [1] 将 kruisee_lidar 目录拷贝到ROS工作目录下
cp {kruisee_lidar folder}/kruisee_lidar {ROS Workspace folder}/src/

# [2] 确认雷达和上位机连接的串口
# 根据串口名，修改protocol.lua中关于串口号的配置 (src/kruisee_lidar/sdk_core/config目录)
-- 串口通讯方式配置
UART = {
    -- 串口号
    name = "/dev/ttyCH341USB0",
    -- 波特率
    baudrate = 460800,
}

# [3] 编译SDK
cd {ROS2 Workspace folder}
source /opt/ros/foxy/setup.bash （以foxy版本为例，其它版本相应更改）
colcon build

编译后会产生build，install目录，在此过程中：
启动文件kruisee_lidar.xml 或 kruisee_lidar.yaml 会由src/kruisee_lidar/launch目录拷贝到install/kruisee_lidar/share/kruisee_lidar/launch
配置.lua文件会由src/kruisee_lidar/sdk_core/config目录拷贝到install/kruisee_lidar/config
编译后若修改配置文件修改上述install目录中的。

# [4] 运行
每个雷达点云的topic，frame_id，按照如下修改install/kruisee_lidar/share/kruisee_lidar/launch/kruisee_lidar.xml：
<param name="topic" value="scan"/>
<param name="frame_id" value="map"/>
(如果后续用yaml格式的文件启动，则修改kruisee_lidar.yaml，xml yaml两种方式选择其一)
      name: "topic"
      value: "scan"

      name: "frame_id"
      value: "map"
启动：
source /opt/ros/foxy/setup.bash
source install/setup.bash
（xml格式的launch文件）
ros2 launch kruisee_lidar kruisee_lidar.xml
（或是用yaml格式的launch文件）
ros2 launch kruisee_lidar kruisee_lidar.yaml      


用户程序通过向以下topic发布消息进行控制启停：
topic：雷达发布点云的topic加_ctl，如点云topic为scan，控制启停的topic为scan_ctl
数据类型：std_msgs/msg/String
数据内容：start stop

使用ros2命令：
启动
ros2 topic pub --once /scan_ctl std_msgs/msg/String "data: 'start'"
停止
ros2 topic pub --once /scan_ctl std_msgs/msg/String "data: 'stop'"

ros2程序示例：
    auto node = std::make_shared<rclcpp::Node>("my_ros2_node");
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = node->create_publisher<std_msgs::msg::String>("/scan_ctl", 5);
    std_msgs::msg::String ctl;
    ctl.data = std::string("stop");
    publisher->publish(ctl);


```

### 4.2 Ubuntu 14.04/Ubuntu 16.04/Ubuntu 18.04/Ubuntu 20.04 NONE ROS2
后续更新

### 4.3 Windows 10
后续更新

## 5 配置使用

用户尽量不要直接修改如下配置文件: filters.lua、platform.lua、pose.lua、protocol.lua
用户可以修改kruisee_lidar.lua或者自定义配置脚本


用户可以在 return options 前进行SDK配置
| 配置 | 描述 | 注意 |
|:---:|:----:|:----:|
| PROTOCOL.use_net_protocol = true | 选择以太网通信协议 |
| PROTOCOL.use_uart_protocol = true | 选择串口通信协议 | 以太网和串口只能选择其一 |
| FILTERS.use_radius_filter = true | 开启半径滤波 |
| FILTERS.use_smooth_filter = true | 开启平滑滤波 |
| FILTERS.use_angle_filter = true | 开启角度滤波 |
| ANGLE_FILTER.range = { { 0, 45 }, {90, 270} } | 滤除0-45, 90-270度的点云数据 |
| PLATFORM.use_polar_coordinates = true | 以极坐标方式输出点云, ROS下为sensor_msgs::LaserScan |
| PLATFORM.use_cartesian_coordinates = true | 以直角坐标系输出点云,ROS下为sensor_msgs::PointCloud |
| POSE.rotate_angle = 123.0 | 设置零度，相当于机械零度逆时针旋转 |
| ETHERNET.radar_ip = "169.254.119.2" | 设置雷达IP |
| UART.name = "/dev/ttyUSB0" | 设置串口号 |
| RADIUS_FILTER.radius = 0.15 | 设置半径滤波器滤波半径 |
| RADIUS_FILTER.min_count = 4 | 设置半径滤波器最小临近点数 |
| SMOOTH_FILTER.level = 3 | 设置平滑滤波器平滑等级，越大滤波程度越高 |
| SMOOTH_FILTER.err = 0.1 |设置平滑滤波器滤波误差，越大滤除点越多 |

基于lua的配置文件解释如下

```lua
--
-- Copyright (c) 2022 ECOVACS
--
-- Use of this source code is governed by a MIT-style
-- license that can be found in the LICENSE file or at
-- https://opensource.org/licenses/MIT
--

-- 包含通信协议的脚本
include "protocol.lua"

-- 包含位姿相关的脚本
include "pose.lua"

-- 包含滤波器相关的脚本
include "filters.lua"

-- 包含平台相关的脚本
include "platform.lua"

-- options 禁止修改
options = {
    -- 导入通信协议
    protocol = PROTOCOL,

    -- 导入滤波器集合
    filters = FILTERS,

    -- 导入激光雷达位姿相关配置
    pose = POSE,

    -- 导入平台依赖配置(例如ROS)
    platform = PLATFORM,
}

-- 此处可以修改配置信息，使用如些所示

-- [1] 选择通信方式，二选一将使用的注释掉或者改为false
PROTOCOL.use_net_protocol = false    -- 采用以太网通信(默认关闭)
PROTOCOL.use_uart_protocol = true  -- 采用串口通信(默认开启)

-- [2] 设置雷达零度位置，相当于机械零度逆时针旋转指定角度
POSE.rotate_angle = 0.0

-- [3] 设置半径滤波
RADIUS_FILTER.radius = 0.15         -- 设置滤波半径
RADIUS_FILTER.min_count = 4         -- 设置滤波点数，在滤波半径内点数小于该值将会该滤除
RADIUS_FILTER.max_distance = 5      -- 设置半径滤波最大范围，单位m
FILTERS.use_radius_filter = true    -- 开启半径滤波

-- 设置近距离离群点滤波
OUTLIER_FILTER_NEAR.radius = 0.015       -- 设置滤波半径
OUTLIER_FILTER_NEAR.min_count = 3        -- 设置滤波点数，在滤波半径内点数小于该值将会该滤除
OUTLIER_FILTER_NEAR.max_distance = 0.3   -- 设置半径滤波最大范围，单位m
FILTERS.use_outlier_filter_near = false   -- 开启近距离半径滤波(默认关闭)

OUTLIER_FILTER_NEAR.front_angle = 172    -- 正前方正常滤波的角度 86*2
OUTLIER_FILTER_NEAR.side_angle = 67      -- 两侧需要强滤波的角度
OUTLIER_FILTER_NEAR.lidar_angle = 213    -- 雷达缺口点逆时针旋转到正前方的角度
OUTLIER_FILTER_NEAR.angle = false         -- 开启侧面角度滤波(默认关闭)

-- [4] 设置平滑滤波
SMOOTH_FILTER.level = 1             -- 设置平滑等级
SMOOTH_FILTER.err =0.1              -- 设置平滑滤波误差，小于该值进行滤除
FILTERS.use_smooth_filter = false    -- 开启平滑滤波(默认关闭)

-- [5] 设置角度滤波
ANGLE_FILTER.range = { {90, 270} }  -- 设置角度滤波范围，角度范围相当于设置零度位置后的角度范围
FILTERS.use_angle_filter = false     -- 开启角度滤波(默认关闭)

-- 默认点云极坐标系输出
PLATFORM.use_polar_coordinates = true

return options
```

## 6 NOTICE

当前版本滤波算法需要用户安装flann库和lz4库, 具体方式:
修改CMakeList.txt => target_link_libraries下libflann.so, libflann_cpp.so, liblz4.a
