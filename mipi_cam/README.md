# Getting Started with Mipi_Cam Node
---
# Intro
---
通过阅读本文档，用户可以在地平线X3开发板上轻松抓取mipi摄像头的视频流数据，并通过ROS平台发布满足ROS标准的图片数据，供其他ROS Node订阅获取。目前支持F37、IMX415、GC4663、IMX219、IMX477、OV5647等mipi标准设备。
Mipi_cam Node package是地平线机器人开发平台的一部分，基于地平线VIO和ROS2 Node进行二次开发，为应用开发提供简单易用的摄像头数据采集功能的功能，避免重复开发获取视频的工作。支持 share mem 方式发布。

# Build
---
## Dependency

依赖库：
ros package：
- sensor_msgs
- hbm_img_msgs

hbm_img_msgs pkg是在hobot_msgs中自定义的图片消息格式，用于shared mem场景下的图片传输。

## 开发环境
- 编程语言：C/C++
- 开发平台：X3/X86
- 系统版本：Ubuntu 20.04
- 编译工具链：Linux GCC 9.3.0/Linaro GCC 9.3.0
## package说明
---
源码包含mipi_cam package。mipi_cam 编译完成后，头文件、动态库以及依赖安装在install/mipi_cam 路径。

## 编译
支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。

### X3 Ubuntu系统上编译
1、编译环境确认

- 板端已安装X3 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
- 已依赖pkg ，详见 Dependency 部分

2、编译：
  `colcon build --packages-select mipi_cam`。


### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令： 

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select mipi_cam \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
     
  ```
  



# Usage
## X3 Ubuntu系统

运行方式1，用户直接调用ros2 命令启动即可：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# 默认F37 sensor
ros2 run mipi_cam mipi_cam
```

打开了shared mem通信方式，只支持发布 hbmem_img 主题的图片。   

node会发布/image_raw topic，对应rgb8格式图片，使用 share mem 发布主题：hbmem_img。相机内参发布话题：/camera_info。

利用 rqt_image_view 可以查看发布的图片主题，也可以用图片消费节点。例如：这个repo下的example去直接获取图片进行推理等应用。

可以设置使用的sensor，发布图片的编码方式和分辨率。

使用video_device参数设置使用的sensor。目前支持 F37（默认），IMX415（通过--ros-args -p video_device:=IMX415设置），F37 默认分辨率是1920x1080；IMX415 是3840x2160。

使用image_width和image_height参数设置发布图片的分辨率:

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用out_format参数设置发布图片的编码方式，默认是bgr8编码方式，支持nv12格式（/image_raw topic），例如使用F37 sensor发布960x540分辨率的nv12格式图片：

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用 io_method 参数设置发布图像采用的方式，目前 shared_mem 发布的主题是固定的：hbmem_img

`ros2 run mipi_cam mipi_cam --ros-args -p io_method:=shared_mem`

使用 camera_calibration_file_path 参数设置相机标定文件路径，此处以使用GC4663相机并读取config文件下的GC4663_calibration.yaml为例(打印信息见下方Attention)：

```
# config中为示例使用的相机标定文件，根据实际安装路径进行拷贝
cp -r install/lib/mipi_cam/config/ .
ros2 run mipi_cam mipi_cam --ros-args -p camera_calibration_file_path:=./config/GC4663_calibration.yaml -p video_device:=GC4663
```

---

## X3 linaro系统

把在docker 交叉编译的install 目录拷贝到linaro 系统下，例如:/userdata
需要首先指定依赖库的路径，例如：
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`


修改 ROS_LOG_DIR 的路径，否则会创建在 /home 目录下，需要执行 mount -o remount,rw /，才可以在 /home 下创建日志
`export ROS_LOG_DIR=/userdata/`

运行 mipi_cam
```
// 默认参数方式
/userdata/install/lib/mipi_cam/mipi_cam
// 传参方式
#/userdata/install/lib/mipi_cam/mipi_cam --ros-args -p image_width:=960 -p image_height:=540

```

运行方式2，使用launch文件启动：
`ros2 launch install/share/mipi_cam/launch/mipi_cam.launch.py`

# Attention
目前设备出来的数据默认为nv12，转rgb8 格式，目前没有用cv，1920*1080 性能耗时 100ms 左右，压缩图需要用中继的方式支持：
ros2 run image_transport republish [in_transport] in:=<in_base_topic> [out_transport] out:=<out_base_topic>
例如：
ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_raw --remap out/compressed:=/image_raw/compressed
则会有 compressed 的话题，利用 sub 端可以订阅到压缩图片话题，例如：
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed
日志显示：
```
root@xj3ubuntu:/userdata/cc_ws/tros_ws# ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed
[WARN] [1648302887.615608845] [example]: This is image_subscriber example!
[WARN] [1648302887.699318639] [ImageSubscriber]: Update sub_img_topic with topic_name: /image_raw/compressed
[WARN] [1648302887.701353516] [ImageSubscriber]: Update save_dir: 
[WARN] [1648302887.701502469] [ImageSubscriber]: Create subscription with topic_name: /image_raw/compressed
[WARN] [1648302887.705133283] [example]: ImageSubscriber init!
[WARN] [1648302887.706179033] [example]: ImageSubscriber add_node!
[INFO] [1648302889.318928227] [img_sub]: Recv compressed img
[WARN] [1648302889.319329711] [img_sub]: Sub compressed img fps = 1
[INFO] [1648302889.319478247] [img_sub]: Recv compressed img: rgb8; jpeg compressed bgr8, stamp: 1648302889.92334955, tmlaps(ms): 227, data size: 33813
```
注意：此项功能，需要安装 ros包 image_transport_plugins，利用命令：
sudo apt-get install ros-foxy-image-transport-plugins


若相机成功运行，正常读取相机标定文件，则会有以下信息输出。mipi_cam提供两种相机型号的标定文件，分别是GC4663和F37，默认是读取config文件下的F37_calibration.yaml。如使用GC4663摄像头，请更换相机标定文件的读取路径！
```
[INFO] [1661863164.454533227] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544567->laps=102 ms.

[INFO] [1661863164.458727776] [mipi_node]: publish camera info.

[INFO] [1661863164.562431009] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544674->laps=103 ms.

[INFO] [1661863164.566239194] [mipi_node]: publish camera info.

[INFO] [1661863164.671290847] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544782->laps=104 ms.

[INFO] [1661863164.675211155] [mipi_node]: publish camera info.

[INFO] [1661863164.780465260] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1544891->laps=104 ms.

[INFO] [1661863164.784429400] [mipi_node]: publish camera info.

[INFO] [1661863164.887891555] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545000->laps=103 ms.

[INFO] [1661863164.891738656] [mipi_node]: publish camera info.

[INFO] [1661863164.994701993] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545107->laps=103 ms.

[INFO] [1661863164.998455013] [mipi_node]: publish camera info.

[INFO] [1661863165.100916367] [mipi_cam]: [get_image]->enc=bgr8,step=5760, w:h=1920:1080,sz=3110400,start 1545214->laps=102 ms.

[INFO] [1661863165.104211776] [mipi_node]: publish camera info.
```
