# originbot
OriginBot智能机器人开源套件



## 软件架构

- originbot_base：机器人底盘驱动
- originbot_driver：机器人设备驱动
    - serial_ros2：串口驱动包
    - ydlidar_ros2_driver：雷达驱动包
- originbot_bringup：机器人启动相关的脚本和文件
- originbot_description：机器人模型及加载脚本
- originbot_linefollower：机器人视觉巡线功能包
- originbot_navigation：机器人建图与导航相关的脚本和配置文件
- originbot_deeplearning：机器人深度学习功能
    - body_tracking：机器人人体跟随功能包
    - gesture_control：机器人手势控制功能包
    - line_follower_perception：机器人AI视觉巡线功能包



## 硬件接口

### 扩展板接口说明

![1280X1280](images/mcu_board.png)



### 硬件连接

1. 扩展板USB旭日派供电口 --> 旭日派TypeC电源口
2. 电池DC头 --> 扩展板12V电池接口
3. 扩展板上位机通信端口 --> 旭日派40PIN扩展口中的UART3（rx、tx、gnd）



### 固件升级方法

- 断开扩展板与MCU的连接
- 使用typeC线连接扩展板的“Burning”口和电脑
- 电脑端启动flymcu软件
- 在flymcu软件中，选择烧写的固件，按照下图进行配置后，点击“开始编程”
![img](images/20220813215508.png)
- 等待右侧日志提示编程完成，固件升级结束



## 系统配置

### 安装系统镜像

#### 使用配置好的系统镜像

使用win32disk或者rufus工具，将提供的sd卡镜像文件烧写到配套的SD卡中。




#### 纯净系统的完整配置过程
1. 安装Ubuntu系统（推荐使用服务器server版本）：
https://developer.horizon.ai/api/v1/fileData/documents_pi/Quick_Start/Quick_Start.html#id3


2. 配置网络：
https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id3

3. 更新系统：
https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id2

3. 安装TogetherROS：
https://developer.horizon.ai/api/v1/fileData/TogetherROS/quick_start/install_tros.html

4. 安装ROS2：
https://hhp.guyuehome.com/hhp/2.3_TogetherROS%E7%B3%BB%E7%BB%9F%E9%85%8D%E7%BD%AE/#ros2
    - 如遇到网络连接问，可参考：https://blog.51cto.com/u_11440114/5102048、https://guyuehome.com/37844

5. 安装功能包：
```bash
$ sudo apt install python3-colcon-common-extensions # ROS2编译器
$ sudo apt install git                              # 安装git工具
$ sudo apt install ros-foxy-navigation              # 安装导航功能包
$ sudo apt install ros-foxy-nav2-bringup            # 安装导航功能包
$ sudo apt install ros-foxy-slam-toolbox            # 安装slam-toolbox
$ sudo apt install ros-foxy-cartographer-ros        # 安装cartographer
$ sudo apt install ros-foxy-teleop-twist-keyboard   # 安装键盘控制节点
$ sudo apt install ros-foxy-robot-localization      # 安装定位功能包
$ sudo apt install cloud-utils                      # 安装磁盘工具
$ sudo apt install ros-foxy-rmw-cyclonedds-cpp      # 安装Cyclone DDS
$ sudo apt install ros-foxy-teleop-twist-joy        # 安装手柄的驱动包
$ sudo apt install ros-foxy-joy-linux               # 安装手柄的驱动包
$ sudo apt install ros-foxy-cv-bridge               # 安装图像处理包
$ sudo apt install ros-foxy-xacro                   # 安装模型解析包
$ sudo apt install ros-foxy-robot-state-publisher   # 安装机器人状态发布者包
$ sudo apt install ros-foxy-joint-state-publisher   # 安装关节状态发布者包
$ sudo apt install ros-foxy-joint-state-publisher-gui   # 安装关节状态发布者包
```

如遇到类似如下问题：
![img](images/20220810103157.png)

需要修改一下软件源的配置：
```bash
$ sudo vi /etc/apt/sources.list
```

将下边被注释掉的几个软件源打开：
![img](images/20220810104154.png)

修改好重新update再安装即可。


6. 配置软链接：
```bash
$ cd /opt/tros 
## 使用/opt/tros目录下的create_soft_link.py创建ROS package至TogetherROS的软链接 
$ python3 create_soft_link.py --foxy /opt/ros/foxy/ --tros /opt/tros 
```

7. 在userdata（或root）文件夹下，创建dev_ws/src工作空间:
```bash
$ mkdir -p /userdata/dev_ws/src
```

8. 下载代码到src中：
```bash
$ cd /userdata/dev_ws/src
$ git clone https://gitee.com/guyuehome/originbot.git
```

9. 安装YDLidar的SDK：
```bash
# 安装工具库
$ sudo apt install cmake pkg-config
$ sudo apt-get install python swig
$ sudo apt-get install python3-pip

# 编译ydlidar SDK
$ cd /userdata
$ git clone https://github.com/YDLIDAR/YDLidar-SDK.git
$ cd YDLidar-SDK
$ mkdir build
$ cd build
$ cmake ..
$ make -j2
$ sudo make install

# 安装python版本的SDK
$ cd ..
$ pip install .
```

10. 在工作空间中编译代码：
```bash
$ cd /userdata/dev_ws
$ source /opt/tros/setup.bash
$ colcon build
```

11. 配置串口的端口号
```bash
$ chmod 0777 /userdata/dev_ws/src/originbot/ydlidar_ros2_driver/startup/*
$ sudo sh /userdata/dev_ws/src/originbot/ydlidar_ros2_driver/startup/initenv.sh
```

12. 添加环境变量到/root/.bashrc（和登录的用户有关系，使用其他用户登录的话，就修改对应用户文件夹下的.bashrc）
```bash
$ vi /root/.bashrc

# 在文件末尾添加如下内容：
source /opt/tros/setup.bash
source /userdata/dev_ws/install/local_setup.bash
```

13. 为保证后续使用的顺畅，可以配置1GB的SWAP空间：
```bash
$ sudo mkdir -p /swapfile 
$ cd /swapfile 
$ sudo dd if=/dev/zero of=swap bs=1M count=1024 
$ sudo chmod 0600 swap 
$ sudo mkswap -f swap 
$ sudo swapon swap 
$ free
```

操作效果如下：
![img](images/20220810105929.png)

14. 修改系统中调用的摄像头，改为对应的使用版本


15. 重启系统，确保以上配置生效



### 系统镜像备份与恢复方法

参考：https://www.elecfans.com/d/comp-4395939-136407485203429933056.html


#### 备份

备份在电脑端Ubuntu系统中完成，为减少镜像大小，这里使用gparted软件压缩SD卡的磁盘空间。

将SD卡插入运行的电脑中，安装并启动gparted软件：

```bash
$ sudo apt-get install gparted
$ sudo gparted
```

在右上角选择SD卡：
![img](images/2022-08-19_18-13.png)

卸载SD卡的挂载：
![img](images/2022-08-19_18-27.png)

右键选择重新设置SD卡的空间大小：
![img](images/2022-08-19_18-27_1.png)

拖拽空间划分条右侧的尺度调整，压缩空间，黄色是占用的分区，白色是空闲的分区，注意保留一点白色空闲分区即可。
![img](images/2022-08-19_18-28.png)

确认后点击软件中的执行按钮，开始进行压缩操作。
![img](images/2022-08-19_18-28_1.png)


压缩完成后，即可关闭gparted软件，打开终端，使用fdisk命令查看当前磁盘情况。
```bash
$ sudo fdisk -u -l
```
![img](images/2022-08-19_18-23.png)

图中/dev/sdb就是SD卡的磁盘编号，继续使用如下命令查看SD卡分区信息：
```bash
$ sudo fdisk -u -l /dev/sdb
```
![img](images/2022-08-19_18-23_1.png)

现在就可以使用dd命令，来备份SD卡到img文件中了。
```bash
$ sudo dd bs=512 count=[fdisk命令中最大的end数+1] if=/dev/sdb of=originbot_backup.img
```
![img](images/2022-08-19_18-25.png)

此时终端中并没有提示信息，但是备份已经开始，时间较长，请耐心等待。

完成备份后，终端指令会自动跳出，当前路径下得到img镜像备份文件。
![img](images/2022-08-19_18-32.png)



## 开发环境搭建

### PC端VSCode远程调试
https://blog.csdn.net/zhaxun/article/details/120568402

![img](images/2022-08-13_01-34.png)



## 操作说明

### 上电启动机器人

- 确认硬件连线完毕
- 打开电源总开关
- 扩展板电源灯点亮
- 旭日派启动
- 雷达开始旋转
- 通过网线或者串口线连接并登陆旭日派
- 启动成功





### 确认相机运行正常

```bash
$ ros2 launch websocket hobot_websocket.launch.py
```

运行成功后，在同一网络中的PC机上，打开浏览器，访问机器人的IP地址，如顺利看到相机图像，则说明相机运行正常。

![img](images/2022-08-12092248.png)

如果运行失败，请确认如下路径中相机型号是“GC4663”，如果不是，请修改好重新运行。

```bash
/opt/tros/share/websocket/launch/hobot_websocket.launch.py
```
![img](images/20220812091950.png)



### 启动相机并发布ROS2话题

```bash
$ ros2 launch originbot_bringup camera.launch.py
```

运行成功后，可在机器人端或同网络下的PC端使用如下命令，确认图像话题已经正常发布。
```bash
$ ros2 topic list
```
![img](images/20220812093211.png)


在同一网络中的PC机上，使用rviz订阅图像话题，即可看到图像。

注意：如果出现图像卡顿或延迟，可以将使用的DDS切换为cyclonedds
https://developer.horizon.ai/api/v1/fileData/TogetherROS/FAQs/FAQs.html#rqt-image-viewx3rgb888-raw

```bash
$ sudo apt install ros-foxy-rmw-cyclonedds-cpp
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'  # 用于选择数据传输的网卡，按照实际需要，修改为eth0或者wlan0，http://www.robotandchisel.com/2020/08/12/cyclonedds/
$ ros2 doctor --report | grep middleware # 确定切换DDS是否成功，需要等待10几秒钟
$ ros2 launch originbot_bringup camera.launch.py
```


### 查看机器人可视化模型

#### PC端

```bash
$ ros2 launch originbot_description display.launch.py
```
![img](images/2022-08-30_13-57.png)



### 查看雷达可视化信息

#### 机器人端

启动机器人底盘及雷达驱动：

```bash
$ ros2 launch originbot_bringup originbot.launch.py use_lidar:=true
```



#### PC端

```bash
$ ros2 run rviz2 rviz2
```



添加Laserscan之后，配置订阅的话题，即可看到可视化的雷达信息：
![img](images/2022-08-10_18-01.png)



### 查看IMU可视化数据

#### 机器人端

启动机器人底盘驱动：

```bash
$ ros2 launch originbot_bringup originbot.launch.py use_imu:=true
```



#### PC端

```bash
$ ros2 run rviz2 rviz2
```



添加IMU之后，配置订阅的话题，即可看到可视化的IMU信息：
![img](images/2022-08-10_18-04.png)



如果Rviz中找不到imu插件，需要进行安装：

```bash
$ sudo apt install ros-foxy-rviz-imu-plugin
```



### 键盘遥控

#### 机器人端

第一个终端，启动机器人底盘驱动：

```bash
$ ros2 launch originbot_bringup originbot.launch.py
```

第二个终端（在PC端运行也可以）：

```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```



#### PC端

如果想要查看机器人的动态运动效果，可以在PC端打开Rviz查看：

```bash
$ ros2 run rviz2 rviz2
```

Fixed Frame选择odom，添加tf显示，即可看到：
![img](images/2022-08-10_23-30.png)


### 手柄遥控

#### 手柄连接
- 有线手柄；将手柄的usb接口插入旭日X3派或电脑端的的USB接口
- 无线手柄；将手柄的无线接收器插入旭日X3派或电脑端的的USB接口

请确认以下功能包在连接手柄的主机中已经安装完成：
```bash
$ sudo apt install ros-foxy-teleop-twist-joy        # 安装手柄的驱动包
$ sudo apt install ros-foxy-joy-linux               # 安装手柄的驱动包
```


#### 机器人端

第一个终端，启动机器人底盘驱动：

```bash
$ ros2 launch originbot_bringup originbot.launch.py
```

第二个终端（在PC端运行也可以）：

```bash
$ ros2 launch originbot_bringup joy_teleop.launch.py
```

- 常速运行：按住L1，拨动手柄左侧摇杆，控制机器人运动
- 快速运行：按住R1，拨动手柄左侧摇杆，控制机器人运动



### 调整电机PID参数

在以上启动机器人驱动节点的情况下：

```bash
$ ros2 service call /originbot_pid originbot_msgs/srv/OriginbotPID "{p: 0.1, i: 0.0, d: 4.0}" 
```
![img](images/20220811103849.png)



### 机器人里程计校准

#### 准备工作
##### 硬件
卷尺、量角器/手机指南针软件

#### 软件
将出厂自带的校准系数归一
![img](images/2022-09-02_10-29.png)
注意使用浮点数，并且修改后要重新编译


#### 线速度校准
- 将卷尺展开放在地上，作为实际直线里程的参考值
![img](images/2022-08-31-1.jpeg)

- 将机器人放在卷尺刻度的零位，使用机器人底盘的前边缘作为参考线
![img](images/2022-08-31-2.jpeg)

- 启动机器人底盘
```bash
$ ros2 launch originbot_bringup originbot.launch.py
```
- 动态显示机器人里程计数据
```bash
$ ros2 topic echo /odom
```
![img](images/2022-09-02_10-35.png)

- 启动键盘控制节点，速度调慢一点，控制机器人向前运动
```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- 运动一段距离后，比如1m左右，停止运行，查看此时机器人实际运行的距离，以及odom反馈的累计距离
比如此时实际运行的距离是1.023m：
![img](images/2022-08-31-8.jpeg)


而里程计反馈的是1.139m：
![img](images/2022-09-02_10-41.png)


- 计算线速度的校准系数 = 实际运行距离 / 里程计反馈距离
1.023/1.139 = 0.898

- 将校准参数填入robot.launch.py中，线速度校准完毕
![img](images/2022-09-02_10-43.png)

为提高校准精度，可以重复以上步骤3~5次，计算得到的系数局平均值


#### 角速度校准
- 将机器人放在平整的地面上，在地面上寻找一个参考线，或者人为绘制一条参考线，作为机器人旋转的零点
![img](images/2022-08-31-3.jpeg)


- 启动机器人底盘
```bash
$ ros2 launch originbot_bringup originbot.launch.py
```
- 动态显示机器人里程计数据
```bash
$ ros2 topic echo /odom
```
![img](images/2022-09-02_10-35.png)

- 启动键盘控制节点，速度调慢一点，控制机器人原地旋转
```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- 旋转一段距离后，比如300度左右，停止运行，标记此时机器人的姿态参考线
![img](images/2022-08-31-4.jpeg)


- 计算机器人实际的旋转角度：可以使用量角器测量，也可以使用手机的指南针测量
比如机器人位于零点时，手机指南针显示的角度是21度
![img](images/2022-08-31-6.jpeg)


机器人停止运动时，手机指南针显示的角度是341度
![img](images/2022-08-31-5.jpeg)


可以得知机器人实际旋转的角度是：341 - 21 = 320度


里程计反馈的旋转角度是：
![img](images/2022-09-02_11-04.png)

这里显示的是四元数，我们需要将四元数转换成欧拉角，可以使用该网站进行在线转换：
https://quaternions.online/

![img](images/2022-09-02_11-06.png)

从计算结果中可以发现，机器人里程计应该是积分超过360度之后，又旋转了6.193，也就是一共积分旋转了360+6.193 = 366.193


- 计算角速度的校准系数 = 实际旋转的角度 / 里程计反馈角度
320/366.193 = 0.874

- 将校准参数填入robot.launch.py中，角速度校准完毕
![img](images/2022-09-02_11-09.png)

为提高校准精度，可以重复以上步骤3~5次，计算得到的系数局平均值



### 查看机器人状态

在以上启动机器人驱动节点的情况下：

```bash
$ ros2 topic echo /originbot_status
```

![img](images/20220811102658.png)



### 控制板载蜂鸣器

在以上启动机器人驱动节点的情况下：

```bash
$ ros2 service call /originbot_buzzer originbot_msgs/srv/OriginbotBuzzer "'on': true"     ## 打开蜂鸣器
$ ros2 service call /originbot_buzzer originbot_msgs/srv/OriginbotBuzzer "'on': false"    ## 关闭蜂鸣器
```

![img](images/20220811103849.png)



### 控制板载LED灯

在以上启动机器人驱动节点的情况下：

```bash
$ ros2 service call /originbot_led originbot_msgs/srv/OriginbotLed "'on': true"      ## 打开LED
$ ros2 service call /originbot_led originbot_msgs/srv/OriginbotLed "'on': false"     ## 关闭LED
```

![img](images/20220811102658.png)



### 查看CPU和BPU的频率与温度

```bash
hrut_somstatus
```

![img](images/20220812222045.png)



### SLAM地图构建

#### 机器人端

第一个终端：

```bash
$ ros2 launch originbot_bringup originbot.launch.py use_lidar:=true
```

第二个终端：

```bash
$ ros2 launch originbot_navigation cartographer.launch.py
```

第三个终端（在PC端运行也可以）：

```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

保存地图：
```bash
$ ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p save_map_timeout:=10000
```



#### PC端

```bash
$ ros2 run rviz2 rviz2
```
添加map、tf、laserscan等显示插件后，可以看到slam的过程


![img](images/2022-08-11_20-47.png)


### 自主导航



#### 机器人端

第一个终端：

```bash
$ ros2 launch originbot_bringup originbot.launch.py use_lidar:=true
```

第二个终端：

```bash
$ ros2 launch originbot_navigation nav_bringup.launch.py
```
启动成功后，会在终端中看到不断输出的信息，这是因为没有设置机器人初始位姿的缘故，后续启动Rviz之后会进行设置，暂时可以忽略。


#### PC端

```bash
$ ros2 run rviz2 rviz2
```
注意：如果在配置好的Rviz中没看到静态地图，可以尝试关闭运行导航的终端，重新打开Rviz之后，再启动Rivz。


- 单目目标点导航
在打开的Rviz中配置好显示项目，点击工具栏中的初始状态估计按钮，在地图中选择机器人的初始位姿，此时此前终端中的警告也会停止，然后点击目标位置选择的按钮，在地图上选择导航目标点，即可开始自主导航。

![img](images/202208111147062.png)

- 多目标点导航
在打开的Rviz中配置好显示项目，点击工具栏中的初始状态估计按钮，在地图中选择机器人的初始位姿，此时此前终端中的警告也会停止，然后点击目标位置选择的按钮，在地图上选择导航目标点，即可开始自主导航。

![img](images/2022-08-13_16-16.png)
![img](images/2022-08-13_16-17.png)
![img](images/2022-08-13_16-23.png)




### 人体跟随

#### 机器人端

第一个终端：

```bash
$ ros2 launch originbot_bringup originbot.launch.py
```

第二个终端：

```bash
$ source /opt/tros/setup.bash

# 从TogetherROS的安装路径中拷贝出运行示例需要的配置文件
$ cd /userdata/dev_ws
$ cp -r /opt/tros/lib/mono2d_body_detection/config/ .

#启动launch文件
$ ros2 launch body_tracking hobot_body_tracking_without_gesture.launch.py 
```

#### PC端

打开浏览器，访问机器人的ip地址，即可看到视觉识别的实时效果。
![img](images/2022-08-10_23-09.png)



### 手势控制

#### 机器人端

第一个终端：

```bash
$ ros2 launch originbot_bringup originbot.launch.py
```

第二个终端：

```bash
# 配置TogetherROS环境
$ source /opt/tros/setup.bash

# 从TogetherROS的安装路径中拷贝出运行示例需要的配置文件
$ cd /userdata/dev_ws
$ cp -r /opt/tros/lib/mono2d_body_detection/config/ .
$ cp -r /opt/tros/lib/hand_lmk_detection/config/ .
$ cp -r /opt/tros/lib/hand_gesture_detection/config/ .

#启动launch文件
$ ros2 launch gesture_control hobot_gesture_control.launch.py
```

### PC端

打开浏览器，访问机器人的ip地址，即可看到视觉识别的实时效果。
![img](images/2022-08-10_23-09.png)



### 视觉巡线

#### 机器人端

第一个终端：

```bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ ros2 launch originbot_bringup originbot.launch.py use_camera:=true
```

第二个终端：

```bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ ros2 run originbot_linefollower follower
```

### PC端

```bash
ros2 run rqt_image_view rqt_image_view
```
订阅/camera/process_image话题，即可看到动态识别的图像效果。

![img](images/2022-08-21_15-15.png)



### 虚拟仿真

#### PC端依赖安装
```bash
$ sudo apt install ros-foxy-gazebo-ros
$ sudo apt install ros-foxy-gazebo-ros2-control
$ sudo apt install ros-foxy-gazebo-plugins
$ sudo apt install ros-foxy-ros2-control
$ sudo apt install ros-foxy-ros2-controllers

```

#### 运行仿真环境
```bash
$ ros2 launch originbot_gazebo originbot_gazebo.launch.py
```
![img](images/2022-08-31_23-42.png)


#### 查看仿真话题列表

```bash
$ ros2 topic list
```
![img](images/2022-08-31_23-43.png)


#### 控制机器人运动
```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


#### 查看相机与雷达可视化数据
在仿真环境中放入一些障碍物：
![img](images/2022-08-31_23-47.png)



```bash
$ ros2 run rviz2 rviz2
```
![img](images/2022-08-31_23-46.png)



## 常见问题

1. 古月居提供的系统镜像已经完成常用ROS2功能包的安装，如有新安装的ROS2包，安装后需要完成和tros的软链接，才能使用；
2. 通过“Burning”口给扩展板下载固件时，需要断开扩展板与旭日派的串口连接，否则将无法连接下载；


## 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request
