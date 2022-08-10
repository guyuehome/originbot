# originbot
OriginBot智能机器人开源套件

## 软件架构
- originbot_base：机器人底盘驱动
- originbot_bringup：机器人启动相关的脚本和文件
- originbot_description：机器人模型及加载脚本
- originbot_navigation：机器人定位与导航相关的脚本和配置文件
- originbot_slam：机器人地图构建相关的脚本和配置文件
- originbot_body_tracking：机器人人体跟随功能包
- originbot_gesture_control：机器人手势控制功能包


## 安装教程
### 系统镜像配置过程
1. 安装Ubuntu系统（推荐使用服务器server版本）：
https://developer.horizon.ai/api/v1/fileData/documents_pi/Quick_Start/Quick_Start.html#id3


2. 配置网络：https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id3

3. 更新系统：https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id2

3. 安装TogetherROS：https://developer.horizon.ai/api/v1/fileData/TogetherROS/quick_start/install_tros.html

4. 安装ROS2：https://hhp.guyuehome.com/hhp/2.3_TogetherROS%E7%B3%BB%E7%BB%9F%E9%85%8D%E7%BD%AE/#ros2
    - 如遇到网络连接问，可参考：https://blog.51cto.com/u_11440114/5102048、https://guyuehome.com/37844

5. 安装功能包：
```bash
$ sudo apt install python3-colcon-common-extensions # ROS2编译器
$ sudo apt install git                              # 安装git工具
$ sudo apt install ros-foxy-navigation              # 安装导航功能包
$ sudo apt install ros-foxy-nav2-bringup            # 安装导航功能包
$ sudo apt install ros-foxy-slam-toolbox            # 安装slam-toolbox
$ sudo apt install ros-foxy-cartographer-ros        # 安装cartographer
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


14. 重启系统，确保以上配置生效

## 使用说明

### 键盘遥控

第一个终端：

```bash
ros2 launch originbot_bringup originbot.launch.py
```

第二个终端：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### SLAM地图构建

第一个终端：

```bash
ros2 launch originbot_bringup originbot_lidar.launch.py
```

第二个终端（选择如下一项启动）：

```bash
ros2 launch originbot_navigation gmapping.launch.py
ros2 launch originbot_navigation cartographer.launch.py
```

第三个终端：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

保存地图：
- Gmapping
```bash
ros2 run nav2_map_server map_saver_cli -f map
```

- Cartographer
```bash
ros2 run nav2_map_server map_saver_cli -f ~/cartorapher --ros-args -p save_map_timeout:=10000
```

### 自主导航

第一个终端：

```bash
ros2 launch originbot_bringup originbot_lidar.launch.py
```

第二个终端：

```bash
ros2 run originbot_navigation nav_bringup.launch.py
```

### 人体跟踪

第一个终端：

```bash
ros2 launch originbot_bringup originbot.launch.py
```

第二终端：

```bash
# 配置TogetherROS环境
source /opt/tros/setup.bash

# 从TogetherROS的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/lib/mono2d_body_detection/config/ .

#启动launch文件
ros2 launch body_tracking hobot_body_tracking_without_gesture.launch.py 
```

### 手势识别

第一个终端：

```bash
ros2 launch originbot_bringup originbot.launch.py
```

第二终端：

```bash
# 配置TogetherROS环境
source /opt/tros/setup.bash

# 从TogetherROS的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/lib/hand_gesture_detection/config/ .

#启动launch文件
ros2 launch gesture_control hobot_gesture_control.launch.py
```

## 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request
