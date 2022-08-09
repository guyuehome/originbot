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
1. 安装Ubuntu系统：https://developer.horizon.ai/api/v1/fileData/documents_pi/Quick_Start/Quick_Start.html#id3
2. 配置网络：https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id3
3. 更新系统：https://developer.horizon.ai/api/v1/fileData/documents_pi/System_Configuration/System_Configuration.html#id2
3. 安装TogetherROS：https://developer.horizon.ai/api/v1/fileData/TogetherROS/quick_start/install_tros.html
4. 安装ROS2：https://hhp.guyuehome.com/hhp/2.3_TogetherROS%E7%B3%BB%E7%BB%9F%E9%85%8D%E7%BD%AE/#ros2
    - https://blog.51cto.com/u_11440114/5102048
    - 可选华为云：https://mp.weixin.qq.com/s/cI9HhFs7ai6eQsUuhdb69A
5. 安装功能包：
```bash
$ sudo apt install python3-colcon-common-extensions           # ROS2编译器
$ sudo apt install git                                        # 安装git工具
$ sudo apt install ros-foxy-navigation2                       # 安装导航功能包
$ sudo apt install ros-foxy-nav2-bringup                      # 安装导航功能包
```
6. 创建dev_ws/src
7. 下载代码：git clone https://gitee.com/guyuehome/originbot.git
8. 编译代码：colcon build
9. 



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
ros2 launch originbot_bringup originbot.launch.py
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
ros2 launch originbot_bringup originbot.launch.py
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
