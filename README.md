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
ros2 run originbot_navigation gmapping.launch.py
ros2 run originbot_navigation cartographer.launch.py
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

## 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request
