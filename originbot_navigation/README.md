# originbot_navigation

## SLAM

此处使用的 gmapping 建图方法，由于目前 foxy 版本未提供二进制下载方式，故使用源码下载 “openslam_gmapping”  ，“slam_gmapping” 两个功能包。

编译方式：

```
colcon build --packages-select originbot_slam
```

### Launch:

```bash
cd originbot/
source install/setup.bash
ros2 launch originbot_slam originbot_slam.launch.py
```




## 自主导航

originbot 自主导航运用 ros2 已有框架 navigation2 实现，在使用上，一方面可以下载源码进行安装，另一方面，可以安装二进制文件。

```
sudo apt install ros-foxy-nav2-*
```

编译方式：

```
colcon build --packages-select originbot_navigation
```

### 使用方式

```bash
cd originbot/
source install/setup.bash
ros2 launch originbot_navigation navigation2.launch.py
```

