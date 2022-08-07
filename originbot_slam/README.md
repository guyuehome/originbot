# originbot_slam

此处使用的 gmapping 建图方法，由于目前 foxy 版本未提供二进制下载方式，故使用源码下载 “openslam_gmapping”  ，“slam_gmapping” 两个功能包。

编译方式：

```
colcon build --packages-select originbot_slam
```

## Launch:

```bash
cd originbot/
source install/setup.bash
ros2 launch originbot_slam originbot_slam.launch.py
```

