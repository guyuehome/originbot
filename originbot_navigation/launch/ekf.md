# ekf使用方法

首先启动底盘和IMU，其中底盘新增了一个参数，可以设置是否发布odom到base_footprint的tf，默认为true。

```
$ ros2 launch originbot_bringup originbot.launch.py use_imu:=true pub_odom:=false
```

其次即可使用ekf融合IMU和odom的数据，并且发布odom到base_footprint的tf。

```
$ ros2 launch originbot_navigation odom_ekf.launch.py
```

如果需要对ekf的参数进行调整，直接修改originbot_navigation/config/ekf.yam即可。