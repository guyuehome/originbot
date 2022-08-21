# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [v0.2] - 2022-8-25
### 新增


### 优化
- 不同传感器的启动脚本都集成在originbot.launch.py中，通过参数来设置
- 调试视觉巡线功能，基于HSV模型识别制定区域内的某种颜色中心，从而转化成控制指令
- 旭日派中ROS启动成功时，蜂鸣器响0.5s


## [v0.1] - 2022-8-19
### 新增
- 提供OriginBot核心功能的驱动节点
    - 支持旭日派与控制器之间的串口驱动
    - 支持里程计话题的发布与TF坐标系维护
    - 支持IMU话题的发布
    - 支持速度控制指令的订阅与运动控制，可选是否启动自动停车功能
- 提供OriginBot常用应用功能包
    - 支持Cartographer SLAM地图构建功能
    - 支持Navigation2定位与导航功能
    - 支持手势控制功能
    - 支持人体跟踪功能

### 问题
- 摄像头驱动节点的标定话题待完善
- 手柄遥控运动待测试
- 二维码识别功能待测试
- 视觉巡线功能待测试
