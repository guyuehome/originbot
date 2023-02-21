# 功能介绍

audio_control package功能为通过语音命令词来控制机器人运动。

此package订阅智能语音结果audio_msg，解析音频智能帧，确定命令词并根据命令词去控制机器人运动。本示例命令词的定义在hobot_audio package中，此package需要搭配hobot_audio一起使用；若用户有其他智能语音处理模块，也可以通过发布audio_msg::msg::SmartAudioData话题消息提供给本package使用。

示例中通过订阅hobot_audio发布的语音智能帧消息，解析出命令词之后来控制机器人旋转和平移运动，定义的语音命令词在hobot_audio package的config文件夹的cmd_word.json文件里面，目前默认的命令词有：

```
{
    "cmd_word": [
        "地平线你好",
        "向前走",
        "向后退",
        "向左转",
		"向右转",
		"停止运动"
    ]
}
```

配置文件的第一项为唤醒词，后面的是命令词。唤醒词用于唤醒机器，后面的命令词用于控制运动。此处若定义的唤醒词以及命令词有所修改，则audio_control package里面相关解析命令词以及控制运动的代码也需要做相应的修改。



# 编译

## 依赖库

ros package：

- audio_msg
- hobot_audio

audio_msg为自定义音频智能帧的消息格式，用于算法模型推理后，发布推理结果，audio_msg pkg定义在hobot_msgs中。

## 开发环境

\- 编程语言: C/C++

\- 开发平台: X3/X86

\- 系统版本：Ubuntu 20.0.4

\- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## **编译**

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### **Ubuntu板端编译**

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select audio_control`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select audio_control \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项



# 使用介绍

## 依赖

- hobot_audio package：采集语音并进行智能处理的package, 发布音频智能帧msg

## 参数

| 参数名                | 类型        | 解释                                 | 是否必须 | 支持的配置                                                   | 默认值       | 是否支持运行时动态配置 |
| --------------------- | ----------- | ------------------------------------ | -------- | ------------------------------------------------------------ | ------------ | ---------------------- |
| ai_msg_sub_topic_name | std::string | 订阅的音频智能帧消息话题             | 否       | 根据实际情况配置                                             | /audio_smart | 是                     |
| twist_pub_topic_name  | std::string | 发布Twist类型的运动控制消息的topic名 | 否       | 根据实际部署环境配置。一般机器人订阅的topic为/cmd_vel，ROS2 turtlesim示例订阅的topic为turtle1/cmd_vel。 | /cmd_vel     | 否                     |
| move_step             | float       | 平移运动的步长，单位米。             | 否       | 无限制                                                       | 0.3         | 是                     |
| rotate_step           | float       | 旋转运动的步长，单位弧度。           | 否       | 无限制                                                       | 0.5          | 是                     |
| motion_duration_seconds           | int       | 平移/旋转动作持续时间，单位秒，小于等于0表示不做限制，达到持续时间后下发停止运动指令，避免机器人一直运动           | 否       | 无限制                                                       | 0          | 否                     |


## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/hobot_audio/config/ .

# 加载音频驱动，设备启动只需要加载一次
bash config/audio.sh

# 启动音频控制节点
ros2 launch install/share/audio_control/launch/hobot_audio_control.launch.py
```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/hobot_audio/config/ .

# 加载音频驱动，设备启动只需要加载一次
sh config/audio.sh

# 启动音频处理pkg
./install/lib/hobot_audio/hobot_audio --log-level error &

# 启动音频控制pkg
./install/lib/audio_control/audio_control
```

## 注意事项

若无实体小车，用户可以使用gazebo仿真环境启动虚拟小车，通过本示例来控制小车运动。



# 结果分析

## X3结果展示

```

        This is audio control package.

============================================
        audio control usage

Wake up device is "地平线你好".
Audio control commnad word definitions are:
        "向前走": move front. (close from controler)
        "向后退": move back. (far from controler)
        "向右转": rotate robot to right.
        "向左转": rotate robot to left. 
============================================

```

以上log截取了部分通过音频控制小车运动的处理结果。用户可以通过定义的命令词“向前走”、“向后退”、“向左转”、“向右转”来控制小车进行运动。

## web效果展示



# 常见问题
1、Ubuntu下运行启动命令报错`-bash: ros2: command not found`

当前终端未设置ROS2环境，执行命令配置环境：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
```

在当前终端执行ros2命令确认当前终端环境是否生效：

```
# ros2
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
```

如果输出以上信息，说明ros2环境配置成功。

注意！对于每个新打开的终端，都需要重新设置ROS2环境。

2、终端无log信息输出

2.1 确认launch文件中的node是否都启动成功

重新开启一个终端（仅对Ubuntu系统有效），执行top命令查看launch文件中的node进程是否都在运行，否则使用ros2 run命令单独启动相关node确认启动失败原因。

2.2 查看每个node是否都有发布msg

根据launch文件中每个node配置的发布和订阅的topic名，使用ros2 topic echo（仅对Ubuntu系统有效）命令显示每个topic是否有消息发布，如果无，再确认没有发布的原因。

注意！如果运行ros2 topic命令失败，执行命令安装依赖：`pip3 install netifaces`

3、无法打开音频设备

3.1 确认音频设备接线是否正常

3.2 确认是否加载音频驱动

