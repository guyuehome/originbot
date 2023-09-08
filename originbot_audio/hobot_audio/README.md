# 功能介绍

地平线智能语音算法采用本地离线模式，订阅音频数据后送给BPU处理，然后发布**唤醒、命令词识别**、**声源定位DOA角度信息**以及**语音ASR识别结果**等消息。智能语音功能的实现对应于TogetheROS.Bot的**hobot_audio** package，适用于地平线RDK配套的环形四麦阵列。

应用场景：智能语音算法能够识别音频中的唤醒词以及自定义的命令词，并将语音内容解读为对应指令或转化为文字，可实现语音控制以及语音翻译等功能，主要应用于智能家居、智能座舱、智能穿戴设备等领域。

# 物料清单

| 机器人名称 | 生产厂家 | 参考链接                                                        |
| :--------- | -------- | --------------------------------------------------------------- |
| RDK X3     | 多厂家   | [点击跳转](https://developer.horizon.cc/rdkx3)                  |
| 麦克风板   | 微雪电子 | [点击跳转](https://www.waveshare.net/shop/Audio-Driver-HAT.htm) |

# 使用方法

## 准备工作

在体验之前，需要具备以下基本条件：

- 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像
- 音频板正确连接到RDK X3

连接步骤：

1. 将麦克风板连接到地平线RDK X3 40PIN GPIO 接口上，连接后实物如下图：

    ![circle_mic_full](./imgs/circle_mic_full.png)

2. 接上电源，网线等。

    将地平线RDK与麦克风阵列接好之后上电，在串口上使用指令`i2cdetect -r -y 0`可以检查设备的接入情况，若成功接好，默认可以在I2C上读取到三个地址。如下图：

    ![detect_mic](./imgs/detect_mic.jpg)

    若没检测到，请重新检查设备的连接。

## 安装功能包

启动RDK X3后，通过终端SSH或者VNC连接机器人，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-hobot-audio
```

## 运行智能语音程序

智能语音功能支持对原始音频进行降噪之后进行ASR识别，默认的唤醒词和命令词定义在智能语音功能代码模块根目录下 *config/hrsc/cmd_word.json* 文件，默认为：

```json
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

`cmd_word`第一个词为唤醒词，后续为命令词，这些词用户都可以根据需要配置，若更改唤醒词效果可能会与默认的唤醒词命令词效果有差异。推荐唤醒词以及命令词使用中文，最好是朗朗上口的词语，且词语长度推荐使用3~5个字。

另外，智能语音功能支持输出声源定位的DOA角度信息，单位为角度，环形麦克风阵列取值范围：0度\~360度。

角度的相对位置关系与麦克风的安装位置强相关，环形麦克风阵列DOA角度示意图如下：

![doa_circle](./imgs/doa_circle.jpg)

地平线RDK板端运行hobot_audio package：

1. 拷贝配置文件和加载音频驱动

    ```shell
    # 从tros.b的安装路径中拷贝出运行示例需要的配置文件，若已拷贝过则可忽略
    cp -r /opt/tros/lib/hobot_audio/config/ .

    # 加载音频驱动，设备启动之后只需要加载一次
    bash config/audio.sh
    ```

2. 配置音频设备号以及是否发布ASR结果

    - 音频设备号通过 *config/audio_config.json* 中 `micphone_name`字段设置，默认为"hw:0,0"。若加载音频驱动时无其他音频设备连接，则无需修改该字段。若加载音频驱动时有其他音频设备连接，例如USB麦克风或带麦克风功能的USB摄像头，则需要修改该字段为对应的设备号，以"hw:0,0"为例，表示音频设备Card0 Device0。
    - 是否发布ASR结果通过配置文件 *config/audio_config.json* 中 `asr_mode`字段设置。该字段默认值为`0`，表示不发布ASR结果。若要发布ASR结果，需要将该字段改为`1`或`2`，`1`表示唤醒后进行一次ASR识别并发布结果，`2`表示一直进行ASR识别并发布结果。

3. 配置tros.b环境和启动应用

    ```shell
    # 配置tros.b环境
    source /opt/tros/setup.bash

    # 屏蔽调式打印信息
    export GLOG_minloglevel=3

    #启动launch文件
    ros2 launch hobot_audio hobot_audio.launch.py
    ```

4. 结果分析

    地平线RDK板端运行终端输出如下信息：

    ```text
    alsa_device_init, snd_pcm_open. handle((nil)), name(hw:0,0), direct(1), mode(0)
    snd_pcm_open succeed. name(hw:0,0), handle(0x557d6e4d00)
    Rate set to 16000Hz (requested 16000Hz)
    Buffer size range from 16 to 20480
    Period size range from 16 to 10240
    Requested period size 512 frames
    Periods = 4
    was set period_size = 512
    was set buffer_size = 2048
    alsa_device_init. hwparams(0x557d6e4fa0), swparams(0x557d6e5210)
    ```

    以上log显示，音频设备初始化成功，并且打开了音频设备，可正常采集音频。

    当人依次在麦克风旁边说出“地平线你好”、“向前走”、“向左转”、“向右转”、“向后退”命令词，语音算法sdk经过智能处理后输出识别结果，log显示如下：

    ```text
    recv hrsc sdk event wakeup success, wkp count is 1
    [WARN] [1657869437.600230208] [hobot_audio]: recv event:0
    recv hrsc sdk doa data: 100
    recv hrsc sdk command data: 向前走
    [WARN] [1657869443.870029101] [hobot_audio]: recv cmd word:向前走
    recv hrsc sdk doa data: 110
    recv hrsc sdk command data: 向左转
    [WARN] [1657869447.623147766] [hobot_audio]: recv cmd word:向左转
    recv hrsc sdk doa data: 100
    recv hrsc sdk command data: 向右转
    [WARN] [1657869449.865822772] [hobot_audio]: recv cmd word:向右转
    recv hrsc sdk doa data: 110
    recv hrsc sdk command data: 向后退
    [WARN] [1657869452.313969277] [hobot_audio]: recv cmd word:向后退
    ```

    log显示，识别到语音命令词“向前走”、“向左转”、“向右转”、“向后退”，并且输出DOA的角度信息，如“recv hrsc sdk doa data: 110”字段表示DOA角度为110度。

    hobot_audio默认发布的智能语音消息话题名为：*/audio_smart*，在另一个终端执行使用`ros2 topic list`命令可以查询到此topic信息：

    ```bash
    $ ros2 topic list
    /audio_smart
    ```

    若开启发布ASR结果，发布消息话题为：*/audio_asr*，`ros2 topic list`结果为：

    ```bash
    $ ros2 topic list
    /audio_smart
    /audio_asr
    ```

# 接口说明

## 话题

| 名称         | 消息类型                                                                                                               | 说明                               |
| ------------ | ---------------------------------------------------------------------------------------------------------------------- | ---------------------------------- |
| /audio_smart | [audio_msg/msg/SmartAudioData](https://github.com/HorizonRDK/hobot_msgs/blob/develop/audio_msg/msg/SmartAudioData.msg) | 发布智能语音处理后的数据和智能结果 |
| /audio_asr   | std_msgs/msg/String                                                                                                    | 发布ASR识别结果                    |

## 参数

| 参数名               | 类型        | 解释               | 是否必须 | 支持的配置       | 默认值       |
| -------------------- | ----------- | ------------------ | -------- | ---------------- | ------------ |
| config_path          | std::string | 配置文件路径       | 否       | 根据实际情况配置 | ./config     |
| audio_pub_topic_name | std::string | 音频智能帧发布话题 | 否       | 根据实际情况配置 | /audio_smart |
| asr_pub_topic_name   | std::string | ASR结果发布话题    | 否       | 根据实际情况配置 | /audio_asr   |

audio_config.json配置文件参数说明：

| 参数名               | 类型   | 解释                                                                                                           | 是否必须 | 支持的配置                                                      | 默认值   |
| -------------------- | ------ | -------------------------------------------------------------------------------------------------------------- | -------- | --------------------------------------------------------------- | -------- |
| micphone_enable      | int    | 是否使能麦克风                                                                                                 | 是       | 0/1                                                             | 1        |
| micphone_name        | string | 麦克风设备号                                                                                                   | 否       | 根据实际情况配置                                                | "hw:0,0" |
| micphone_rate        | int    | 麦克风采样率                                                                                                   | 否       | 16000                                                           | 16000    |
| micphone_chn         | int    | 麦克风通道数                                                                                                   | 是       | 根据实际硬件配置                                                | 8        |
| micphone_buffer_time | int    | 环形缓冲区长度时间，单位微妙                                                                                   | 否       | 根据实际情况配置                                                | 0        |
| micphone_nperiods    | int    | 周期时间，单位微妙                                                                                             | 否       | 4                                                               | 4        |
| micphone_period_size | int    | 音频包大小                                                                                                     | 否       | 根据实际情况配置                                                | 512      |
| voip_mode            | int    | 是否是voip模式，若配置成voip模式，则发布降噪后的音频，并且不支持ASR识别功能。即音频降噪与ASR识别模式互斥。     | 是       | 0/1                                                             | 0        |
| mic_type             | int    | 麦克风阵列类型                                                                                                 | 否       | 0/1，0：环形麦克风阵列，1：线形麦克风阵列                       | 0        |
| asr_mode             | int    | asr模式                                                                                                        | 否       | 0/1/2，0: 不输出asr结果， 1: 唤醒后输出asr， 2: 一直开启asr输出 | 0        |
| asr_channel          | int    | asr通道选择                                                                                                    | 否       | 0/1/2/3，当asr_mode为2时，选择输出具体通道的ars结果             | 3        |
| save_audio           | int    | 是否保存音频数据，包括麦克风采集的原始音频和voip模式下算法输出的降噪后的音频。音频默认保存在程序运行当前目录。 | 否       | 0/1                                                             | 0        |

cmd_word.json

此配置文件配置语音智能分析部分的唤醒词以及命令词，配置文件的第一项为唤醒词，后面的是命令词。默认配置文件配置如下：

```json
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

# 原理简介

智能语音hobot_audio package开始运行之后，会从麦克风阵列采集音频，并且将采集到的音频数据送入语音智能算法SDK模块做智能处理，输出唤醒事件、命令词、ASR结果等智能信息，其中唤醒事件、命令词通过`audio_msg::msg::SmartAudioData`类型消息发布，ASR结果通过`std_msgs::msg::String`类型消息发布。

具体流程如下图：

![hobot_audio](./imgs/hobot_audio.jpg)

# 常见问题

1. 无法打开音频设备？

- 确认音频设备连接是否正常
- 确认是否加载音频驱动
- 确认配置文件 *config/audio_config.json* 中`micphone_name`字段设置为正确的音频设备号
