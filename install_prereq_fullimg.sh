#!/bin/sh
# Install the prerequisites for the ROS exploring code
# 清理 第三方软件源列表，添加ros2、tros软件源
# 安装 ros2-foxy 、升级tros和其他可升级软件
# 安装 originbot 依赖包

sudo rm /etc/apt/sources.list.d/*

sudo apt update && sudo apt install curl gnupg2
echo -e "\e[32m 添加ros软件源 \e[0m"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "\e[32m 安装ros2-foxy \e[0m"
sudo apt update && sudo apt install ros-foxy-ros-base

sudo apt update && sudo apt install curl
echo -e "\e[32m 添加tros软件源 \e[0m"
sudo curl -sSL http://sunrise.horizon.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://sunrise.horizon.cc/ubuntu-rdk-sim focal main" | sudo    tee /etc/apt/sources.list.d/sunrise.list > /dev/null

sudo apt update
echo -e "\e[32m 更新可升级的包(tros) \e[0m"
sudo apt upgrade

echo -e "\e[32m 安装originbot 依赖包 \e[0m"
sudo apt -y install \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  ros-foxy-slam-toolbox \
  ros-foxy-cartographer-ros \
  ros-foxy-teleop-twist-keyboard \
  ros-foxy-robot-localization \
  cloud-utils \
  ros-foxy-rmw-cyclonedds-cpp \
  ros-foxy-teleop-twist-joy \
  ros-foxy-joy-linux \
  ros-foxy-cv-bridge \
  ros-foxy-xacro \
  ros-foxy-robot-state-publisher \
  ros-foxy-joint-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-nav2* \
  libboost-python1.71.0 

# 清理旧包
sudo apt autoremove -y

# 设置pip清华源
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# 安装colcon-clean 工作区清理工具
pip install colcon-clean
# 安装opencv-contrib
pip install opencv-contrib-python==4.8.1.78
# 安装torch
pip install torch==2.2.0
# 安装pyclipper
pip install pyclipper==1.3.0.post5

# 从TogetheROS的安装路径中拷贝出运行示例需要的配置文件
echo -e "\e[32m 拷贝配置文件到工作空间 \e[0m"
cp -r /opt/tros/lib/mono2d_body_detection/config/ /userdata/dev_ws/
cp -r /opt/tros/lib/hand_lmk_detection/config/ /userdata/dev_ws/
cp -r /opt/tros/lib/hand_gesture_detection/config/ /userdata/dev_ws/

# 编译安装硬件驱动
echo -e "\e[32m 编译安装硬件驱动 \e[0m"

# 检查qpOASES目录是否存在
if [ -d "/userdata/dev_ws/src/originbot/originbot_driver/qpOASES/" ]; then
    cd /userdata/dev_ws/src/originbot/originbot_driver/qpOASES/
    mkdir build && cd build
    cmake ..
    sudo make && sudo make install
    cd .. && sudo rm -r build/
else
    echo "目录 /userdata/dev_ws/src/originbot/originbot_driver/qpOASES/ 不存在，跳过编译安装。"
fi

# 检查serial_ros2目录是否存在
if [ -d "/userdata/dev_ws/src/originbot/originbot_driver/serial_ros2/" ]; then
    cd /userdata/dev_ws/src/originbot/originbot_driver/serial_ros2/
    make && make install
    make clean
else
    echo "目录 /userdata/dev_ws/src/originbot/originbot_driver/serial_ros2/ 不存在，跳过编译安装。"
fi
