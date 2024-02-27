#!/bin/sh
# Install the prerequisites for the ROS exploring code
# 升级 tros和其他可升级软件
# 安装 ros2-foxy
# 安装 originbot 依赖包

echo -e "\e[32m 更新可升级的包(tros) \e[0m"
sudo apt update && sudo apt upgrade
echo -e "\e[32m 安装originbot 依赖包 \e[0m"
sudo apt -y install \
  git \
  ros-foxy-ros-base \
  ros-foxy-demo-nodes-cpp \
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
