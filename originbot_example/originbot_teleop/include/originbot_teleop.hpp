/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef ORIGINBOT_TELOP_H
#define ORIGINBOT_TELOP_H

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/thread/thread.hpp>
#include <dirent.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// 定义键盘按键键值
#define KEYCODE_A (0x61)
#define KEYCODE_C (0x63)
#define KEYCODE_D (0x64)
#define KEYCODE_E (0x65)
#define KEYCODE_G (0x67)
#define KEYCODE_H (0x68)
#define KEYCODE_I (0x69)
#define KEYCODE_J (0x6a)
#define KEYCODE_K (0x6b)
#define KEYCODE_L (0x6c)
#define KEYCODE_Q (0x71)
#define KEYCODE_S (0x73)
#define KEYCODE_W (0x77)
#define KEYCODE_Z (0x7A)

#define MAX_SPEED_LINEARE_X (0.5)
#define MAX_SPEED_ANGULAR_Z (0.5)

class OriginbotTeleop : public rclcpp::Node
{
  private:
    float _speed_linear_x;
    float _speed_angular_z;
    geometry_msgs::msg::Twist cmdvel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    struct termios initial_settings, new_settings;
    int kfd = 0;

  public:
    OriginbotTeleop(std::string nodeName);
    ~OriginbotTeleop()
    {
        tcsetattr(0, TCSANOW, &new_settings);
    };
    void showMenu();
    void stopRobot();
    void teleopKeyboardLoop();
};

#endif // ORIGINBOT_TELOP_H