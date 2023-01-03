#ifndef ORIGINBOT_TELOP_H
#define ORIGINBOT_TELOP_H

#include <iostream>
#include <unistd.h>
#include <termios.h>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/poll.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <boost/thread/thread.hpp>

//定义键盘按键键值
#define KEYCODE_A   (0x61)
#define KEYCODE_C   (0x63)
#define KEYCODE_D   (0x64)
#define KEYCODE_E   (0x65)
#define KEYCODE_G   (0x67)
#define KEYCODE_H   (0x68)
#define KEYCODE_J   (0x6a)
#define KEYCODE_K   (0x6b)
#define KEYCODE_L   (0x6c)
#define KEYCODE_Q   (0x71)
#define KEYCODE_S   (0x73)
#define KEYCODE_W   (0x77)
#define KEYCODE_Z   (0x7A)

#define MAX_SPEED_LINEARE_X   (0.5)
#define MAX_SPEED_ANGULAR_Z   (0.5)

class OriginbotTeleop : public rclcpp::Node {
    private:
        float _speed_linear_x;
        float _speed_angular_z;
        geometry_msgs::msg::Twist cmdvel_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
        struct termios initial_settings, new_settings;
        int kfd = 0;
    public:
        OriginbotTeleop(std::string nodeName);
        ~OriginbotTeleop(){tcsetattr(0, TCSANOW, &new_settings);};
        void showMenu();
        void stopRobot();
        void originbot_teleopkeyboardLoop();
};

#endif //ORIGINBOT_TELOP_H