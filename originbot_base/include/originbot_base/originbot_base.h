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

#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <thread>
#include <algorithm>
#include <csignal>
#include <stdlib.h>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "originbot_msgs/msg/originbot_status.hpp"
#include "originbot_msgs/srv/originbot_led.hpp"
#include "originbot_msgs/srv/originbot_buzzer.hpp"
#include "originbot_msgs/srv/originbot_pid.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define ORIGINBOT_WHEEL_TRACK  (0.11)

// originbot协议数据格式
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    uint8_t data[6];
    uint8_t check;
    uint8_t tail;
} DataFrame;

// IMU数据结构体
typedef struct {
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_x;
    float angular_y;
    float angular_z;
    float roll;
    float pitch;
    float yaw;
} DataImu;

// 机器人状态结构体
typedef struct {
    float battery_voltage;
    bool buzzer_on;
    bool led_on;
} RobotStatus;

// 帧ID枚举
enum {
    FRAME_ID_MOTION       = 0x01,
    FRAME_ID_VELOCITY     = 0x02,
    FRAME_ID_ACCELERATION = 0x03,
    FRAME_ID_ANGULAR      = 0x04,
    FRAME_ID_EULER        = 0x05,
    FRAME_ID_SENSOR       = 0x06,
    FRAME_ID_HMI          = 0x07,
};

class OriginbotBase : public rclcpp::Node
{
public:
    OriginbotBase(std::string nodeName);
    ~OriginbotBase();
    
private:
    // 读取原始数据
    void readRawData();
    // 检查数据帧
    bool checkDataFrame(DataFrame &frame);
    // 处理数据帧
    void processDataFrame(DataFrame &frame);

    // 处理速度数据
    void processVelocityData(DataFrame &frame);
    // 处理角速度数据  
    void processAngularData(DataFrame &frame);
    // 处理加速度数据
    void processAccelerationData(DataFrame &frame);
    // 处理欧拉角数据
    void processEulerData(DataFrame &frame);
    // 处理传感器数据
    void processSensorData(DataFrame &frame);

    // IMU数据转换
    double imu_conversion(uint8_t data_high, uint8_t data_low);
    // IMU校准
    bool   imu_calibration();
    // 角度转弧度
    double degToRad(double deg);

    // 发布里程计数据
    void odom_publisher(float vx, float vth);
    // 发布IMU数据
    void imu_publisher();

    // 蜂鸣器控制
    bool buzzer_control(bool on);
    // LED控制
    bool led_control(bool on);

    // 速度控制回调函数
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // 蜂鸣器服务回调函数
    void buzzer_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Request>  request,
                               std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Response> response);
    // LED服务回调函数                           
    void led_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotLed::Request>  request,
                            std::shared_ptr<originbot_msgs::srv::OriginbotLed::Response> response);
    // 左轮PID服务回调函数
    void left_pid_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request>  request,
                            std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> response);
    // 右轮PID服务回调函数
    void right_pid_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request>  request,
                            std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> response);

    // 100ms定时器回调函数
    void timer_100ms_callback();

private:
    serial::Serial serial_;
    rclcpp::Time current_time_;
    float odom_x_=0.0, odom_y_=0.0, odom_th_=0.0;

    float correct_factor_vx_ = 1.0;
    float correct_factor_vth_ = 1.0;    
    
    std::shared_ptr<std::thread> read_data_thread_;
    DataImu imu_data_;
    RobotStatus robot_status_;

    rclcpp::TimerBase::SharedPtr timer_100ms_;
    bool auto_stop_on_ = true;
    bool pub_odom_ = false;
    bool use_imu_ = false;
    unsigned int auto_stop_count_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<originbot_msgs::msg::OriginbotStatus>::SharedPtr status_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
   
    rclcpp::Service<originbot_msgs::srv::OriginbotBuzzer>::SharedPtr buzzer_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotLed>::SharedPtr led_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr left_pid_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr right_pid_service_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string port_name_;
};