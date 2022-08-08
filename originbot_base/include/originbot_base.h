#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <algorithm>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define ORIGINBOT_WHEEL_TRACK (0.1)

typedef struct _RECEIVE_DATA_
{
    uint8_t rx[9];              //定义长度
    uint8_t flag_stop;          //标识位
    unsigned char frame_header; //帧头
    float left_dirct;           //左电机速度方向
    float left_speed;           //左电机速度
    float right_dirct;          //右电机速度方向
    float right_speed;          //右电机速度
    unsigned char flag_test;    //校验位
    unsigned char frame_tail;   //帧尾
} RECEIVE_DATA;

class originbot_driver : public rclcpp::Node
{
public:
    originbot_driver();
    ~originbot_driver();

    void driver_loop();
    
private:
    bool get_sensor_data();
    int  find_data(uint8_t ar[], int n, int element);
    float imu_conversion(uint8_t data_high, uint8_t data_low);

    void odom_publisher(float vx, float vth);
    void imu_publisher(float linear_acceleration[3], float angular_velocity[3], float euler[3]);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    serial::Serial serial_;
    rclcpp::Time current_time_, last_time_;
    float x_, y_, th_;
    uint8_t sensor_data_raw_[53];

    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};