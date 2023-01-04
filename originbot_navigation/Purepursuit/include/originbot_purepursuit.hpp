#ifndef ORIGINBOT_PUREPURSUIT_H
#define ORIGINBOT_PUREPURSUIT_H

#include <iostream>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
//自定义头文件
#include "loadpath.h"

using namespace std;
using Matrix = Eigen::MatrixXd;
using std::placeholders::_1;

class PurePursuit : public rclcpp::Node{
    public :
        PurePursuit(string nodeName);
        ~PurePursuit(){};

    private :
        void InitParm();
        void LoadPath();
        int FindCloestindex(const nav_msgs::msg::Odometry::SharedPtr odom);
        void PubPath();
        double  calSteeringAngle(double alpha,double ld) ;
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
        double track = 0.11;
        double ld0 = 0.5;
        double kv = 0.1;
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        string waypoints_path;
};




#endif //ORIGINBOT_PUREPURSUIT_H 