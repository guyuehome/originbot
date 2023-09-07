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

#ifndef ORIGINBOT_MPC_H
#define ORIGINBOT_MPC_H

#include "loadPath.h"

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

#include "qpOASES.hpp"

//此demo借鉴Apollo，其使用的是单车模型，此处我们采集输出为转弯数据用于模拟差动模型角速度
//参考链接：https://blog.csdn.net/u013914471/article/details/83824490

using namespace std;
using Matrix = Eigen::MatrixXd;
using std::placeholders::_1;

class MpcController : public rclcpp::Node {
public :
    MpcController(string nodeName);
    virtual ~MpcController(){};

private :
    void initParm();
    void loadPath();
    int findCloestIndex(const double x,const double y);
    //计算误差模块
    void computeStateError(const double x,
                            const double y, 
                            const double v, 
                            const double heading, 
                            const double heading_rate, 
                            const int index);
    //计算状态空间矩阵
    void computeStateMatrix(const double linear_velocity);
    //mpc求解
    double mpcSolver();
    //qp解算器
    double solveQP(const Matrix& H_matrix,
                            const Matrix& G_matrix, 
                            const Matrix& inequality_matrix, 
                            const Matrix& inequality_lower_boundary_matrix,
                            const Matrix& inequality_upper_boundary_matrix,
                            const Matrix& lower_bound, 
                            const Matrix& upper_bound, 
                            const int numOfIter);
    //前馈控制
    double feedForwardControl(const double v, const double kappa);
    //最终控制
    double computeSteering(const double x,
                            const double y,
                            const double v,
                            const double heading, 
                            const double heading_rate);
    double normalizeAngle(const double angle);

    void pubPath();
    double calSteeringAngle(double alpha,double ld) ;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    string waypoints_path;
    //小车物理属性
    //控制时间域
    double ts_;
    //左前轮质量 
    double mass_front_left_wheel;
    //右前轮质量
    double mass_front_right_wheel;
    //左后轮质量
    double mass_rear_left_wheel;
    //右后轮质量
    double mass_rear_right_wheel;
    //车身质量
    double mass_body;
    //整车质量
    double mass_;
    //车辆轴长
    double wheel_base;
    //前轮轴长
    double lf_;
    //后轮轴长
    double lr_;
    //前轮侧偏刚度
    double cf_;
    //后轮侧偏刚度
    double cr_;
    //z轴旋转惯量
    double izz_;
    //最大转向角
    double max_steer_angle;
    //期望速度
    double v_ref;
    //预测步长
    double horizon_;
    //mpc最大迭代步长
    double numOfIter;

    //状态空间矩阵
    //状态大小
    //ed,ed_dot,e_psi,e_psi_dot
    const int basic_state_size = 4;
    //控制量大小
    const int basic_control_size = 1;
    //控制权重矩阵
    Matrix r_;
    //状态权重矩阵
    Matrix q_;
    //状态矩阵
    Matrix state_mat_;
    //控制矩阵
    Matrix control_mat_;
    //状态空间矩阵
    Matrix a_mat_;
    //离散状态空间矩阵
    Matrix a_mat_d_;
    //控制空间矩阵
    Matrix b_mat_;
    //离散控制空间矩阵
    Matrix b_mat_d_;
    //扰动空间矩阵
    Matrix c_mat_;
    //离散扰动空间矩阵
    Matrix c_mat_d_;
    
    //状态误差
    double lateral_error;
    double heading_error;
    double lateral_error_rate;
    double heading_error_rate;

    vector<double> xr;
    vector<double> yr;
    vector<double> yawr;
    vector<double> kappar;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    
};

#endif //ORIGINBOT_MPC_H 