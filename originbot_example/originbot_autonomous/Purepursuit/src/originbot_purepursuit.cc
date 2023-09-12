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

#include "originbot_purepursuit.h"

PurePursuit::PurePursuit(string nodename):Node(nodename) 
{
    RCLCPP_INFO(this->get_logger(),"Starting Pure Pursuit");

    initParm();
    loadPath();
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    pub_path = this->create_publisher<nav_msgs::msg::Path>("base_path",1);
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom",1,bind(&PurePursuit::odom_callback,this,_1));
}

void PurePursuit::initParm() 
{
    track = 0.11;
    ld0 = 0.5;
    kv = 0.1;
    string pkg_name = "originbot_autonomous";
    string waypoints_name = "path.csv";
    string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;

    RCLCPP_INFO(this->get_logger(),"waypoints file is loaded at directory: %s",waypoints_path.c_str());
}

//加载路径点信息
void PurePursuit::loadPath() 
{
    WaypointLoader wp(waypoints_path);
    //检查文件是否存在或是否为空
    bool isLoaded = wp.loadWayPoints();
    if (!isLoaded) {
        RCLCPP_ERROR(this->get_logger(),"File is not exist or file is empty!");
        exit(1);
    }
    vector<vector<double>> wp_temp = wp.getWayPoints();
    for(vector<vector<double>>::const_iterator it=wp_temp.begin();it!=wp_temp.end();it++) {
        //获取路径点和位姿信息
        xr.push_back((*it)[0]);     //x
        yr.push_back((*it)[1]);     //y
        yawr.push_back((*it)[2]);   //yaw
        kappar.push_back((*it)[3]);      //kappa
    }
}

void PurePursuit::pubPath() 
{
    geometry_msgs::msg::PoseStamped pose;
    nav_msgs::msg::Path base_path;
    base_path.header.stamp = this->get_clock()->now();
    base_path.header.frame_id = "odom";
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "odom";
    for(int i=0;i<(int)xr.size();i++) {   
        pose.pose.position.x = xr[i];
        pose.pose.position.y = yr[i];
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0,0,yawr[i]);
    
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        base_path.poses.push_back(pose);
    }
    pub_path->publish(base_path);
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) 
{
    pubPath();
    int index = findCloestindex(odom);
    double x_prev = xr[index];
    double y_prev = yr[index];

    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double v = odom->twist.twist.linear.x;
    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    );

    double roll,pitch,yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    double alpha = atan2(y_prev-y,x_prev-x)-yaw;
    double ld = kv*v+ld0;
    double steer = calSteeringAngle(alpha,ld);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = steer;
    pub_cmd->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(),"Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);
}

int PurePursuit::findCloestindex(const nav_msgs::msg::Odometry::SharedPtr odom) 
{   
    int index;
    vector<double> dist;
    double linear_x = odom->twist.twist.linear.x;
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;

    for (int i =0;i<(int)xr.size();i++) {
        double dist_temp = pow(xr[i]-x,2) + pow(yr[i]-y,2);
        dist.push_back(dist_temp);
    }
    auto smallest = min_element(dist.begin(),dist.end());
    index = distance(dist.begin(),smallest);

    //公式 : ld=l+kvld=l+kv (过于依赖前视距离的选取，可以采用动态前视距离ld=l+kvld=l+kv其中l、kl、k为系数，根据速度调整前视距离)
    double ld = kv*linear_x+ld0;
    double ld_now = 0;

    while(ld_now<ld && index<=(int)xr.size()) {
        double dx_ref = xr[index+1] - xr[index];
        double dy_ref = yr[index+1] - yr[index];
        ld_now += sqrt(pow(dx_ref,2)+pow(dy_ref,2));
        index++;
    }
    return index;
}

double PurePursuit::calSteeringAngle(double alpha,double ld) 
{
    double steer = atan2(2*track*sin(alpha),ld);

    if (steer > M_PI) {
        steer -= M_PI*2;
    } else if (steer < (-M_PI)) {
        steer += M_PI*2;
    }
    return steer;
}

int main(int argc, char *argv[]) 
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);

    // 创建purepursuit节点，通过spin不断查询订阅话题
    rclcpp::spin(std::make_shared<PurePursuit>("purepursuit"));
    
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();

    return 0;
}

