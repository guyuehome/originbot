//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "node.h"
#include "filters/trailingfilter.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include <assert.h>

#include <map>
#include <algorithm>

#include <std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr Node::node_handle_;

Node::Node(const std::string &frame_id)
    : frame_id_(frame_id), ctl_run_(true), work_cmd_handler_(0) {}

Node::~Node() {}

std::unique_ptr<Node> Node::Create(const NodeAttribute &attribute)
{
    std::unique_ptr<Node> node(new Node(attribute.frame_id));
    if (node == nullptr)
        return nullptr;

    if (!node->Init(attribute.topic, attribute.queue_size))
        return nullptr;

    return node;
}

bool Node::Init(const std::string &topic, uint32_t queue_size)
{
    publisher_ = node_handle_->create_publisher<sensor_msgs::msg::LaserScan>(topic, queue_size);
    publisher_orig_ = node_handle_->create_publisher<sensor_msgs::msg::LaserScan>(topic + "_orig", queue_size);
    publisher_err_ = node_handle_->create_publisher<std_msgs::msg::String>(topic + "_err", queue_size);
#ifdef SDK_TEST
    //tst_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("/orig", queue_size);
#endif

    ctl_subscriber_ = node_handle_->create_subscription<std_msgs::msg::String>(topic + "_ctl", 5, std::bind(&Node::CtlHandle, this, std::placeholders::_1));

    return true;
}

void Node::CtlHandle(const std_msgs::msg::String::SharedPtr ctl)
{

    if(ctl->data == std::string("start"))
    {
        std::cout << "start lidar" << std::endl;
        ctl_run_ = true;
        if(work_cmd_handler_)
        {
            work_cmd_handler_(std::string("startlds$"));
        }
    }
    else if(ctl->data == std::string("stop"))
    {
        std::cout << "stop lidar" << std::endl;
        ctl_run_ = false;
        if(work_cmd_handler_)
        {
            work_cmd_handler_(std::string("holdlds$"));
        }
    }
}

void Node::ErrHandle(const std::string &err)
{
    std_msgs::msg::String err_msg;
    err_msg.data = std::string(err);
    publisher_err_->publish(err_msg);
}

bool Node::GetCtlRun()
{
    return ctl_run_;
}

void Node::SetWorkCmdHandler(WorkCmdHandler handler)
{
    work_cmd_handler_ = handler;
}

#ifdef SDK_TEST
void Node::HandleRawData(const LScan &scan)
{
    sensor_msgs::LaserScan orig;
    orig.header.frame_id = frame_id_;
    orig.header.stamp = rclcpp::Time(scan.stamp)+rclcpp::Duration(1);
    orig.angle_increment = scan.angle_increment;
    orig.angle_min = scan.min_angle;
    orig.angle_max = scan.max_angle;
    orig.range_min = scan.min_range;
    orig.range_max = scan.max_range;

    orig.ranges.resize(scan.ranges.size());
    orig.intensities.resize(scan.intensities.size());
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        orig.ranges[i] = scan.ranges[i];
        orig.intensities[i] = scan.intensities[i];
    }

    tst_publisher_.publish(orig);
}
#endif
#include<time.h>
struct timespec time1 = {0, 0};
void Node::HandleFilteredData(const LScan &scan)
{
    clock_gettime(CLOCK_REALTIME, &time1);
    sensor_msgs::msg::LaserScan orig;
    orig.header.frame_id = frame_id_;
    orig.header.stamp = rclcpp::Time(time1.tv_sec,time1.tv_nsec);
    orig.angle_increment = scan.angle_increment;
    orig.angle_min = scan.min_angle;
    orig.angle_max = scan.max_angle;
    orig.range_min = scan.min_range;
    orig.range_max = scan.max_range;
    orig.time_increment = scan.time_increment;

    orig.ranges.resize(scan.ranges.size());
    orig.intensities.resize(scan.intensities.size());
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        orig.ranges[i] = scan.ranges[i];
        orig.intensities[i] = scan.intensities[i];
    }

    if(scan.orig)
    {
        publisher_orig_->publish(orig);
       // publisher_->publish(orig);
    }
    else
    {
        publisher_->publish(orig);
    }
}

