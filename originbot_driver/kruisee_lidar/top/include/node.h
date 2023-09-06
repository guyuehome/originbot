//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>
#include <tuple>

#include "common/scripts.h"
#include "drivers/msg.h"
#include "filters/ismoother.h"
#include "filters/outlierfilter.h"
#include "drivers/iworker.h"

#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

struct NodeAttribute {
    std::string topic;
    int queue_size;
    std::string frame_id;
};

class Node {
public:
    static std::unique_ptr<Node> Create(const NodeAttribute &attribute);
    ~Node();

public:
    static rclcpp::Node::SharedPtr node_handle_;

    void SetWorkCmdHandler(WorkCmdHandler handler);

#ifdef SDK_TEST
    void HandleRawData(const LScan &scan);
#endif

    void HandleFilteredData(const LScan &scan);

    bool GetCtlRun();
    
    void ErrHandle(const std::string &err);

private:
    Node(const std::string &frame_id);
    bool Init(const std::string &topic, uint32_t queue_size);
    void CtlHandle(const std_msgs::msg::String::SharedPtr ctl);
    

private:    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_orig_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_err_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctl_subscriber_;

    bool ctl_run_;

#ifdef SDK_TEST
    //ros::Publisher tst_publisher_;
#endif

    std::string frame_id_;

    WorkCmdHandler work_cmd_handler_;
};
