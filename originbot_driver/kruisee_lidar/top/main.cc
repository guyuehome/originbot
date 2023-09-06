//
// Copyright (c) 2022 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <iostream>

#include "common/loader.h"
#include "common/scripts.h"
#include "common/configurator.h"
#include "drivers/uart.h"
#include "drivers/ethernet.h"
#include "drivers/lidar.h"
#include "filters/outlierfilter.h"
#include "filters/smoother.h"

#include "sdkcore.h"

#include "node.h"
#include "err.h"

namespace {

bool GetNodeAttribute(NodeAttribute &attribute, std::string &sdk_core_path)
{
    Node::node_handle_->declare_parameter<std::string>("sdk_core_path", "");
    if (!Node::node_handle_->get_parameter("sdk_core_path", sdk_core_path)) {
        std::cerr << "Failed to load sdk core path" << std::endl;
        return false;
    }
    std::cout << "sdk_core_path:" << sdk_core_path << std::endl;

    Node::node_handle_->declare_parameter<std::string>("topic", "");
    if (!Node::node_handle_->get_parameter("topic", attribute.topic)) {
        std::cerr << "Failed to load topic" << std::endl;
        return false;
    }
    std::cout << "topic:" << attribute.topic << std::endl;
    
    Node::node_handle_->declare_parameter<int>("queue_size", 100);
    if (!Node::node_handle_->get_parameter("queue_size", attribute.queue_size)) {
        std::cerr << "Failed to load ros topic queue size value" << std::endl;
        return false;
    }
    std::cout << "queue_size:" << attribute.queue_size << std::endl;
    if (attribute.queue_size < 0) {
        std::cerr << "Illegal data value" << std::endl;
        return false;
    }

    Node::node_handle_->declare_parameter<std::string>("frame_id", "");
    if (!Node::node_handle_->get_parameter("frame_id", attribute.frame_id)) {
        std::cerr << "Failed to load frame_id" << std::endl;
        return false;
    }
    std::cout << "frame_id:" << attribute.frame_id << std::endl;
    

    return true;
}

// 预定义SDK CORE 配置文件路径
const std::string kSDKCOREConfigDefaultPath{ SDK_CFG_FILES_DIR };

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    Node::node_handle_ = rclcpp::Node::make_shared("kruisee_lidar_node");

    std::string sdk_core_path;
    NodeAttribute attribute;
    if (!GetNodeAttribute(attribute, sdk_core_path)) {
        std::cerr << "Failed to get ros node attribute" << std::endl;
        return ERR_INVALID_ARGS;
    }

    auto sdkcore = SdkCore::Create(std::vector<std::string>{ kSDKCOREConfigDefaultPath, sdk_core_path.append("/config") }, CFG_FILE);
    if (sdkcore == nullptr) {
        std::cerr << "Failed to create sdk core" << std::endl;
        return ERR_NOT_LAUNCH;
    }

    auto node = Node::Create(attribute);
    if (node == nullptr) {
        std::cerr << "Failed to create ROS Node" << std::endl;
        return ERR_NOT_MEMORY;
    }

#ifdef SDK_TEST
    sdkcore->RegisterDataDistributor(SdkCore::Raw, std::bind(&Node::HandleRawData, node.get(), std::placeholders::_1));
#endif

    node->SetWorkCmdHandler(std::bind(&SdkCore::Write, sdkcore.get(), std::placeholders::_1));

    sdkcore->RegisterCtlRunHandler(std::bind(&Node::GetCtlRun, node.get()));
    sdkcore->RegisterErrHandler(std::bind(&Node::ErrHandle, node.get(), std::placeholders::_1));

    sdkcore->RegisterDataDistributor(SdkCore::Filtered, std::bind(&Node::HandleFilteredData, node.get(), std::placeholders::_1));

    sdkcore->Run();

    rclcpp::spin(Node::node_handle_);
	return NO_ERROR;
}
