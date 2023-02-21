// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/audio_control_node.h"

#include <fstream>
#include <string>

AudioControlNode::AudioControlNode(const std::string& node_name,
                                   SmartCbType smart_cb)
    : Node(node_name), smart_cb_(smart_cb) {
  this->declare_parameter<std::string>("twist_pub_topic_name",
                                       twist_pub_topic_name_);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);

  this->get_parameter<std::string>("twist_pub_topic_name",
                                   twist_pub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);
  std::stringstream ss;
  ss << "Parameter:"
     << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
     << "\n twist_pub_topic_name: " << twist_pub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("AudioControlNode"), "%s", ss.str().c_str());

  smart_subscription_ =
      this->create_subscription<audio_msg::msg::SmartAudioData>(
          ai_msg_sub_topic_name_,
          10,
          std::bind(&AudioControlNode::SmartTopicCallback,
                    this,
                    std::placeholders::_1));
  twist_publisher_ = this->create_publisher<Twist>(twist_pub_topic_name_, 10);
}

AudioControlNode::~AudioControlNode() {
  Twist msg;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;;
  twist_publisher_->publish(msg);
  RCLCPP_WARN(rclcpp::get_logger("AudioControlNode"), "Publish stop cmd");
}

void AudioControlNode::RobotCtl(const Twist& msg) const {
  // std::stringstream ss;
  // ss << "RobotCtl " << msg.angular.x
  // << " " << msg.angular.y
  // << " " << msg.angular.z;
  // static std::ofstream ofs("dump.log");
  // ofs << ss.str() << std::endl;
  twist_publisher_->publish(msg);
}

void AudioControlNode::SmartTopicCallback(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr msg) {
  std::stringstream ss;
  ss << "Recved audio msg"
     << ", frame type: " << msg->frame_type.value
     << ", event_type: " << msg->event_type.value
     << ", cmd_word: " << msg->cmd_word
     << ", doa_theta: " << msg->doa_theta << "\n";
  RCLCPP_INFO(rclcpp::get_logger("AudioControlNode"), "%s", ss.str().c_str());

  if (smart_cb_) {
    smart_cb_(msg);
  }
}
