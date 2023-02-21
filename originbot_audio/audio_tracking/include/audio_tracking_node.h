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

#ifndef AUDIO_TRACKING_NODE_H
#define AUDIO_TRACKING_NODE_H

#include <string>

#include "audio_msg/msg/smart_audio_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "include/audio_common.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
// #include "tf/tf.h"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "tf2/exceptions.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

using geometry_msgs::msg::Twist;
using SmartAudioCbType = std::function<void(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg)>;

using Odometry = nav_msgs::msg::Odometry;
using TfPoseInfoCb =
    std::function<void(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)>;

class AudioTrackingNode : public rclcpp::Node {
 public:
  AudioTrackingNode(const std::string &node_name,
                    SmartAudioCbType smart_audio_cb,
                    TfPoseInfoCb tf_pose_cb);

  ~AudioTrackingNode();

  void RobotCtl(const Twist &msg) const;

 private:
  // audio
  void SmartTopicCallback(
      const audio_msg::msg::SmartAudioData::ConstSharedPtr msg);

  std::string ai_msg_sub_topic_name_ = "/audio_smart";
  SmartAudioCbType smart_audio_cb_ = nullptr;
  rclcpp::Subscription<audio_msg::msg::SmartAudioData>::SharedPtr
      smart_subscription_ = nullptr;

  std::string twist_pub_topic_name_ = "/cmd_vel";
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher_ = nullptr;

  // tf 位姿信息
  std::string tf_msg_sub_topic_name_ = "/odom";
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tf_subscription_ =
      nullptr;
  void PoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  TfPoseInfoCb tf_pose_cb_ = nullptr;
};

#endif  // AUDIO_TRACKING_NODE_H
