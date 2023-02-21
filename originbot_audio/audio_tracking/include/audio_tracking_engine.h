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

#ifndef AUDIO_TRACKING_ENGINE_H
#define AUDIO_TRACKING_ENGINE_H

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "audio_msg/msg/smart_audio_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "include/audio_tracking_node.h"
#include "include/param_node.h"
#include "rclcpp/rclcpp.hpp"

using geometry_msgs::msg::Twist;
using rclcpp::NodeOptions;
using nav_msgs::msg::Odometry;

class AudioTrackingEngine {
 public:
  static std::shared_ptr<AudioTrackingEngine> Instance();
  ~AudioTrackingEngine();

 public:
  void FeedAudioSmart(
      const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg);
  void PoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes() {
    std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
    node_ptrs.push_back(param_node_);
    node_ptrs.push_back(audio_tracking_node_);
    return node_ptrs;
  }

 private:
  AudioTrackingEngine();
  void ProcessAudioSmart(
      const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg);
  int ProcessDoa(const float doa_theta);
  int ProcessEvent(const int event_type);
  
  private:
  // move control
  void MoveToDoaTheta(const float doa_theta);
  void DoRotate(const float doa_theta, int direction);
  void MoveToFront();
  void CancelMove();
  // push dest world position msg to queue, which will be pub using action
  void FeedMovePoseMsg(const Twist::SharedPtr &pose);

 private:
  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<AudioTrackingNode> audio_tracking_node_ = nullptr;
  bool start_ = false;

  // 设备是否已经唤醒
  bool is_device_active_ = false;
  float doa_theta_ = 0.0;
  int rate_ = 10;
  float forward_distance_ = 0.2;  // the unit is m
  bool moving_ = false;
  struct MoveInfo {
    double motion_duration = 1000.0;  // milliseconds
    int direction = 1;
    float move_angle = 0.0;
  };

  size_t queue_len_limit_ = 20;
  // audio queue
  std::queue<audio_msg::msg::SmartAudioData::ConstSharedPtr> audio_queue_;
  std::mutex audio_queue_mtx_;
  std::condition_variable audio_queue_condition_;

  std::shared_ptr<std::thread> smart_process_task_ = nullptr;
  bool last_ctrl_is_cancel_ = false;
  double move_duration_ = 0.0;
  RobotMoveCfg move_cfg_;

  // 控制管理，当控制时长超过阈值，发布停止指令
  std::shared_ptr<std::thread> ctrl_manage_task_ = nullptr;
  std::mutex ctrl_manage_mtx_;
   std::condition_variable move_condition_;
  std::chrono::system_clock::time_point last_ctrl_tp;

  // 位置信息
  bool has_tf_info_ = false;
  tf2::Quaternion q_msg;
  double roll, pitch, yaw, i_yaw, m_yaw = 0;
  void MoveToDoaThetaByPose(const float doa_theta);
};

#endif  // AUDIO_TRACKING_ENGINE_H
