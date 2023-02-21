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

#ifndef AUDIO_CONTROL_ENGINE_H
#define AUDIO_CONTROL_ENGINE_H

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "include/audio_control_node.h"
#include "audio_msg/msg/smart_audio_data.hpp"
#include "include/param_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using geometry_msgs::msg::Twist;
using rclcpp::NodeOptions;

class AudioControlEngine {
 public:
  static std::shared_ptr<AudioControlEngine> Instance();
  ~AudioControlEngine();

 public:
  void FeedSmart(const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg);
  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes() {
    std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
    node_ptrs.push_back(param_node_);
    node_ptrs.push_back(audio_control_node_);
    return node_ptrs;
  }

 private:
  AudioControlEngine();
  void ProcessSmart(const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg);
  int ProcessCmdWord(const std::string cmd_word);
  int GetMoveCmd(const std::string cmd_word);
  // rotate robot
  // direction: 0 left, 1 right; unit of step is radian
  void DoRotate(int direction, float step = 0.5);
  // direction: 0 front, 1 back, 2 left, 3 right; unit of step is m/s
  void DoMove(int direction, float step = 0.3, float rotate_step = 0.5);
  void CancelMove();
  // push dest world position msg to queue, which will be pub using action
  void FeedMovePoseMsg(const Twist::SharedPtr &pose);

  private:
  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<AudioControlNode> audio_control_node_ = nullptr;
  bool start_ = false;

  size_t queue_len_limit_ = 20;
  std::queue<audio_msg::msg::SmartAudioData::ConstSharedPtr> smart_queue_;
  std::mutex smart_queue_mtx_;
  std::condition_variable smart_queue_condition_;

  std::shared_ptr<std::thread> smart_process_task_ = nullptr;
  bool last_ctrl_is_cancel_ = false;
  RobotMoveCfg move_cfg_;
  
  // 控制管理，当控制时长超过阈值，发布停止指令
  std::shared_ptr<std::thread> ctrl_manage_task_ = nullptr;
  std::mutex ctrl_manage_mtx_;
  std::chrono::system_clock::time_point last_ctrl_tp;
};

#endif  // AUDIO_CONTROL_ENGINE_H
