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

#include "include/audio_control_engine.h"
#include "include/audio_common.h"
#include <fstream>
#include <memory>
#include <sstream>
#include <vector>
#include <utility>

std::shared_ptr<AudioControlEngine> AudioControlEngine::Instance() {
  static std::shared_ptr<AudioControlEngine> inst =
      std::shared_ptr<AudioControlEngine>(new AudioControlEngine());
  return inst;
}

AudioControlEngine::AudioControlEngine() {
  RCLCPP_INFO(rclcpp::get_logger("audio_control"),
              "AudioControlEngine construct");
  start_ = true;
  param_node_ = std::make_shared<ParametersClass>(&move_cfg_);
  audio_control_node_ = std::make_shared<AudioControlNode>(
      "audio_control",
      std::bind(&AudioControlEngine::FeedSmart, this, std::placeholders::_1));

  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          return !smart_queue_.empty();
        });
        if (smart_queue_.empty() || !rclcpp::ok()) {
          continue;
        }
        auto smart_frame = std::move(smart_queue_.front());
        smart_queue_.pop();
        lg.unlock();
        ProcessSmart(smart_frame);
      }

      // 退出前发布停止运动指令，避免程序退出后机器人还一直处于运动状态（如果最后一次收到的指令是启动运动并且运动控制模块没有做超时管理）
      RCLCPP_WARN(rclcpp::get_logger("audio_control"),
                  "pkg exit! cancel move");
      CancelMove();
    });
  }
  
  RCLCPP_WARN(rclcpp::get_logger("audio_control"),
              "control timeout: %d second",
              move_cfg_.motion_duration_seconds);
  if (move_cfg_.motion_duration_seconds > 0 && !ctrl_manage_task_) {
    ctrl_manage_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(ctrl_manage_mtx_);
        if (last_ctrl_is_cancel_) {
          lg.unlock();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        } else {
          auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::system_clock::now() - last_ctrl_tp)
                              .count();
          if (interval >= 1000 * move_cfg_.motion_duration_seconds) {
            RCLCPP_WARN(rclcpp::get_logger("audio_control"),
                        "Cancel move, control timeout: %d seconds",
                        move_cfg_.motion_duration_seconds);
            CancelMove();
          } else {
            lg.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
        }
      }

      CancelMove();
    });
  }
}

AudioControlEngine::~AudioControlEngine() {
  RCLCPP_INFO(rclcpp::get_logger("audio_control"),
              "AudioControlEngine deconstruct");
  start_ = false;
  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }
  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }
}

void AudioControlEngine::FeedSmart(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg) {
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);
  smart_queue_.push(msg);
  if (smart_queue_.size() > queue_len_limit_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_control"),
                 "smart queue len exceed limit: %d",
                 queue_len_limit_);
    smart_queue_.pop();
  }
  smart_queue_condition_.notify_one();
  lg.unlock();
}

void AudioControlEngine::ProcessSmart(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr &ai_msg) {
  if (!ai_msg || !rclcpp::ok()) {
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_control"),
              "process audio frame type:%d",
              ai_msg->frame_type.value);
  switch (ai_msg->frame_type.value) {
    case Smart_Frame_Type_Voip:
    case Smart_Frame_Type_Wakeup_data:
      break;
    case Smart_Frame_Type_Event:
      /* code */
      break;
    case Smart_Frame_Type_Doa:
      /* code */
      break;
    case Smart_Frame_Type_Cmd_word:
      ProcessCmdWord(ai_msg->cmd_word);
      break;
    default:
      break;
  }

  return;
}

int AudioControlEngine::GetMoveCmd(const std::string cmd_word) {
  if (cmd_word.find("向前") != std::string::npos) {
    return Move_To_Forward;
  } else if (cmd_word.find("向后") != std::string::npos) {
    return Move_To_Backward;
  } else if (cmd_word.find("向左") != std::string::npos) {
    return Move_To_Left;
  } else if (cmd_word.find("向右") != std::string::npos) {
    return Move_To_Right;
  } else if (cmd_word.find("停") != std::string::npos) {
    return Move_Stop;
  } else {
    return Move_To_Unknow;
  }
}

int AudioControlEngine::ProcessCmdWord(const std::string cmd_word) {
  if (cmd_word.find("精灵精灵") != std::string::npos) {
    return 0;
  }
  int direction = GetMoveCmd(cmd_word);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
              "call cmd fun, cmd:%s, move direction:%d",
              cmd_word,
              direction);
  if (direction == Move_Stop) {
    CancelMove();
    return 0;
  }

  DoMove(direction, move_cfg_.move_step, move_cfg_.rotate_step);
  return 0;
}

void AudioControlEngine::DoMove(int direction,
                                float move_step,
                                float rotate_step) {
  last_ctrl_is_cancel_ = false;
  std::unique_lock<std::mutex> lg(ctrl_manage_mtx_);
  last_ctrl_tp = std::chrono::system_clock::now();
  lg.unlock();
  RCLCPP_WARN(rclcpp::get_logger("audio_control"),
              "do move, direction: %d, step: %f, rotate step: %f",
              direction,
              move_step,
              rotate_step);
  auto twist = std::make_shared<Twist>();
  if (Move_To_Forward == direction) {
    twist->linear.x += move_step;
  } else if (Move_To_Backward == direction) {
    twist->linear.x -= move_step;
  } else if (Move_To_Left == direction) {
    twist->linear.y += move_step;
    twist->angular.z = rotate_step;
  } else if (Move_To_Right == direction) {
    twist->linear.y -= move_step;
    twist->angular.z -= rotate_step;
  }

  FeedMovePoseMsg(twist);
}

void AudioControlEngine::DoRotate(int direction, float step) {
  last_ctrl_is_cancel_ = false;
  std::unique_lock<std::mutex> lg(ctrl_manage_mtx_);
  last_ctrl_tp = std::chrono::system_clock::now();
  lg.unlock();
  RCLCPP_WARN(rclcpp::get_logger("audio_control"),
              "do rotate, direction: %d, step: %f",
              direction,
              step);

  int direct = 1;
  if (Move_To_Right == direction) {
    direct = -1;
  }

  auto twist = std::make_shared<Twist>();
  twist->angular.z = direct * step;
  FeedMovePoseMsg(twist);
}

void AudioControlEngine::CancelMove() {
  if (last_ctrl_is_cancel_) return;
  last_ctrl_is_cancel_ = true;
  RCLCPP_WARN(rclcpp::get_logger("audio_control"), "cancel move");
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;
  FeedMovePoseMsg(twist);
}

void AudioControlEngine::FeedMovePoseMsg(const Twist::SharedPtr &pose) {
  if (audio_control_node_ && pose) {
    audio_control_node_->RobotCtl(*pose);
  }
}
