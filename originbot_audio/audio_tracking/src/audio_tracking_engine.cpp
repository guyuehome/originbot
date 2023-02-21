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

#include "include/audio_tracking_engine.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "include/audio_common.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

const float PI = 3.14159;

std::shared_ptr<AudioTrackingEngine> AudioTrackingEngine::Instance() {
  static std::shared_ptr<AudioTrackingEngine> inst =
      std::shared_ptr<AudioTrackingEngine>(new AudioTrackingEngine());
  return inst;
}

AudioTrackingEngine::AudioTrackingEngine() {
  RCLCPP_INFO(rclcpp::get_logger("audio_tracking"),
              "AudioTrackingEngine construct");
  start_ = true;
  param_node_ = std::make_shared<ParametersClass>(&move_cfg_);
  audio_tracking_node_ = std::make_shared<AudioTrackingNode>(
      "audio_tracking",
      std::bind(
          &AudioTrackingEngine::FeedAudioSmart, this, std::placeholders::_1),
      std::bind(
          &AudioTrackingEngine::PoseCallback, this, std::placeholders::_1));

  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(audio_queue_mtx_);
        audio_queue_condition_.wait_for(
            lg, std::chrono::milliseconds(200), [&]() {
              return !audio_queue_.empty();
            });
        if (audio_queue_.empty() || !rclcpp::ok()) {
          continue;
        }
        auto audio_frame = std::move(audio_queue_.front());
        audio_queue_.pop();
        lg.unlock();

        bool moving = false;
        {
          std::lock_guard<std::mutex> lck(ctrl_manage_mtx_);
          moving = moving_;
        }
        if (moving) continue;
        ProcessAudioSmart(audio_frame);
      }

      // 退出前发布停止运动指令，避免程序退出后机器人还一直处于运动状态（如果最后一次收到的指令是启动运动并且运动控制模块没有做超时管理）
      RCLCPP_WARN(rclcpp::get_logger("audio_tracking"),
                  "pkg exit! cancel move");
      CancelMove();
    });
  }

  if (!ctrl_manage_task_) {
    ctrl_manage_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(ctrl_manage_mtx_);
        move_condition_.wait_for(lg, std::chrono::milliseconds(200), [&]() {
          return !audio_queue_.empty();
        });
        if (!moving_ || !rclcpp::ok()) continue;
        lg.unlock();
        if (has_tf_info_) {
          MoveToDoaThetaByPose(doa_theta_);
        } else {
          MoveToDoaTheta(doa_theta_);
        }
        {
          std::lock_guard<std::mutex> lck(ctrl_manage_mtx_);
          moving_ = false;
        }
      }
    });
  }
}

AudioTrackingEngine::~AudioTrackingEngine() {
  RCLCPP_INFO(rclcpp::get_logger("audio_tracking"),
              "AudioTrackingEngine deconstruct");
  start_ = false;
  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }
  if (ctrl_manage_task_ && ctrl_manage_task_->joinable()) {
    ctrl_manage_task_->join();
    ctrl_manage_task_ = nullptr;
  }
}

void AudioTrackingEngine::FeedAudioSmart(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr &msg) {
  if (msg->frame_type.value == 0 || msg->frame_type.value == 1 ||
      msg->frame_type.value == 4) {  // if audio data, return
    std::cout << "call smart cb  msg type:" << msg->frame_type.value
              << std::endl;
    return;
  }
  std::unique_lock<std::mutex> lg(audio_queue_mtx_);
  audio_queue_.push(msg);
  if (audio_queue_.size() > queue_len_limit_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_tracking"),
                 "smart queue len exceed limit: %d",
                 queue_len_limit_);
    audio_queue_.pop();
  }
  audio_queue_condition_.notify_one();
  lg.unlock();
}

void AudioTrackingEngine::ProcessAudioSmart(
    const audio_msg::msg::SmartAudioData::ConstSharedPtr &ai_msg) {
  if (!ai_msg || !rclcpp::ok()) {
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("audio_tracking"),
              "process audio frame type:%d",
              ai_msg->frame_type.value);
  switch (ai_msg->frame_type.value) {
    case Smart_Frame_Type_Voip:
    case Smart_Frame_Type_Wakeup_data:
      break;
    case Smart_Frame_Type_Event:
      ProcessEvent(ai_msg->event_type.value);
      break;
    case Smart_Frame_Type_Doa:
      ProcessDoa(ai_msg->doa_theta);
      break;
    case Smart_Frame_Type_Cmd_word:
      break;
    default:
      break;
  }

  return;
}

int AudioTrackingEngine::ProcessEvent(const int event_type) {
  RCLCPP_INFO(rclcpp::get_logger("audio_tracking"),
              "process audio event type:%d",
              event_type);
  if (event_type == Event_WkpNormal || event_type == Event_WkpOneshot) {
    is_device_active_ = true;
  }
  return 0;
}

int AudioTrackingEngine::ProcessDoa(const float doa_) {
  if (!is_device_active_) return 0;
  RCLCPP_WARN(
      rclcpp::get_logger("audio_tracking"), "process audio doa theta:%f", doa_);
  {
    std::lock_guard<std::mutex> lck(ctrl_manage_mtx_);
    moving_ = true;
    doa_theta_ = doa_;
  }
  move_condition_.notify_one();
  return 0;
}

void AudioTrackingEngine::MoveToDoaTheta(const float doa_theta) {
  bool move_front = false;
  bool rotate = false;
  int direction = 1;
  float angle = 0.0;
  if (80 <= doa_theta &&
      doa_theta <= 100) {  // 声源在前方偏移左右10度左右，不转角
    move_front = true;
  } else if (0 <= doa_theta && doa_theta <= 80) {  // 声源在左边
    move_front = true;
    rotate = true;
    angle = 90 - doa_theta;
  } else if (100 <= doa_theta && doa_theta <= 180) {  // 声源在右边
    move_front = true;
    rotate = true;
    direction = -1;
    angle = doa_theta - 90;
  }

  if (rotate) DoRotate(angle, direction);
  if (move_front) MoveToFront();
}

void AudioTrackingEngine::DoRotate(const float doa_theta, int direction) {
  float angle = doa_theta * PI / 180.0;  // doa_theta * PI / 180.0;
  float ratio = 3.2;
  float angular_speed = ratio * move_cfg_.rotate_step;  // 角速度
  // 旋转时长
  // float angular_duration = angle / move_cfg_.rotate_step;
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;                    // move_cfg_.move_step;
  twist->linear.y = move_cfg_.move_step;  // 0;
  if (direction == -1) {
    twist->linear.y = move_cfg_.move_step;
  }
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = angular_speed;
  if (direction == -1) {
    twist->angular.z = -angular_speed;
  }
  // 机器人开始旋转，延时一定时间使机器人转到所需角度
  int ticks = int(angle * rate_);
  ticks = (ticks == 0) ? 1 : ticks;
  int sleep_time = 1000 / rate_;
  RCLCPP_INFO(
      rclcpp::get_logger("audio_tracking"),
      "process audio doa move theta:%f, angle:%f, direction:%d, ticks:%d",
      doa_theta,
      angle,
      direction,
      ticks);
  for (int i = 0; i < ticks; ++i) {
    FeedMovePoseMsg(twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
  }

  CancelMove();
}

void AudioTrackingEngine::MoveToFront() {
  float linear_speed = move_cfg_.move_step;  // 0.2;  // 线速度
  // 到达目标的时间
  float linear_duration = forward_distance_ / linear_speed;
  auto twist = std::make_shared<Twist>();
  twist->linear.x = linear_speed;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;
  int ticks = int(linear_duration * rate_);
  ticks = (ticks == 0) ? 1 : ticks;
  RCLCPP_INFO(rclcpp::get_logger("audio_tracking"),
              "process audio doa move to front distance:%f, speed:%f, "
              "duration:%f, ticks:%d",
              forward_distance_,
              linear_speed,
              linear_duration,
              ticks);
  int sleep_time = 1000 / rate_;
  for (int i = 0; i < ticks; ++i) {
    FeedMovePoseMsg(twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
  }

  CancelMove();
}

void AudioTrackingEngine::CancelMove() {
  RCLCPP_WARN(rclcpp::get_logger("audio_tracking"), "cancel move");
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;

  FeedMovePoseMsg(twist);
}

void AudioTrackingEngine::FeedMovePoseMsg(const Twist::SharedPtr &pose) {
  if (audio_tracking_node_ && pose) {
    audio_tracking_node_->RobotCtl(*pose);
  }
}

double getYaw(const tf2::Quaternion quat) {
  tf2::Matrix3x3 mat(quat);
  double dummy;
  double yaw;
  mat.getRPY(dummy, dummy, yaw);
  return yaw;
}

void AudioTrackingEngine::PoseCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
  has_tf_info_ = true;
  geometry_msgs::msg::Quaternion quat;
  quat.x = odom->pose.pose.orientation.x;
  quat.y = odom->pose.pose.orientation.y;
  quat.z = odom->pose.pose.orientation.z;
  quat.w = odom->pose.pose.orientation.w;
  double yaw_tmp = 0;
  tf2::Quaternion quat_tf;
  tf2::convert(quat, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw_tmp);
  yaw = (180 / PI) * yaw_tmp;
}

#define UNIT_ANGLE 180 / 3.1149
void AudioTrackingEngine::MoveToDoaThetaByPose(const float doa_theta) {
  bool move_front = false;
  bool rotate = false;
  int direction = 1;
  float angle = 0.0;
  if (80 <= doa_theta &&
      doa_theta <= 100) {  // 声源在前方偏移左右10度左右，不转角
    move_front = true;
  } else if (0 <= doa_theta && doa_theta <= 80) {  // 声源在左边
    move_front = true;
    rotate = true;
    angle = 90 - doa_theta;
  } else if (100 <= doa_theta && doa_theta <= 180) {  // 声源在右边
    move_front = true;
    rotate = true;
    direction = -1;
    angle = doa_theta - 90;
  }

  double angle_tmp = 0.0;
  i_yaw = yaw;
  while (start_) {
    if (angle_tmp * UNIT_ANGLE < angle) {
      auto twist = std::make_shared<Twist>();
      twist->linear.x = 0;
      twist->linear.z = 0;
      twist->angular.x = 0;
      twist->angular.y = 0;
      if (direction == -1) {
        twist->linear.y = -move_cfg_.move_step;     // 线速度
        twist->angular.z = -move_cfg_.rotate_step;  // 角速度
        angle_tmp = i_yaw - yaw;
      } else {
        twist->linear.y = move_cfg_.move_step;     // 线速度
        twist->angular.z = move_cfg_.rotate_step;  // 角速度
        angle_tmp = yaw - i_yaw;
      }
      FeedMovePoseMsg(twist);
    }
    if (angle_tmp * UNIT_ANGLE >= angle) break;
  }
  if (rotate) DoRotate(angle, direction);
  if (move_front) MoveToFront();
}
