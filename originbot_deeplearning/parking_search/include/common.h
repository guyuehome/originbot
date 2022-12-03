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

#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include <string>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "functional"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT
using geometry_msgs::msg::Twist;

enum class DirectionType { FRONT = 0, BACK = 1, LEFT = 2, RIGHT = 3};

enum class CtrlType {
  GoFront = 8,
  GoLeft = 4,
  GoBack = 2,
  GoRight = 6,
  Stay = 5,
};

enum class SearchStatus { INITING = 0, SEARCHING, ARRIVED };

struct SearchInfo {
  SearchStatus search_sta = SearchStatus::INITING;
  uint64_t frame_ts_ms = 0;
  int goal = 0;
  int finalcount = 0;
  bool find_parking = false;
};

struct SearchCfg {
  // define area parm
  int area_height = 40;
  int area_width = 120;
  int ingored_bottom = 40;

  // define iou parm
  float mid_parking_iou = 0.7;
  float sides_parking_iou = 0.6;
  float mid_path_iou = 0.9;
  float sides_path_iou = 0.8;

  // define arrived count
  int arrived_count = 400;

  // define robot step
  float move_step = 0.1;
  float rotate_step = 0.1;

  int img_mid_width = 320;
};

#endif  // COMMON_H
