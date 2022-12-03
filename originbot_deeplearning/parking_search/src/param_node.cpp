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

#include <string>
#include "include/param_node.h"

ParametersClass::ParametersClass(SearchCfg *cfg, const std::string &node_name)
    : Node(node_name) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ParametersClass node construct");
  cfg_ = cfg;
  if (cfg_) {
    this->declare_parameter<int>("area_height", cfg_->area_height);
    this->declare_parameter<int>("area_width", cfg_->area_width);
    this->declare_parameter<int>("ingored_bottom", cfg_->ingored_bottom);
    this->declare_parameter<float>("mid_parking_iou", cfg_->mid_parking_iou);
    this->declare_parameter<float>("sides_parking_iou", cfg_->sides_parking_iou);
    this->declare_parameter<float>("mid_path_iou", cfg_->mid_path_iou);
    this->declare_parameter<float>("sides_path_iou", cfg_->sides_path_iou);
    this->declare_parameter<int>("arrived_count", cfg_->arrived_count);
    this->declare_parameter<float>("move_step", cfg_->move_step);
    this->declare_parameter<float>("rotate_step", cfg_->rotate_step);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&ParametersClass::Respond, this));
  }
}

void ParametersClass::Respond() {
  if (!cfg_) return;
  this->get_parameter("area_height", cfg_->area_height);
  this->get_parameter("area_width", cfg_->area_width);
  this->get_parameter("ingored_bottom", cfg_->ingored_bottom);
  this->get_parameter("mid_parking_iou", cfg_->mid_parking_iou);
  this->get_parameter("sides_parking_iou", cfg_->sides_parking_iou);
  this->get_parameter("mid_path_iou", cfg_->mid_path_iou);
  this->get_parameter("sides_path_iou", cfg_->sides_path_iou);
  this->get_parameter("arrived_count", cfg_->arrived_count);
  this->get_parameter("move_step", cfg_->move_step);
  this->get_parameter("rotate_step", cfg_->rotate_step);

  std::stringstream ss;
  ss << "area_height: " << cfg_->area_height << std::endl;
  ss << "area_width: " << cfg_->area_width << std::endl;
  ss << "move_step: " << cfg_->move_step << std::endl;
  ss << "mid_parking_iou: " << cfg_->mid_parking_iou << std::endl;
  ss << "sides_parking_iou: " << cfg_->sides_parking_iou << std::endl;
  ss << "mid_path_iou: " << cfg_->mid_path_iou << std::endl;
  ss << "sides_path_iou: " << cfg_->sides_path_iou << std::endl;
  ss << "arrived_count: " << cfg_->arrived_count << std::endl;
  ss << "move_step: " << cfg_->move_step << std::endl;
  ss << "rotate_step: " << cfg_->rotate_step << std::endl;
  if (first_dump_config_) {
    first_dump_config_ = false;
    RCLCPP_WARN(this->get_logger(), "SearchCfg param are\n%s", ss.str().data());
  } else {
    RCLCPP_DEBUG(this->get_logger(), "SearchCfg param are\n%s", ss.str().data());
  }
}
