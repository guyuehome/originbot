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

#ifndef AUDIO_PARAM_NODE_H
#define AUDIO_PARAM_NODE_H

#include <string>

#include "include/audio_common.h"
#include "rclcpp/rclcpp.hpp"

class ParametersClass : public rclcpp::Node {
 public:
  ParametersClass(
      RobotMoveCfg *cfg,
      const std::string &node_name = "audio_control_parameter_node");
  void Respond();

 private:
  RobotMoveCfg *cfg_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_dump_config_ = true;
};

#endif  // AUDIO_PARAM_NODE_H
