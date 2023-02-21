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

#include <cstdio>

#include "include/audio_tracking_engine.h"

int main(int argc, char** argv) {
  std::stringstream ss;
  ss << "\n\tThis is audio tracking package.\n\n"
     << "============================================\n"
     << "\taudio tracking usage\n"
     << "\nWake up device is \"地平线你好\".\n"
     << "Audio control commnad word definitions are:\n"
     << "\t\"向前走\"\n"
     << "\t\"向后退\"\n"
     << "\t\"向右转\"\n"
     << "\t\"向左转\" \n"
     << "When you say the wake word, the car turns toward you \n"
     << "Let's start the experience\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto nodes = AudioTrackingEngine::Instance()->GetNodes();
  for (auto& node : nodes) {
    exec.add_node(node);
  }
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
