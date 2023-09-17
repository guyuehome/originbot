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

#ifndef INCLUDE_HBAUDIOCAPTURE_H_
#define INCLUDE_HBAUDIOCAPTURE_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utils/alsa_device.h"
#include "audio_msg/msg/smart_audio_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace hobot {
namespace audio {

using rclcpp::NodeOptions;
class HBAudioCapture : public rclcpp::Node {
 public:
  // node_name为创建的节点名，options为选项，用法和ROS Node相同
  HBAudioCapture(const std::string& node_name,
                 const NodeOptions& options = NodeOptions());

  virtual ~HBAudioCapture();

  int Run();

 public:
  int Init();
  int DeInit();
  int Start();
  int Stop();

 private:
  int ParseConfig(std::string config_file);
  int MicphoneGetThread();
  void AudioDataFunc(char* buffer, int size);
  void AudioSmartDataFunc(float theta);
  void AudioCmdDataFunc(const char* cmd_word);
  void AudioEventFunc(int event);
  void AudioASRDataFunc(const char* asr);

 private:
  int micphone_enable_ = 1;
  std::shared_ptr<std::thread> micphone_thread_;
  alsa_device_t* micphone_device_ = nullptr;
  bool exit_ = true;
  bool is_init_ = false;
  int audio_num_ = 0;
  std::string micphone_name_ = "hw:0,0";
  int micphone_rate_ = 16000;
  int micphone_chn_ = 8;
  int micphone_buffer_time_ = 0;
  int micphone_nperiods_ = 4;
  int micphone_period_size_ = 512;
  int voip_mode_ = 0;
  int mic_type_ = 0;
  int asr_output_mode_ = 0;
  int asr_output_channel_ = 3;

  std::string config_path_ = "./config";
  std::string audio_pub_topic_name_ = "/audio_smart";
  std::string asr_pub_topic_name_ = "/audio_asr";
  std::ofstream audio_infile_;
  std::ofstream audio_sdk_;
  bool save_audio_ = false;

  rclcpp::Publisher<audio_msg::msg::SmartAudioData>::SharedPtr msg_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asr_msg_publisher_;
};

}  // namespace audio
}  // namespace hobot

#endif
