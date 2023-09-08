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

#include "audio_engine/audioengine.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace audio {

void VoipDataCallback(const void *, const HrscCallbackData *data) {
  if (!data) return;
  RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
               "recv hrsc sdk callback audio, angle:%f, score:%f, data size:%d",
               data->angle, data->score, data->audio_buffer.size);
  if (AudioEngine::Instance()->GetAudioDataCb()) {
    AudioEngine::Instance()->GetAudioDataCb()(
        reinterpret_cast<char *>(data->audio_buffer.audio_data),  // nolint
        data->audio_buffer.size);
  }
  // if (AudioEngine::Instance()->GetAudioSmartDataCb()) {
  //   AudioEngine::Instance()->GetAudioSmartDataCb()(data->angle);
  // }
}

void WakeupDataCallback(const void *, const HrscCallbackData *data,
                        const int keyword_index) {
  if (!data) return;
  std::cout << "recv hrsc sdk wakeup data , size is " << data->audio_buffer.size
            << ", key index:" << keyword_index << std::endl;
}

void EventCallback(const void *, HrscEventType event) {
  static int wkp_count = 0;
  if (event == kHrscEventWkpNormal || event == kHrscEventWkpOneshot) {
    std::cout << "recv hrsc sdk event wakeup success, wkp count is "
              << ++wkp_count << std::endl;
    if (AudioEngine::Instance()->GetAudioEventCb()) {
      AudioEngine::Instance()->GetAudioEventCb()(event);
    }
  }
}

void CmdDataCallback(const void *, const char *cmd) {
  if (!cmd) return;
  std::cout << "recv hrsc sdk command data: " << cmd << std::endl;
  if (AudioEngine::Instance()->GetAudioCmdDataCb()) {
    AudioEngine::Instance()->GetAudioCmdDataCb()(cmd);
  }
}

void DoaCallback(const void *, int doa) {
  std::cout << "recv hrsc sdk doa data: " << doa << std::endl;
  if (AudioEngine::Instance()->GetAudioSmartDataCb()) {
    AudioEngine::Instance()->GetAudioSmartDataCb()(doa);
  }
}

void AsrCallback(const void *, const char *asr) {
  std::cout << "asr is: " << asr << std::endl;
  if (AudioEngine::Instance()->GetASREventCb()) {
    AudioEngine::Instance()->GetASREventCb()(asr);
  }
}

AudioEngine::AudioEngine() {}

AudioEngine::~AudioEngine() {
  if (adapter_buffer_) delete[] adapter_buffer_;
}

int AudioEngine::ParseConfig(std::string config_file) {
  if (config_file.empty()) return -1;
  RCLCPP_INFO(rclcpp::get_logger("audio_capture"), "hrsc config file:%s",
              config_file.c_str());
  std::ifstream ifs(config_file);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "open config file:%s fail", config_file.c_str());
    return -1;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.find("\"micphone_chn\"") != std::string::npos) {
      size_t nEndPos = line.find(":");
      if (nEndPos == std::string::npos) continue;
      line = line.substr(nEndPos + 1);
      mic_chn_num_ = atoi(line.c_str());
      RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "mic_chn_num_: %d",
                  mic_chn_num_);
    }
  }
  ifs.close();
  return 0;
}

int AudioEngine::Init(AudioDataFunc audio_cb, AudioSmartDataFunc audio_smart_cb,
                      AudioCmdDataFunc cmd_cb, AudioEventFunc event_cb,
                      AudioASRDataFunc asr_cb,
                      const int mic_chn, const std::string config_path,
                      const int voip_mode, const int mic_type,
                      const int asr_output_mode, const int asr_output_channel) {
  if (init_) {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
                "has already initialized.");
    return 0;
  }
  mic_chn_num_ = mic_chn;
  voip_mode_ = voip_mode;
  mic_type_ = mic_type;
  asr_mode_ = asr_output_mode;
  asr_channel_ = asr_output_channel;
  std::string config_file = config_path + "/audio_config.json";
  sdk_file_path_ = config_path + "/hrsc";
  // ParseConfig(config_file);
  int ret = InitSDK();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "init audio sdk fail");
    return ret;
  }

  audio_cb_ = audio_cb;
  audio_smart_cb_ = audio_smart_cb;
  audio_cmd_cb_ = cmd_cb;
  audio_event_cb_ = event_cb;
  audio_asr_cb_ = asr_cb;
  init_ = true;
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "init hrsc sdk success!");
  return 0;
}

int AudioEngine::DeInit() { return 0; }

int AudioEngine::Start() {
  if (!init_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "engine not init.");
    return -1;
  }

  if (start_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "engine has already started.");
    return 0;
  }

  start_ = true;
  if (save_file_) {
    audio_inconvert_file_.open(
        "./audio_sdk.pcm", std::ios::app | std::ios::out | std::ios::binary);
  }

  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine start success");
  return 0;
}

int AudioEngine::Reset() { return 0; }

int AudioEngine::Stop() {
  if (!start_) return 0;
  DeInitSDK();
  if (audio_inconvert_file_.is_open()) audio_inconvert_file_.close();
  return 0;
}

int AudioEngine::InitSDK() {
  input_cfg_.audio_channels = sdkin_chn_num_;
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine sdk in chn:%d",
              sdkin_chn_num_);
  input_cfg_.sample_rate = 16000;
  input_cfg_.audio_format = kHrscAudioFormatPcm16Bit;

  output_cfg_.audio_channels = 1;
  output_cfg_.sample_rate = 16000;
  output_cfg_.audio_format = kHrscAudioFormatPcm16Bit;
  memset(&effect_cfg_, 0, sizeof(HrscEffectConfig));
  effect_cfg_.input_cfg = input_cfg_;
  effect_cfg_.output_cfg = output_cfg_;
  effect_cfg_.priv = &effect_cfg_;
  effect_cfg_.asr_timeout = 5000;
  effect_cfg_.cfg_file_path = sdk_file_path_.c_str();
  if (!voip_mode_) {
    std::string cus_path = sdk_file_path_ + "/cmd_word.json";
    RCLCPP_INFO(rclcpp::get_logger("audio_capture"),
                "hrsc sdk file path:%s, cmd file:%s", sdk_file_path_.c_str(),
                cus_path.c_str());
    if (cus_path.empty()) {
      effect_cfg_.custom_wakeup_word = nullptr;
    } else {
      std::fstream stream(cus_path, std::ios::in);
      if (!stream.is_open()) {
        effect_cfg_.custom_wakeup_word = nullptr;
      } else {
        // get length of file:
        stream.seekg(0, stream.end);
        int length = stream.tellg();
        stream.seekg(0, stream.beg);
        char *buffer = new char[length + 1];
        stream.read(buffer, length);
        buffer[length] = '\0';
        effect_cfg_.custom_wakeup_word = buffer;
        stream.close();
        // delete[] buffer;
        std::cout << "hrsc sdk wakeup word is:" << std::endl
                  << effect_cfg_.custom_wakeup_word << std::endl;
      }
    }
  } else {
    effect_cfg_.custom_wakeup_word = nullptr;
  }

  effect_cfg_.vad_timeout = 5000;
  effect_cfg_.ref_ch_index = 6;  // 参考信号通道index
  effect_cfg_.target_score = 0;
  effect_cfg_.support_command_word = 0;
  effect_cfg_.wakeup_prefix = 200;
  effect_cfg_.wakeup_suffix = 200;
  effect_cfg_.is_use_linear_mic_flag = mic_type_;

  effect_cfg_.asr_output_mode = asr_mode_;
  effect_cfg_.asr_output_channel = asr_channel_;

  effect_cfg_.HrscVoipDataCallback = VoipDataCallback;
  effect_cfg_.HrscWakeupDataCallback = WakeupDataCallback;
  effect_cfg_.HrscEventCallback = EventCallback;
  effect_cfg_.HrscCmdCallback = CmdDataCallback;
  effect_cfg_.HrscDoaCallbadk = DoaCallback;
  effect_cfg_.HrscAsrCallback = AsrCallback;

  sdk_handle_ = HrscInit(&effect_cfg_);
  if (sdk_handle_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc init error!!!");
    return -1;
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc init success!");
  if (HRSC_CODE_SUCCESS != HrscStart(sdk_handle_)) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc start error!!!");
    return -1;
  }

  int value = 0;
  if (voip_mode_) {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
                "audioengine sdk init on voip mode");
    value = 1;
  }
  HrscParamData data;
  data.value = &value;
  data.param_type = kHrscParasTypeVoipDataSwitch;
  HrscSetParam(sdk_handle_, &data);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc start success!");
  return 0;
}

int AudioEngine::InputData(char *data, int len, bool end) {
  if (!init_ || !start_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "engine not init or start.");
    return -1;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
              "audioengine send audio data to sdk");
  if (end) {
    RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
              "audioengine recv the last audio frame");
  }
  int size = len / mic_chn_num_ * sdkin_chn_num_;
  if (audio_size_ != size) {
    if (adapter_buffer_) delete[] adapter_buffer_;
    audio_size_ = size;
    adapter_buffer_ = new char[size];
    memset(adapter_buffer_, 0, size);
  }

  if (mic_chn_num_ == sdkin_chn_num_) {
    memcpy(adapter_buffer_, data, size);
  } else {
    // 8通道数据->6通道数据，因X3Pi麦克风8mic，参考信号为7,8通道，剔除5, 6通道，数据类型为s16
    char *dst_ptr = adapter_buffer_;
    char *src_ptr = data;
    int frame_count = len / 16;
    int index = 0;
    while (index++ < frame_count) {
      memcpy(dst_ptr, src_ptr, 4 * 2);
      dst_ptr += 4 * 2;
      src_ptr += 6 * 2;
      memcpy(dst_ptr, src_ptr, 2 * 2);
      dst_ptr += 2 * 2;
      src_ptr += 2 * 2;
    }
//     int chn_tmp = sdkin_chn_num_ << 1;
//     int data_wide = mic_chn_num_ << 1;
//     int count = len / mic_chn_num_ >> 1;
//     for (int i = 0; i < count; ++i) {
//       int k = 0;
//       for (int j = 0; j < chn_tmp; ++j) {
//         adapter_buffer_[chn_tmp * i + j] = data[data_wide * i + k];
//         ++k;
//         if (k >= data_wide) k = 0;
//       }
// #if 0
//       k = 0;
//       for (auto item : sdk_ref_chn_id_list_) {
//         if (k < ref_chn_id_list_.size()) {
//           adapter_buffer_[chn_tmp * i + (item * 2)] =
//               data[data_wide * i + ref_chn_id_list_[k] * 2];
//           adapter_buffer_[chn_tmp * i + (item * 2 + 1)] =
//               data[data_wide * i + (ref_chn_id_list_[k] * 2 + 1)];
//         } else {
//           adapter_buffer_[chn_tmp * i + (item * 2)] = 0;
//           adapter_buffer_[chn_tmp * i + (item * 2 + 1)] = 0;
//         }
//         k++;
//       }
// #endif
//     }
  }


  HrscAudioBuffer hrsc_buffer;
  hrsc_buffer.audio_data = adapter_buffer_;
  hrsc_buffer.size = audio_size_;
  HrscProcess(sdk_handle_, &hrsc_buffer);
  if (save_file_ && audio_inconvert_file_.is_open()) {
    audio_inconvert_file_.write(adapter_buffer_, audio_size_);
  }
  return 0;
}

void AudioEngine::DeInitSDK() {
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "before stop audio sdk!");
  HrscStop(sdk_handle_);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "stop audio sdk success!");
  HrscRelease(&sdk_handle_);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
              "destory audio sdk success!");
}

}  // namespace audio
}  // namespace hobot
