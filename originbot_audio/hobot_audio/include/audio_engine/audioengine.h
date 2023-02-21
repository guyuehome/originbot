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

#ifndef INCLUDE_AUDIOPLUGIN_AUDIOENGINE_H_
#define INCLUDE_AUDIOPLUGIN_AUDIOENGINE_H_
#include <string.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "horizon_speech_sdk/include/horizonspeechsdk/hrsc_sdk.h"

namespace hobot {
namespace audio {

using AudioDataFunc = std::function<void(char *, int)>;
using AudioSmartDataFunc = std::function<void(float)>;
using AudioCmdDataFunc = std::function<void(const char *)>;
using AudioEventFunc = std::function<void(int)>;

class AudioEngine {
 public:
  static std::shared_ptr<AudioEngine> &Instance() {
    static std::shared_ptr<AudioEngine> engine;
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {
      engine = std::shared_ptr<AudioEngine>(new AudioEngine());
    });
    return engine;
  }

  ~AudioEngine();
  int Init(AudioDataFunc audio_cb, AudioSmartDataFunc audio_smart_cb,
           AudioCmdDataFunc cmd_cb, AudioEventFunc event_cb,
           const int mic_chn,
           const std::string config_path = "",
           const int voip_mode = 0);
  int DeInit();
  int InputData(char *data, int len, bool end);
  int Start();
  int Stop();
  int Reset();

 public:
  AudioDataFunc GetAudioDataCb() { return audio_cb_; }
  AudioSmartDataFunc GetAudioSmartDataCb() { return audio_smart_cb_; }
  AudioCmdDataFunc GetAudioCmdDataCb() { return audio_cmd_cb_; }
  AudioEventFunc GetAudioEventCb() { return audio_event_cb_; }

 private:
  int InitSDK();
  void DeInitSDK();
  int ParseConfig(std::string config_file);

 private:
  AudioEngine();
  AudioEngine(const AudioEngine &);
  AudioEngine &operator=(const AudioEngine &);

 private:
  bool init_ = false;
  bool start_ = false;
  std::string sdk_config_file_;

  int voip_mode_ = 0;
  HrscAudioConfig input_cfg_;
  HrscAudioConfig output_cfg_;
  HrscEffectConfig effect_cfg_;
  void *sdk_handle_ = 0;
  AudioDataFunc audio_cb_ = nullptr;
  AudioSmartDataFunc audio_smart_cb_ = nullptr;
  AudioCmdDataFunc audio_cmd_cb_ = nullptr;
  AudioEventFunc audio_event_cb_ = nullptr;
  bool save_file_ = false;
  std::ofstream audio_inconvert_file_;
  std::string sdk_file_path_ = "./config/hrsc";
  int mic_chn_num_ = 8;
  int ref_chn_num_ = 0;
  int sdkin_chn_num_ = 6;
  int sdk_ref_chn_num_ = 1;
  std::vector<int> sdk_in_chn_id_list_;
  std::vector<int> sdk_ref_chn_id_list_;
  std::vector<int> ref_chn_id_list_;

  char *adapter_buffer_ = nullptr;
  int audio_size_ = 0;
};

}  // namespace audio
}  // namespace hobot

#endif  // INCLUDE_AUDIOPLUGIN_AUDIOENGINE_H_
