// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef INCLUDE_AUDIOCOMMON_H_
#define INCLUDE_AUDIOCOMMON_H_

#include <memory>
#include <cstring>
#include <iostream>
#include <string>

/**
 * Audio Frame Type
 */
typedef enum {
  Frame_Type_Unknow = 0,       // unknow frame type
  Frame_Type_Audio = 1,        // normal audio data
  Frame_Type_Smart = 2         // smart audio data
} AudioFrameType;

/**
 * Smart Audio Frame TYPE
 */
typedef enum {
  Smart_Frame_Type_Unknow = 0,
  Smart_Frame_Type_Voip = 1,          // the audio after noise reduction
  Smart_Frame_Type_Event = 2,         // audio event
  Smart_Frame_Type_Cmd_word = 3,      // command word data
  Smart_Frame_Type_Wakeup_data = 4,   // wake up audio data
  Smart_Frame_Type_Doa = 5           // doa data
}SmartAudioFrameType;


/**
 * EVNET TYPE
 */
typedef enum {
  Event_Unknow = 0,
  Event_WkpNormal = 1,        // normal wakeup
  Event_WkpOneshot = 2,      // one shot wakeup
  Event_WaitAsrTimeout = 3,  // asr detect timeout
  Event_VadBegin = 4,        // vad begin
  Event_VadMid = 5,          // vad middle
  Event_VadEnd = 6           // vad end
} AudioEventType;

struct SmartAudioFrame {
 public:
  SmartAudioFrame() {}
  SmartAudioFrame(char* data, int size) {
    buffer = reinterpret_cast<char*>(malloc(size));
    memset(buffer, 0, size);
    memcpy(buffer, data, size);
    data_size = size;
  }
  virtual ~SmartAudioFrame() {
    if (buffer) {
      free(buffer);
      buffer = nullptr;
    }
  }

  // smart audio data
  char* buffer = nullptr;
  int data_size = 0;
  SmartAudioFrameType frame_type = Smart_Frame_Type_Cmd_word;
  AudioEventType event_type = Event_WkpNormal;
  std::string cmd_word;
  float doa_theta = 0.0;
};
using SmartAudioFramePtr = std::shared_ptr<SmartAudioFrame>;

struct AudioFrame {
 public:
  AudioFrame() {}
  AudioFrame(char* data, int size) {
    buffer = reinterpret_cast<char*>(malloc(size));
    memset(buffer, 0, size);
    memcpy(buffer, data, size);
    data_size = size;
  }
  virtual ~AudioFrame() {
    if (buffer) {
      free(buffer);
      buffer = nullptr;
    }
  }

  // sequence id, would increment automatically
  uint64_t sequence_id_ = 0;
  // time stamp
  uint64_t time_stamp_ = 0;

  // audio data
  char* buffer = nullptr;
  int data_size = 0;
  SmartAudioFramePtr smart_frame;
};
using AudioFramePtr = std::shared_ptr<AudioFrame>;

/**
 * Robot Move Direction
 */
typedef enum {
  Move_To_Unknow = 0,    //
  Move_To_Forward = 1,   // to move forward
  Move_To_Backward = 2,  // backward moving
  Move_To_Left = 3,      // move to the left
  Move_To_Right = 4,     // move to the right
  Move_Stop = 5
} MoveType;

struct RobotMoveCfg {
  float move_step = 0.3;
  float rotate_step = 0.5;
  // 平移/旋转动作持续时间，单位秒，小于等于0表示不做限制
  // 收到语音控制指令时重置计时并向机器人下发控制指令，达到持续时间后下发停止运动指令，避免机器人一直运动
  int motion_duration_seconds = 0;
};

#endif
