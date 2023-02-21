/*
 * Copyright (c) 2019 horizon.ai.
 */

#ifndef __HRSC_SDK_H__
#define __HRSC_SDK_H__

#ifdef __cplusplus
extern "C" {
#endif

#define HRSC_SDK_VERSION (1.0)

/*************************SDK CODE****************************/
#define HRSC_CODE_SUCCESS (0)                // success
#define HRSC_CODE_FAILED (-1)                // failed
#define HRSC_CODE_PARAMS_ERROR (-2)          // param err
#define HRSC_CODE_RESOURCES_NOT_EXISTS (-3)  // resouce not exist
#define HRSC_CODE_CONFIG_ERROR (-4)          // config err
#define HRSC_CODE_UNKNOW  (-5)               // unknown err
#define HRSC_CODE_INNER_ERROR (-6)           // inner err
/*************************************************************/

typedef unsigned long long HrscTimeStamp;
typedef void *SpeechSDKHandle;

typedef enum {
  kHrscAudioFormatPcm16Bit = 0,  // 16bit resample
  kHrscAudioFormatPcm8Bit,       // 8 bit resample
  kHrscAudioFormatPcm24Bit,      // 24bit resample
  kHrscAudioFormatPcm32Bit,      // 32bit resample
} HrscAudioFormat;

/**
 * SDK STATUS
 */
typedef enum {
  kHrscStatusUnwkp = 0,   // the status of unwkp
  kHrscStatusWkp,         // the status of wkp
  kHrscStatusAsr,         // the status of asr
  kHrscStatusVoip,        // the status of voip(comms)
  kHrscStatusMult,        // the status of multi dialog
  kHrscStatusConts        // the status of continous dialog
} HrscStatus;

/**
 * SDK EVNET TYPE
 */
typedef enum {
  kHrscEventWkpNormal = 0,     // normal wakeup
  kHrscEventWkpOneshot,        // one shot wakeup
  kHrscEventWaitAsrTimeout,    // asr detect timeout
  kHrscEventVadBegin,          // vad begin
  kHrscEventVadMid,            // vad middle
  kHrscEventVadEnd,            // vad end
  kHrscEventSDKStart,          // sdk start
  kHrscEventSDKEnd             // sdk end
} HrscEventType;

/**
 * SDK param setting key
 */
typedef enum {
  kHrscParasTypeWkpEventSwitch,     // wkp event 0 disable, 1 enable(default)
  kHrscParasTypeWkpDataSwitch,      // wkp data 0 disable, 1 enable(default)
  kHrscParasTypeVadSwitch,          // asr use vad, 0 disable, 1 enable(default)
  kHrscParasTypeProcessDataSwitch,  // proc data, 0 disable(default), 1 enable
  kHrscParasTypeVoipDataSwitch,     // voip data, 0, disable(default), 1 enable
  kHrscParasTypeOneshotSwitch,      // one shot, 0 disable(default), 1 enable
  kHrscParasTypeAsrRecState,        // asr recognize state, 0 start, 1 end
  kHrscParasTypeReduceComputation   // reduce computation, 0 close, 1, open
} HrscParamType;

/**
 * SDK DATA TYPE
 */
typedef enum {
  kHrscCallbackDataAll = 0,  // all data
  kHrscCallbackDataHead,     // the head of data
  kHrscCallbackDataMid,      // the middle of data
  kHrscCallbackDataTail      // the tail of data
} HrscCallbackDataType;

/**
 * SDK PARAM DATA
 */
typedef struct {
  HrscParamType param_type;  // the type of param
  void *value;               // the value of param
} HrscParamData;

/**
 * SDK AUDIO CONFIG
 */
typedef struct {
  unsigned int sample_rate;     // sample rate
  unsigned int audio_channels;  // channel
  HrscAudioFormat audio_format; // sample bits
} HrscAudioConfig;

/**
 * SDK AUDIO BUFFER
 */
typedef struct {
  void *audio_data;     // audio data
  unsigned int size;    // the len of audio data
  HrscTimeStamp start;  // the begin of audio data time
  HrscTimeStamp end;    // the end of audio data time
} HrscAudioBuffer;

/**
 * SDK callback data
 */
typedef struct {
  HrscCallbackDataType hrsc_data_type;  // see HrscCallbackDataType
  HrscAudioBuffer audio_buffer;         // see HrscAudioBuffer
  float angle;                          // the angle of wkp
  float score;                          // the score of wkp
  char *result;                         // the result of wkp or cmd
  unsigned int result_size;             // the size of result
} HrscCallbackData;

typedef struct {
  /**
   * @brief : input auido format
   */
  HrscAudioConfig input_cfg;
  /**
   * @brief : target output format
   */
  HrscAudioConfig output_cfg;
  /**
   * @brief : channel index of reference
   */
  unsigned char ref_ch_index;
  /**
   * @brief : ms, pcm data before wakeup word
   */
  unsigned int wakeup_prefix;
  /**
   * @brief : ms, pcm data after wakeup word
   */
  unsigned int wakeup_suffix;
  /**
   * @brief : asr timeout(seconds)
   */
  unsigned int asr_timeout;
  /**
   * @brief : VAD timeout(seconds)
   */
  unsigned int vad_timeout;
  /**
    * @brief : 0.0 ~ 1.0, do not send wakeup event when score < target_score
    */
  float target_score;
  /**
   * @brief : config file path if needed, e.g., "/vendor/etc/hrsc_cfg.cfg"
   */
  const char *cfg_file_path;
  /**
   * @brief : private data pointer
   */
  void *priv;
  /**
   * @brief : custom wakeup word (GBK Encoding)
   */
  const char *custom_wakeup_word;
  /**
   * @breif: output name
   */
  const char *output_name;
  /**
   * @brief: the flag of support command word recognition
   */
  unsigned char support_command_word;
  /**
   * @brief: the unique str of device if needed activate
   */
  const char *device_unique_str;
  /**
   * @brief notify user when a new event happen
   * @param cookie, hrsc_effect_config_t->priv
   * @param event, see hrsc_event_t
   */
  void (*HrscEventCallback)(const void *cookie, HrscEventType event);
  /**
   * @brief send the wake up data to user
   * @param cookie, HrscEffectConfig->priv
   * @param data, see HrscCallbackData
   * @param keyword_index: wakeup keyword index, ignore if
   *        just support one keyword
   */
  void (*HrscWakeupDataCallback)(const void *cookie,
                                 const HrscCallbackData *data,
                                 const int keyword_index);
  /**
   * @brief asr data callback handler
   * @param cookie, HrscEffectConfig->priv
   * @param data, see HrscCallbackData
   */
  void (*HrscAsrDataCallback)(const void *cookie, const HrscCallbackData *data);
  /**
   * @brief voip data callback handler
   * @param cookie, HrscEffectConfig->priv
   * @param data, see HrscCallbackData
   */
  void (*HrscVoipDataCallback)(const void *cookie,
                               const HrscCallbackData *data);
  /**
   * @brief send processed data to user
   * @param cookie, HrscEffectConfig->priv
   * @param data, see HrscCallbackData
   */
  void (*HrscProcessedDataCallback)(const void *cookie,
                                    const HrscCallbackData *data);
  /**
   * @brief the callback of auth code
   * @param cookie, HrscEffectConfig->priv
   * @param code
   */
  void (*HrscAuthCallback)(const void *cookie, const int code);
  /**
   * @brief the callback of auth code
   * @param cookie, HrscEffectConfig->priv
   * @param code
   */
  void (*HrscCmdCallback)(const void *cookie, const char *cmd);
  /**
   * @brief doa of wkp or cmd
   * @param cookie, HrscEffectConfig->priv
   * @param code
   */
  void (*HrscDoaCallbadk)(const void *cookie, int doa);
} HrscEffectConfig;

/**
 * @brief get the version of SDK
 * @return version
 */
const char *HrscGetVersion();

/**
 * @brief init speech sdk
 * @param effect_config see HrscEffectConfig
 * @return speech handle
 */
void *HrscInit(HrscEffectConfig *effect_config);

/**
 * @brief start speech sdk
 * @param handle: speech handle, get from HrscInit
 * @return see SDK CODE
 */
int HrscStart(void *handle);

/**
 * @brief input audio data to process
 * @param handle: speech handle, get from HrscInit
 * @param buffer, see HrscAudioBuffer
 * @return see SDK CODE
 */
int HrscProcess(void *handle, HrscAudioBuffer *buffer);

/**
 * @brief get the sdk running status
 * @param handle: speech handle, get from HrscInit
 * @return see SDK CODE
 */
HrscStatus HrscGetStatus(void *handle);

/**
 * @brief set sdk work status
 * @param handle: speech handle, get from HrscInit
 * @param status see HrscStatus
 * @return see SDK CODE
 */
int HrscSetStatus(void *handle, HrscStatus status);

/**
 * @brief set sdk param
 * @param handle: speech handle, get from HrscInit
 * @param param_data: see HrscParamData
 * @return see SDK CODE
 */
int HrscSetParam(void *handle, HrscParamData *param_data);

/**
 * @brief get sdk param
 * @param handle: speech handle, get from HrscInit
 * @param param_data: see HrscParamData
 * @param value: param value
 * @return see SDK CODE
 */
int HrscGetParam(void *handle, HrscParamData *param_data, void *value);

/**
 * @brief stop speech sdk
 * @param handle: speech handle, get from HrscInit
 * @return see SDK CODE
 */
int HrscStop(void *handle);

/**
 * @brief release speech sdk
 * @param handle: speech handle, get from HrscInit
 * @return see SDK CODE
 */
int HrscRelease(void **handle);

#ifdef __cplusplus
};
#endif

#endif //__HRSC_SDK_H__
