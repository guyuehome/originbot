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

#ifndef _ALSA_DEVICE2_H_
#define _ALSA_DEVICE2_H_

#include <alsa/asoundlib.h>

typedef struct alsa_device {
  snd_pcm_t *handle;        /* sound device handle */
  char *name;               /* alsa device name (eg. default) */
  snd_pcm_format_t format;  /* sample format */
  snd_pcm_stream_t direct;  /* stream direction */
  unsigned int rate;        /* stream rate */
  unsigned int channels;    /* count of channels */
  unsigned int buffer_time; /* ring buffer length in us */
  unsigned int period_time; /* period time in us */
  unsigned int nperiods;    /* number of periods */
  int mode;                 /* SND_PCM_NONBLOCK, SND_PCM_ASYNC... */
  snd_pcm_uframes_t
      period_size;  /* period_size, how many frames one period contains */
  snd_pcm_uframes_t
      buffer_size;  /* buffer_size, totally alsa buffer. nperiods period_size */
} alsa_device_t;

alsa_device_t *alsa_device_allocate(void);
int alsa_device_init(alsa_device_t *adev);
int alsa_device_read(alsa_device_t *adev, void *buffer,
                     snd_pcm_uframes_t frames);
int alsa_device_write(alsa_device_t *adev, void *buffer,
                      snd_pcm_uframes_t frames);
void alsa_device_deinit(alsa_device_t *adev);
void alsa_device_free(alsa_device_t *obj);

/* helper function */
void alsa_device_debug_enable(int enable);
#endif /* _ALSA_DEVICE_H_ */
