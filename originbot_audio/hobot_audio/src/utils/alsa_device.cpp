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

#include "utils/alsa_device.h"

#include <stdio.h>
#include <sys/time.h>

#include "utils/utils.h"

/*
 * Definitions
 */
#ifndef timersub
#define timersub(a, b, result)                       \
  do {                                               \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
    if ((result)->tv_usec < 0) {                     \
      --(result)->tv_sec;                            \
      (result)->tv_usec += 1000000;                  \
    }                                                \
  } while (0)
#endif

#ifndef timermsub
#define timermsub(a, b, result)                      \
  do {                                               \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
    (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
    if ((result)->tv_nsec < 0) {                     \
      --(result)->tv_sec;                            \
      (result)->tv_nsec += 1000000000L;              \
    }                                                \
  } while (0)
#endif

/*
 * static variables
 */
static snd_output_t *log;
static int dump_hw_params = 0;

/*
 * functions
 */
static void xrun(snd_pcm_t *handle, int stream) {
  snd_pcm_status_t *status;
  int res;

  snd_pcm_status_alloca(&status);
  if ((res = snd_pcm_status(handle, status)) < 0) {
    fprintf(stderr, "status error: %s\n", snd_strerror(res));
    return;
  }

  if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN) {
    struct timeval now, diff, tstamp;
    gettimeofday(&now, 0);
    snd_pcm_status_get_trigger_tstamp(status, &tstamp);
    timersub(&now, &tstamp, &diff);
    fprintf(stderr, "%s!!! (at least %.3f ms long)\n",
            stream == SND_PCM_STREAM_PLAYBACK ? "underrun" : "overrun",
            diff.tv_sec * 1000 + diff.tv_usec / 1000.0);
    fprintf(stderr, "Status:\n");

    if (dump_hw_params) snd_pcm_status_dump(status, log);
    if ((res = snd_pcm_prepare(handle)) < 0) {
      fprintf(stderr, "xrun: prepare error: %s", snd_strerror(res));
      return;
    }
    return; /* ok, data should be accepted again */
  }
  if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING) {
    fprintf(stderr, "Status(DRAINING):\n");
    if (dump_hw_params) snd_pcm_status_dump(status, log);
    if (stream == SND_PCM_STREAM_CAPTURE) {
      fprintf(stderr, "capture stream format change? attempting recover...\n");
      if ((res = snd_pcm_prepare(handle)) < 0) {
        fprintf(stderr, "xrun(DRAINING): prepare error: %s", snd_strerror(res));
        return;
      }
      return;
    }
  }

  fprintf(stderr, "Status(R/W):\n");
  if (dump_hw_params) snd_pcm_status_dump(status, log);
  fprintf(stderr, "read/write error, state = %s\n",
          snd_pcm_state_name(snd_pcm_status_get_state(status)));
}

void alsa_device_debug_enable(int enable) {
  int err;

  if ((dump_hw_params && enable) || (!dump_hw_params && !enable)) return;

  if (enable) {
    err = snd_output_stdio_attach(&log, stderr, 0);
    assert(err >= 0);
    printf("err(%d)\n", err);
  } else {
    snd_output_close(log);
  }

  dump_hw_params = enable ? 1 : 0;
}

alsa_device_t *alsa_device_allocate(void) {
  alsa_device_t *adev =
    reinterpret_cast<alsa_device_t *>(calloc(1, sizeof(alsa_device_t)));

  trace_in();

  if (!adev) {
    fprintf(stderr, "%s allocate for alsa_device_t failed\n", __func__);
    return NULL;
  }

  // set default values for alsa device
  adev->handle = NULL;  // allocated by snd_pcm_open
  adev->name =  const_cast<char *>("default");
  adev->format = SND_PCM_FORMAT_S16;
  adev->direct = SND_PCM_STREAM_PLAYBACK;
  adev->rate = 48000;
  adev->channels = 2;
  adev->buffer_time = 0;
  adev->period_time = 0;
  adev->nperiods = 4;  // default periods number (4)
  adev->mode = 0;

  trace_out();

  return adev;
}

static int set_hwparams(alsa_device_t *adev, snd_pcm_hw_params_t *params,
                        snd_pcm_access_t access) {
  unsigned int rrate;
  int err;
  snd_pcm_uframes_t period_size_min;
  snd_pcm_uframes_t period_size_max;
  snd_pcm_uframes_t buffer_size_min;
  snd_pcm_uframes_t buffer_size_max;

  trace_in();

  if (!adev || !adev->handle) return -EINVAL;

  snd_pcm_t *handle = adev->handle;

  /* choose all parameters */
  err = snd_pcm_hw_params_any(handle, params);
  if (err < 0) {
    fprintf(
        stderr,
        "Broken configuration for playback: no configurations available: %s\n",
        snd_strerror(err));
    return err;
  }

  if (dump_hw_params) {
    fprintf(stderr, "HW Params of device \"%s\":\n", snd_pcm_name(handle));
    fprintf(stderr, "--------------------\n");
    snd_pcm_hw_params_dump(params, log);
    fprintf(stderr, "--------------------\n");
  }

  /* set the interleaved read/write format */
  err = snd_pcm_hw_params_set_access(handle, params, access);
  if (err < 0) {
    fprintf(stderr, "Access type not available for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  /* set the sample format */
  err = snd_pcm_hw_params_set_format(handle, params, adev->format);
  if (err < 0) {
    fprintf(stderr, "Sample format not available for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  /* set the count of channels */
  err = snd_pcm_hw_params_set_channels(handle, params, adev->channels);
  if (err < 0) {
    fprintf(stderr, "Channels count (%i) not available for playbacks: %s\n",
            adev->channels, snd_strerror(err));
    return err;
  }

  /* set the stream rate */
  rrate = adev->rate;
  err = snd_pcm_hw_params_set_rate(handle, params, adev->rate, 0);
  if (err < 0) {
    fprintf(stderr, "Rate %iHz not available for playback: %s\n", adev->rate,
            snd_strerror(err));
    return err;
  }

  if (rrate != adev->rate) {
    fprintf(stderr, "Rate doesn't match (requested %iHz, get %iHz, err %d)\n",
            adev->rate, rrate, err);
    return -EINVAL;
  }
  printf("Rate set to %iHz (requested %iHz)\n", rrate, adev->rate);

  /* set the buffer time */
  err = snd_pcm_hw_params_get_buffer_size_min(params, &buffer_size_min);
  err = snd_pcm_hw_params_get_buffer_size_max(params, &buffer_size_max);
  err = snd_pcm_hw_params_get_period_size_min(params, &period_size_min, NULL);
  err = snd_pcm_hw_params_get_period_size_max(params, &period_size_max, NULL);
  printf("Buffer size range from %lu to %lu\n", buffer_size_min,
         buffer_size_max);
  printf("Period size range from %lu to %lu\n", period_size_min,
         period_size_max);

  if (adev->period_size > 0) {
    printf("Requested period size %lu frames\n", adev->period_size);
    err = snd_pcm_hw_params_set_period_size_near(handle, params,
                                                 &adev->period_size, NULL);
    if (err < 0) {
      fprintf(stderr, "Unable to set period size %lu frames for playback: %s\n",
              adev->period_size, snd_strerror(err));
      return err;
    }
  }

  if (adev->period_time > 0) {
    printf("Requested period time %u us\n", adev->period_time);
    err = snd_pcm_hw_params_set_period_time_near(handle, params,
                                                 &adev->period_time, NULL);
    if (err < 0) {
      fprintf(stderr, "Unable to set period time %u us for playback: %s\n",
              adev->period_time, snd_strerror(err));
      return err;
    }
  }

  if (adev->buffer_time > 0) {
    printf("Requested buffer time %u us\n", adev->buffer_time);
    err = snd_pcm_hw_params_set_buffer_time_near(handle, params,
                                                 &adev->buffer_time, NULL);
    if (err < 0) {
      fprintf(stderr, "Unable to set buffer time %u us for playback: %s\n",
              adev->buffer_time, snd_strerror(err));
      return err;
    }
  }

  if (!adev->buffer_time && !adev->period_time && !adev->period_size) {
    adev->buffer_size = buffer_size_max;
    if (!adev->period_time)
      adev->buffer_size = (adev->buffer_size / adev->nperiods) * adev->nperiods;
    printf("Using max buffer size %lu\n", adev->buffer_size);

    err = snd_pcm_hw_params_set_buffer_size_near(handle, params,
                                                 &adev->buffer_size);
    if (err < 0) {
      fprintf(stderr, "Unable to set buffer size %lu for playback: %s\n",
              adev->buffer_size, snd_strerror(err));
      return err;
    }
  }

  if (!adev->buffer_time || !adev->period_time) {
    printf("Periods = %u\n", adev->nperiods);
    err = snd_pcm_hw_params_set_periods_near(handle, params, &adev->nperiods,
                                             NULL);
    if (err < 0) {
      fprintf(stderr, "Unable to set nperiods %u for playback: %s\n",
              adev->nperiods, snd_strerror(err));
      return err;
    }
  }

  /* write the parameters to device */
  err = snd_pcm_hw_params(handle, params);
  if (err < 0) {
    fprintf(stderr, "Unable to set hw params for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  if (dump_hw_params) {
    fprintf(stderr, "Applied HW Params of device \"%s\":\n",
            snd_pcm_name(handle));
    fprintf(stderr, "--------------------\n");
    snd_pcm_hw_params_dump(params, log);
    fprintf(stderr, "--------------------\n");
  }

  snd_pcm_hw_params_get_buffer_size(params, &adev->buffer_size);
  snd_pcm_hw_params_get_period_size(params, &adev->period_size, NULL);
  printf("was set period_size = %lu\n", adev->period_size);
  printf("was set buffer_size = %lu\n", adev->buffer_size);

  if (2 * adev->period_size > adev->buffer_size) {
    fprintf(stderr, "buffer to small, could not use\n");
    return -EINVAL;
  }

  trace_out();

  return 0;
}

static int set_swparams(alsa_device_t *adev, snd_pcm_sw_params_t *swparams) {
  int err;

  // printf("do nothing for alsa swparams. but just use default params\n");
  /* do nothing for software params first */
  // return 0;

  trace_in();

  if (!adev || !adev->handle) return -EINVAL;

  snd_pcm_t *handle = adev->handle;

  /* get the current swparams */
  err = snd_pcm_sw_params_current(handle, swparams);
  if (err < 0) {
    fprintf(stderr, "Unable to determine current swparams for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  /* start the transfer when a buffer is full */
  if (adev->direct == SND_PCM_STREAM_PLAYBACK) {
    err = snd_pcm_sw_params_set_start_threshold(handle, swparams,
                                                adev->buffer_size);
    if (err < 0) {
      fprintf(stderr, "Unable to set start threshold mode for playback: %s\n",
              snd_strerror(err));
      return err;
    }
  } else {
    err = snd_pcm_sw_params_set_start_threshold(handle, swparams, 1);
    if (err < 0) {
      fprintf(stderr, "Unable to set start threshold mode for playback: %s\n",
              snd_strerror(err));
      return err;
    }
  }

  /* allow the transfer when at least period_size frames can be processed */
  err = snd_pcm_sw_params_set_avail_min(handle, swparams, adev->period_size);
  if (err < 0) {
    fprintf(stderr, "Unable to set avail min for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  /* write the parameters to the playback device */
  err = snd_pcm_sw_params(handle, swparams);
  if (err < 0) {
    fprintf(stderr, "Unable to set sw params for playback: %s\n",
            snd_strerror(err));
    return err;
  }

  trace_out();

  return 0;
}

int alsa_device_init(alsa_device_t *adev) {
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_sw_params_t *swparams;
  int r;

  trace_in();

  if (!adev) return -EINVAL;

  printf("%s, snd_pcm_open. handle(%p), name(%s), direct(%d), mode(0)\n",
         __func__, adev->handle, adev->name, adev->direct);
  /* open speaker */
  r = snd_pcm_open(&adev->handle, adev->name, adev->direct, 0);
  if (r < 0) {
    fprintf(stderr, "%s snd_pcm_open %s failed\n", __func__, adev->name);
    goto err1;
  }

  printf("snd_pcm_open succeed. name(%s), handle(%p)\n", adev->name,
         adev->handle);

  /* pcm hwparams */
  if (snd_pcm_hw_params_malloc(&hwparams) < 0) goto err1;

  r = set_hwparams(adev, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (r < 0) {
    fprintf(stderr, "Setting of hwparams failed: %s\n", snd_strerror(r));
    goto err2;
  }

  /* pcm swparams */
  if (snd_pcm_sw_params_malloc(&swparams) < 0) goto err2;

  r = set_swparams(adev, swparams);
  if (r < 0) {
    fprintf(stderr, "Setting of swparams failed: %s\n", snd_strerror(r));
    goto err2;
  }

  printf("%s. hwparams(%p), swparams(%p)\n", __func__, hwparams, swparams);
  snd_pcm_hw_params_free(hwparams);
  snd_pcm_sw_params_free(swparams);

  trace_out();

  return 0;

err2:
  snd_pcm_hw_params_free(hwparams);
  snd_pcm_close(adev->handle);

err1:
  return r;
}

int alsa_device_read(alsa_device_t *adev, void *buffer,
                     snd_pcm_uframes_t frames) {
  int rc;

  trace_in();

  if (!adev || !adev->handle) return -EINVAL;

#if 0
  printf("%s snd_pcm_readi begin. handle(%p), name(%s), frames(%d)\n",
    __func__, adev->handle, adev->name, static_cast<int>(frames));
#endif

  rc = snd_pcm_readi(adev->handle, buffer, frames);

  // printf("%s snd_pcm_readi done\n", __func__);
  if (rc == -EPIPE) {
    /* EPIPE means overrun */
    fprintf(stderr, "%s overrun occurred\n", adev->name);
    xrun(adev->handle, SND_PCM_STREAM_CAPTURE);
  } else if (rc < 0) {
    fprintf(stderr, "%s error from readi: %d (%s)\n", adev->name, rc, snd_strerror(rc));
  } else {
    if (rc != static_cast<int>(frames)) {
      fprintf(stderr, "%s short read, read %d frames\n", adev->name, rc);
    }
  }

  trace_out();

  return rc;
}

int alsa_device_write(alsa_device_t *adev, void *buffer,
                      snd_pcm_uframes_t frames) {
  int rc;

  trace_in();

  if (!adev || !adev->handle) return -EINVAL;

#if 0
  printf("%s snd_pcm_writei begin. handle(%p), name(%s), frames(%d)\n",
    __func__, adev->handle, adev->name, static_cast<int>(frames));
#endif

  rc = snd_pcm_writei(adev->handle, buffer, frames);

  // printf("%s snd_pcm_writei done\n", __func__);

  if (rc == -EPIPE) {
    /* EPIPE means underrun */
    fprintf(stderr, "%s underrun occurred\n", adev->name);
    xrun(adev->handle, SND_PCM_STREAM_PLAYBACK);
  } else if (rc < 0) {
    fprintf(stderr, "%s error from writei: %d(%s)\n", adev->name, rc, snd_strerror(rc));
  } else {
    if (rc != static_cast<int>(frames)) {
      fprintf(stderr, "%s short read, write %d frames\n", adev->name, rc);
    }
  }

  trace_out();

  return rc;
}

void alsa_device_deinit(alsa_device_t *adev) {
  trace_in();

  if (!adev || !adev->handle) return;

  snd_pcm_drain(adev->handle);
  snd_pcm_close(adev->handle);

  trace_out();
}

void alsa_device_free(alsa_device_t *obj) {
  trace_in();

  if (obj) free(obj);

  trace_out();
}
