/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VENC_H_
#define X3_VIO_VENC_H_

#include <stdio.h>

#include "vio/hb_comm_venc.h"
#include "vio/hb_venc.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int x3_venc_common_init();
int x3_venc_common_deinit();
int x3_venc_init(VENC_CHN chn_id, VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_reinit(int Vechn, VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_start(int VeChn);
int x3_venc_stop(int VeChn);
int x3_venc_deinit(int VeChn);
int x3_set_vencfps(int VeChn, int InputFps, int OutputFps);
int x3_venc_setjpegmode(int vechn, HB_VENC_JPEG_ENCODE_MODE_E mode);
int x3_venc_setrcattr(int VeChn);
     
int x3_venc_setroi(int VeChn, VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_seth264_profile(int VeChn, VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_set_rcmode(int VeChn, VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_set_rcmodes(int VeChn);

uint32_t x3_venc_get_framerate(VENC_CHN_ATTR_S *vencChnAttr);
uint32_t x3_venc_get_bitrate(VENC_CHN_ATTR_S *vencChnAttr);
int x3_venc_set_bitrate(int VeChn, int bitrate);

int x3_venc_get_chn_attr(int Vechn, VENC_CHN_ATTR_S *vencChnAttr);
uint32_t x3_venc_get_gop(VENC_CHN_ATTR_S *vencChnAttr);
uint32_t x3_venc_save_stream(PAYLOAD_TYPE_E enType, FILE *pFd, VIDEO_STREAM_S *pstStream);
uint32_t x3_venc_get_file_postfix(PAYLOAD_TYPE_E enPayload, char *szFilePostfix);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif // X3_VIO_VENC_H_