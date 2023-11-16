/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VOT_H_
#define X3_VIO_VOT_H_

#include "vio/hb_common_vot.h"

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int x3_vot_sendframe(VOT_FRAME_INFO_S *out_frame);
int x3_vot_init();
int x3_vot_deinit();

int x3_vot_wb_init(int wb_src, int wb_format);
int x3_vot_wb_bind_vps(int vps_grp, int vps_chn);
int x3_vot_wb_bind_venc(VENC_CHN vencChn);
int x3_vot_wb_unbind_vps(int vps_grp, int vps_chn);
int x3_vot_wb_unbind_venc(VENC_CHN vencChn);
int x3_vot_wb_deinit();

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_VIO_VOT_H_