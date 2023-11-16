/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef SW_VDEC
#include "vio/hb_comm_vdec.h"
#include "vio/hb_vdec.h"
#include "vio/hb_comm_video.h"
#include "vio/hb_type.h"
#include "vio/hb_common.h"
#include "utils/utils_log.h"
#include "logging.h"

#include "x3_vio_vdec.h"

int x3_vdec_init(VDEC_CHN vdecChn, VDEC_CHN_ATTR_S* vdecChnAttr)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_CreateChn(vdecChn, vdecChnAttr);
    if (s32Ret != 0) {
        ROS_printf("HB_VDEC_CreateChn failed %x\n", s32Ret);
        return s32Ret;
    }

    s32Ret = HB_VDEC_SetChnAttr(vdecChn, vdecChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf("HB_VDEC_SetChnAttr failed %x\n", s32Ret);
        return s32Ret;
    }

	LOGI_print("ok!\n");
    return 0;
}

int x3_vdec_start(VDEC_CHN vdecChn)
{
	int s32Ret = HB_VDEC_StartRecvStream(vdecChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VDEC_StartRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }
	LOGI_print("ok!\n");
    return 0;
}

int x3_vdec_stop(VDEC_CHN vdecChn)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_StopRecvStream(vdecChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VDEC_StopRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }
	LOGI_print("ok!\n");

    return 0;
}

int x3_vdec_deinit(VDEC_CHN vdecChn)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_DestroyChn(vdecChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VDEC_ReleaseFrame failed %d\n", s32Ret);
        return s32Ret;
    }
	LOGI_print("ok!\n");
    return 0;
}

#endif