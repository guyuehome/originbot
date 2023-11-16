/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>

#include "vio/hb_vot.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"

#include "x3_vio_vot.h"

int x3_vot_sendframe(VOT_FRAME_INFO_S *out_frame)
{
    int ret = 0;
    ret = HB_VOT_SendFrame(0, 0, out_frame, -1);

    return ret;
}

int x3_vot_init(x3_vot_info_t *vot_info)
{
    int ret = 0;

    ret = HB_VOT_SetPubAttr(0, &vot_info->m_devAttr);
    if (ret) {
        ROS_printf("HB_VOT_SetPubAttr failed\n");
        goto err;
    }

    ret = HB_VOT_Enable(0);
    if (ret) {
        ROS_printf("HB_VOT_Enable failed.\n");
        goto err;
    }

    ret = HB_VOT_SetVideoLayerAttr(0, &vot_info->m_stLayerAttr);
    if (ret) {
        ROS_printf("HB_VOT_SetVideoLayerAttr failed.\n");
        goto err;
    }

    ret = HB_VOT_EnableVideoLayer(0);
    if (ret) {
        ROS_printf("HB_VOT_EnableVideoLayer failed.\n");
        HB_VOT_Disable(0);
        goto err;
    }

    ret = HB_VOT_SetChnAttr(0, 0, &vot_info->m_stChnAttr);
    if (ret) {
        ROS_printf("HB_VOT_SetChnAttr 0: %d\n", ret);
        HB_VOT_DisableVideoLayer(0);
        HB_VOT_Disable(0);
        goto err;
    }

    ret = HB_VOT_SetChnCrop(0, 0, &vot_info->m_cropAttrs);
    ROS_printf("HB_VOT_SetChnCrop: %d\n", ret);

    ret = HB_VOT_EnableChn(0, 0);
    if (ret) {
        ROS_printf("HB_VOT_EnableChn: %d\n", ret);
        HB_VOT_DisableVideoLayer(0);
        HB_VOT_Disable(0);
        goto err;
    }

err:
    return ret;
}

int x3_vot_deinit()
{
    int ret = 0;

    ret = HB_VOT_DisableChn(0, 0);
    if (ret) {
        ROS_printf("HB_VOT_DisableChn failed.\n");
    }

    ret = HB_VOT_DisableVideoLayer(0);
    if (ret) {
        ROS_printf("HB_VOT_DisableVideoLayer failed.\n");
    }

    ret = HB_VOT_Disable(0);
    if (ret) {
        ROS_printf("HB_VOT_Disable failed.\n");
    }

    return 0;
}

int x3_vot_wb_init(int wb_src, int wb_format)
{
    int ret = 0;
    VOT_WB_ATTR_S stWbAttr;

    stWbAttr.wb_src = wb_src;
    stWbAttr.wb_format = wb_format;
    HB_VOT_SetWBAttr(0, &stWbAttr);

    ret = HB_VOT_EnableWB(0);
    if (ret) {
        ROS_printf("HB_VOT_EnableWB failed.\n");
        return -1;
    }

    return 0;
}

int x3_vot_wb_bind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		ROS_printf("HB_SYS_Bind failed\n");

    return ret;
}

int x3_vot_wb_bind_venc(VENC_CHN vencChn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = vencChn;
    dst_mod.s32ChnId = vencChn;
    ret = HB_SYS_Bind(&src_mod, &dst_mod);
    if (ret != 0) {
      ROS_printf("HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int x3_vot_wb_unbind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret != 0)
		ROS_printf("HB_SYS_Bind failed\n");

    return ret;
}

int x3_vot_wb_unbind_venc(VENC_CHN vencChn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = vencChn;
    dst_mod.s32ChnId = vencChn;
    ret = HB_SYS_UnBind(&src_mod, &dst_mod);
    if (ret != 0) {
      ROS_printf("HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int x3_vot_wb_deinit()
{
    int ret = 0;

    ret = HB_VOT_DisableWB(0);
    if (ret) {
        ROS_printf("HB_VOT_DisableWB failed.\n");
        return -1;
    }
    return 0;
}
