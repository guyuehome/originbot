/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

//#include "hb_comm_venc.h"
#include "vio/hb_vdec.h"
//#include "hb_venc.h"
#include "vio/hb_vp_api.h"
#include "vio/hb_vio_interface.h"
//#include "utils/utils_log.h"
//#include "logging.h"

#include "x3_vio_venc.h"
extern "C" int ROS_printf(char *fmt, ...);

void print_h264cvb_attr(VENC_RC_ATTR_S *stRcAttr)
{
	ROS_printf("u32BitRate: %d\n",  stRcAttr->stH264Cbr.u32BitRate);
	ROS_printf("u32FrameRate: %d\n",	stRcAttr->stH264Cbr.u32FrameRate);
	ROS_printf("u32IntraPeriod: %d\n",  stRcAttr->stH264Cbr.u32IntraPeriod);
	ROS_printf("u32VbvBufferSize: %d\n",	stRcAttr->stH264Cbr.u32VbvBufferSize);
	ROS_printf("u32IntraQp: %d\n",  stRcAttr->stH264Cbr.u32IntraQp);
	ROS_printf("u32InitialRcQp: %d\n",  stRcAttr->stH264Cbr.u32InitialRcQp);
	ROS_printf("bMbLevelRcEnable: %d\n",	stRcAttr->stH264Cbr.bMbLevelRcEnable);
	ROS_printf("u32MaxIQp: %d\n",  stRcAttr->stH264Cbr.u32MaxIQp);
	ROS_printf("u32MinIQp: %d\n",  stRcAttr->stH264Cbr.u32MinIQp);
	ROS_printf("u32MaxPQp: %d\n",  stRcAttr->stH264Cbr.u32MaxPQp);
	ROS_printf("u32MinPQp: %d\n",  stRcAttr->stH264Cbr.u32MinPQp);
	ROS_printf("u32MaxBQp: %d\n",  stRcAttr->stH264Cbr.u32MaxBQp);
	ROS_printf("u32MinBQp: %d\n",  stRcAttr->stH264Cbr.u32MinBQp);
	ROS_printf("bHvsQpEnable: %d\n",	stRcAttr->stH264Cbr.bHvsQpEnable);
	ROS_printf("s32HvsQpScale: %d\n",  stRcAttr->stH264Cbr.s32HvsQpScale);
	ROS_printf("u32MaxDeltaQp: %d\n",  stRcAttr->stH264Cbr.u32MaxDeltaQp);
	ROS_printf("bQpMapEnable: %d\n",	stRcAttr->stH264Cbr.bQpMapEnable);
	return;
}

void print_venc_chn_attr(VENC_CHN_ATTR_S *chn_attr)
{
	ROS_printf("VENC channle attr:\n");
	ROS_printf("enType: %d\n",  chn_attr->stVencAttr.enType);
	ROS_printf("u32PicWidth: %d\n",  chn_attr->stVencAttr.u32PicWidth);
	ROS_printf("u32PicHeight: %d\n",	chn_attr->stVencAttr.u32PicHeight);
	ROS_printf("enMirrorFlip: %d\n",	chn_attr->stVencAttr.enMirrorFlip);
	ROS_printf("enRotation: %d\n",  chn_attr->stVencAttr.enRotation);
	ROS_printf("bEnable: %d\n",  chn_attr->stVencAttr.stCropCfg.bEnable);
	ROS_printf("bEnableUserPts: %d\n",  chn_attr->stVencAttr.bEnableUserPts);
	ROS_printf("s32BufJoint: %d\n",  chn_attr->stVencAttr.s32BufJoint);
	ROS_printf("s32BufJointSize: %d\n",  chn_attr->stVencAttr.s32BufJointSize);
	ROS_printf("enPixelFormat: %d\n",  chn_attr->stVencAttr.enPixelFormat);
	ROS_printf("u32BitStreamBufferCount: %d\n",  chn_attr->stVencAttr.u32BitStreamBufferCount);
	ROS_printf("u32FrameBufferCount: %d\n",  chn_attr->stVencAttr.u32FrameBufferCount);
	ROS_printf("bExternalFreamBuffer: %d\n",	chn_attr->stVencAttr.bExternalFreamBuffer);
	ROS_printf("vlc_buf_size: %d\n",	chn_attr->stVencAttr.vlc_buf_size);
	ROS_printf("u32BitStreamBufSize: %d\n",  chn_attr->stVencAttr.u32BitStreamBufSize);
	ROS_printf("enRcMode: %d\n",	chn_attr->stRcAttr.enRcMode);
	if (chn_attr->stRcAttr.enRcMode == VENC_RC_MODE_H264CBR) {
		print_h264cvb_attr(&chn_attr->stRcAttr);
	}
	ROS_printf("h264_profile: %d\n",	chn_attr->stVencAttr.stAttrH264.h264_profile);
	ROS_printf("h264_level: %d\n",  chn_attr->stVencAttr.stAttrH264.h264_level);
	ROS_printf("u32GopPresetIdx: %d\n",  chn_attr->stGopAttr.u32GopPresetIdx);
	ROS_printf("s32DecodingRefreshType: %d\n",  chn_attr->stGopAttr.s32DecodingRefreshType);
	
	return;
}

int x3_venc_h264cbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H264_CBR_S *pstH264Cbr);

int x3_venc_h264vbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H264_VBR_S *pstH264Vbr);

int x3_venc_h264avbr(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_AVBR_S *pstH264AVbr);

int x3_venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_FIXQP_S *pstH264FixQp);

int x3_venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_QPMAP_S *pstH264QpMap);

int x3_venc_h265cbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_CBR_S *pstH265Cbr);

int x3_venc_h265vbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_VBR_S *pstH265Vbr);

int x3_venc_h265avbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_AVBR_S *pstH265AVbr);

int x3_venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_FIXQP_S *pstH265FixQp);

int x3_venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_QPMAP_S *pstH265QpMap);

int x3_venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_MJPEG_FIXQP_S *pstMjpegFixQp);

int x3_venc_setgop(VENC_GOP_ATTR_S *pstGopAttr, int presetidx,
                                int refreshtype, int qp, int intraqp);
int x3_venc_setRefParam(int Vechn, uint32_t longterm_pic_period,
                    uint32_t longterm_pic_using_period);

// #define VENC_BIND
int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    int streambuf = 2*1024*1024;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

    // pVencChnAttr->stVencAttr.u32InputFrameRate = 30;
    // pVencChnAttr->stVencAttr.u32OutputFrameRate = 30;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
    pVencChnAttr->stVencAttr.bEnableUserPts = HB_TRUE;
    pVencChnAttr->stVencAttr.s32BufJoint = 0;
    pVencChnAttr->stVencAttr.s32BufJointSize = 8000000;

    if (p_Width * p_Height > 2688 * 1522) {
        streambuf = 3 * 1024 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900*1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        streambuf = 2048 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        streambuf = 1536 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else if (p_Width * p_Height > 704 * 576) {
        streambuf = 512 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        streambuf = 256 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048*1024;
    }
    streambuf = (p_Width * p_Height)&0xfffff000;
    // pVencChnAttr->stVencAttr.vlc_buf_size = 0;
    //     ((((p_Width * p_Height) * 9) >> 3) * 3) >> 1;
    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        // streambuf = (p_Width * p_Height * 3/2)&0xfffff000;
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 30;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
        pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.s32X = 1920;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.s32Y = 1080;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.u32Width = 1920;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.u32Height = 1080;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 4;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 3;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    }

    if (p_enType == PT_H265) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 30;
    }
    if (p_enType == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        // pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 30;
        pVencChnAttr->stVencAttr.stAttrH264.h264_profile = (VENC_H264_PROFILE_E)0;
        pVencChnAttr->stVencAttr.stAttrH264.h264_level = (HB_H264_LEVEL_E)0;
    }

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}

void venc_thread(void *vencpram)
{
	ROS_printf("venc chn xxx thread exit\n");
}

int x3_venc_setjpegmode(int vechn, HB_VENC_JPEG_ENCODE_MODE_E mode)
{
    return HB_VENC_SetJpegEncodeMode(vechn, mode);
}

int x3_venc_common_init()
{
    int s32Ret;

    s32Ret = HB_VENC_Module_Init();
    if (s32Ret) {
        ROS_printf("HB_VENC_Module_Init: %d\n", s32Ret);
    }

    return s32Ret;
}

int x3_venc_common_deinit()
{
    int s32Ret;

    s32Ret = HB_VENC_Module_Uninit();
    if (s32Ret) {
        ROS_printf("HB_VENC_Module_Init: %d\n", s32Ret);
    }

    return s32Ret;
}

int x3_venc_setrcattr(int VeChn)
{
    VENC_RC_ATTR_S stRcParam;
    int s32Ret;

    s32Ret = HB_VENC_GetRcParam(VeChn, &stRcParam);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_GetRcParam failed.\n");
        return -1;
    }
    ROS_printf("stRcParam.enRcMode: %d\n", stRcParam.enRcMode);
    // stRcParam.stH264Cbr.u32BitRate = 2000;
    // stRcParam.stH264Cbr.u32IntraPeriod = 60;
    // stRcParam.stH264Cbr.u32FrameRate = 25;
    stRcParam.stH264AVbr.u32BitRate = 8192;
    stRcParam.stH264AVbr.u32FrameRate = 25;
    stRcParam.stH264AVbr.u32IntraQp = 30;
    stRcParam.stH264AVbr.u32InitialRcQp = 45;
    stRcParam.stH264AVbr.u32IntraPeriod = 50;
    stRcParam.stH264AVbr.u32VbvBufferSize = 3000;
    stRcParam.stH264AVbr.bMbLevelRcEnable = HB_FALSE;
    stRcParam.stH264AVbr.u32MaxIQp = 51;
    stRcParam.stH264AVbr.u32MinIQp = 28;
    stRcParam.stH264AVbr.u32MaxPQp = 51;
    stRcParam.stH264AVbr.u32MinPQp = 28;
    stRcParam.stH264AVbr.u32MaxBQp = 51;
    stRcParam.stH264AVbr.u32MinBQp = 28;
    stRcParam.stH264AVbr.bHvsQpEnable = HB_TRUE;
    stRcParam.stH264AVbr.s32HvsQpScale = 2;
    stRcParam.stH264AVbr.u32MaxDeltaQp = 3;
    stRcParam.stH264AVbr.bQpMapEnable = HB_FALSE;

    s32Ret = HB_VENC_SetRcParam(VeChn, &stRcParam);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetRcParam failed.\n");
        return -1;
    }

    return 0;
}

int x3_venc_initattr(int VeChn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;
    VENC_RC_ATTR_S *pstRcParam;
    /*VENC_PARAM_MOD_S stModParam;*/
    PAYLOAD_TYPE_E ptype = vencChnAttr->stVencAttr.enType;

    /*VencChnAttrInit(vencChnAttr, ptype, width, height, HB_PIXEL_FORMAT_NV12);*/
    // if (VeChn == 0) {
    //     vencChnAttr->stVencAttr.stCropCfg.bEnable = HB_TRUE;
    // }

	/*print_venc_chn_attr(vencChnAttr);*/

    s32Ret = HB_VENC_CreateChn(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_CreateChn %d failed, %d.\n", VeChn, s32Ret);
        return -1;
    }

    // stModParam.u32OneStreamBuffer = 0;
    // s32Ret = HB_VENC_SetModParam(VeChn, &stModParam);
    // if (s32Ret != 0) {
    //     ROS_printf("HB_VENC_SetModParam %d failed, %d.\n", VeChn, s32Ret);
    //     return -1;
    // }

    if (ptype == PT_H264) {
		// 1、在默认参数配置中已经配置好了部分RcAttr的属性，此处需要先把预先配置好的属性备份一下
		// 2、通过HB_VENC_GetRcParam来获取完整的配置
		// 3、把备份的预先设置的属性回填
		VENC_RC_ATTR_S temp_rc_attr;// = {0};
		if (vencChnAttr->stRcAttr.enRcMode == VENC_RC_MODE_H264CBR) {
			// union 数据类型用memcpy会导致数据异常，所以需要实现字段赋值
			x3_venc_h264cbr(&temp_rc_attr, &vencChnAttr->stRcAttr.stH264Cbr);
		}
		pstRcParam = &(vencChnAttr->stRcAttr);
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
		/*print_h264cvb_attr(&temp_rc_attr);*/
        switch (vencChnAttr->stRcAttr.enRcMode) {
        case VENC_RC_MODE_H264CBR:
        {
        	// 3、把备份的预先设置的属性回填
            x3_venc_h264cbr(pstRcParam, &temp_rc_attr.stH264Cbr);
			/*print_venc_chn_attr(vencChnAttr);*/
            break;
        }
        case VENC_RC_MODE_H264VBR:
        {
            VENC_H264_VBR_S stH264Vbr;
            memset(&stH264Vbr, 0, sizeof(stH264Vbr));
            stH264Vbr.bQpMapEnable = HB_FALSE;
            stH264Vbr.u32FrameRate = 30;
            stH264Vbr.u32IntraPeriod = 30;
            stH264Vbr.u32IntraQp = 35;
            x3_venc_h264vbr(pstRcParam, &stH264Vbr);
            break;
        }
        case VENC_RC_MODE_H264AVBR:
        {
            VENC_H264_AVBR_S stH264Avbr;
            memset(&stH264Avbr, 0, sizeof(stH264Avbr));
            stH264Avbr.u32BitRate = vencChnAttr->stRcAttr.stH264AVbr.u32BitRate;
            stH264Avbr.u32FrameRate = 25;
            stH264Avbr.u32IntraPeriod = 50;
            stH264Avbr.u32VbvBufferSize = 3000;
            stH264Avbr.u32IntraQp = 30;
            stH264Avbr.u32InitialRcQp = 45;
            stH264Avbr.bMbLevelRcEnable = HB_FALSE;
            stH264Avbr.u32MaxIQp = 51;
            stH264Avbr.u32MinIQp = 28;
            stH264Avbr.u32MaxPQp = 51;
            stH264Avbr.u32MinPQp = 28;
            stH264Avbr.u32MaxBQp = 51;
            stH264Avbr.u32MinBQp = 28;
            stH264Avbr.bHvsQpEnable = HB_TRUE;
            stH264Avbr.s32HvsQpScale = 2;
            stH264Avbr.u32MaxDeltaQp = 3;
            stH264Avbr.bQpMapEnable = HB_FALSE;
            x3_venc_h264avbr(pstRcParam, &stH264Avbr);
            break;
        }
        case VENC_RC_MODE_H264FIXQP:
        {
            VENC_H264_FIXQP_S stH264FixQp;
            memset(&stH264FixQp, 0, sizeof(stH264FixQp));
            stH264FixQp.u32FrameRate = 30;
            stH264FixQp.u32IntraPeriod = 30;
            stH264FixQp.u32IQp = 35;
            stH264FixQp.u32PQp = 35;
            stH264FixQp.u32BQp = 35;
            x3_venc_h264fixqp(pstRcParam, &stH264FixQp);
            break;
        }
        case VENC_RC_MODE_H264QPMAP:
        {
            VENC_H264_QPMAP_S stH264QpMap;
            memset(&stH264QpMap, 0, sizeof(stH264QpMap));
            stH264QpMap.u32FrameRate = 30;
            stH264QpMap.u32IntraPeriod = 30;
            stH264QpMap.u32QpMapArrayCount =
                ((vencChnAttr->stVencAttr.u32PicWidth+0x0f)&~0x0f)/16 *
                ((vencChnAttr->stVencAttr.u32PicHeight+0x0f)&~0x0f)/16;
            stH264QpMap.u32QpMapArray = (unsigned char*)malloc(stH264QpMap.u32QpMapArrayCount);
            for(int i = 0; i < stH264QpMap.u32QpMapArrayCount; i++) {
                stH264QpMap.u32QpMapArray[i] = 30;
            }
            x3_venc_h264qpmap(pstRcParam, &stH264QpMap);
            ROS_printf("x3_venc_h264qpmap 30\n");
            break;
        }
        default:
            break;
        }
        ROS_printf(" vencChnAttr->stRcAttr.enRcMode = %d\n",
                vencChnAttr->stRcAttr.enRcMode);
    } else if (ptype == PT_H265) {
        pstRcParam = &(vencChnAttr->stRcAttr);
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        switch (vencChnAttr->stRcAttr.enRcMode) {
        case VENC_RC_MODE_H265CBR:
        {
            VENC_H265_CBR_S stH265Cbr;
            memset(&stH265Cbr, 0, sizeof(stH265Cbr));
            stH265Cbr.u32IntraPeriod = 30;
            stH265Cbr.u32IntraQp = 30;
            stH265Cbr.u32BitRate = vencChnAttr->stRcAttr.stH265Cbr.u32BitRate;
            stH265Cbr.u32FrameRate = 30;
            stH265Cbr.u32InitialRcQp = 45;
            stH265Cbr.u32VbvBufferSize = 3000;
            stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
            stH265Cbr.u32MaxIQp = 45;
            stH265Cbr.u32MinIQp = 22;
            stH265Cbr.u32MaxPQp = 45;
            stH265Cbr.u32MinPQp = 22;
            stH265Cbr.u32MaxBQp = 45;
            stH265Cbr.u32MinBQp = 22;
            stH265Cbr.bHvsQpEnable = HB_FALSE;
            stH265Cbr.s32HvsQpScale = 2;
            stH265Cbr.u32MaxDeltaQp = 10;
            stH265Cbr.bQpMapEnable = HB_FALSE;
            x3_venc_h265cbr(pstRcParam, &stH265Cbr);
            break;
        }
        case VENC_RC_MODE_H265VBR:
        {
            VENC_H265_VBR_S stH265Vbr;
            memset(&stH265Vbr, 0, sizeof(stH265Vbr));
            stH265Vbr.u32IntraPeriod = 30;
            stH265Vbr.u32FrameRate = 30;
            stH265Vbr.bQpMapEnable = HB_FALSE;
            stH265Vbr.u32IntraQp = 32;
            x3_venc_h265vbr(pstRcParam, &stH265Vbr);
            break;
        }
        case VENC_RC_MODE_H265AVBR:
        {
            VENC_H265_AVBR_S stH265Avbr;
            memset(&stH265Avbr, 0, sizeof(stH265Avbr));
            stH265Avbr.u32IntraPeriod = 30;
            stH265Avbr.u32IntraQp = 30;
            stH265Avbr.u32BitRate = vencChnAttr->stRcAttr.stH265AVbr.u32BitRate;
            stH265Avbr.u32FrameRate = 30;
            stH265Avbr.u32InitialRcQp = 45;
            stH265Avbr.u32VbvBufferSize = 3000;
            stH265Avbr.bCtuLevelRcEnable = HB_FALSE;
            stH265Avbr.u32MaxIQp = 45;
            stH265Avbr.u32MinIQp = 22;
            stH265Avbr.u32MaxPQp = 45;
            stH265Avbr.u32MinPQp = 22;
            stH265Avbr.u32MaxBQp = 45;
            stH265Avbr.u32MinBQp = 22;
            stH265Avbr.bHvsQpEnable = HB_TRUE;
            stH265Avbr.s32HvsQpScale = 2;
            stH265Avbr.u32MaxDeltaQp = 10;
            stH265Avbr.bQpMapEnable = HB_FALSE;
            x3_venc_h265avbr(pstRcParam, &stH265Avbr);
            break;
        }
        case VENC_RC_MODE_H265FIXQP:
        {
            VENC_H265_FIXQP_S stH265FixQp;
            memset(&stH265FixQp, 0, sizeof(stH265FixQp));
            stH265FixQp.u32FrameRate = 30;
            stH265FixQp.u32IntraPeriod = 30;
            stH265FixQp.u32IQp = 35;
            stH265FixQp.u32PQp = 35;
            stH265FixQp.u32BQp = 35;
            x3_venc_h265fixqp(pstRcParam, &stH265FixQp);
            break;
        }
        case VENC_RC_MODE_H265QPMAP:
        {
            VENC_H265_QPMAP_S stH265QpMap;
            memset(&stH265QpMap, 0, sizeof(stH265QpMap));
            stH265QpMap.u32FrameRate = 30;
            stH265QpMap.u32IntraPeriod = 30;
            stH265QpMap.u32QpMapArrayCount =
				((vencChnAttr->stVencAttr.u32PicWidth+0x0f)&~0x0f)/16 *
				((vencChnAttr->stVencAttr.u32PicHeight+0x0f)&~0x0f)/16;
            stH265QpMap.u32QpMapArray = (unsigned char*)malloc(stH265QpMap.u32QpMapArrayCount);
            x3_venc_h265qpmap(pstRcParam, &stH265QpMap);
            break;
        }
        default:
            break;
        }
        ROS_printf(" m_VencChnAttr->stRcAttr.enRcMode = %d\n",
                vencChnAttr->stRcAttr.enRcMode);
    } else if (ptype == PT_MJPEG) {
        VENC_MJPEG_FIXQP_S stMjpegFixQp;
        pstRcParam = &(vencChnAttr->stRcAttr);
        vencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        memset(&stMjpegFixQp, 0,  sizeof(stMjpegFixQp));
        stMjpegFixQp.u32FrameRate = 30;
        stMjpegFixQp.u32QualityFactort = 50;
        x3_venc_mjpgfixqp(pstRcParam, &stMjpegFixQp);
    }

    return 0;
}

int x3_venc_init(int VeChn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;

    s32Ret = x3_venc_initattr(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_initattr failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, vencChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetChnAttr failed, s32Ret: %d\n", s32Ret);
        return -1;
    }

    return 0;
}

static int venc_setroi(int VeChn, int width, int height)
{
    VENC_ROI_ATTR_S pstRoiAttr;
    int stroi_map_len;
    int s32Ret;

    stroi_map_len = ((width+15) >> 4) * ((height+15) >> 4);

    memset(&pstRoiAttr, 0, sizeof(VENC_ROI_ATTR_S));
    pstRoiAttr.roi_enable = HB_TRUE;
    pstRoiAttr.roi_map_array_count = stroi_map_len;
    pstRoiAttr.roi_map_array =
        (unsigned char*)malloc(stroi_map_len * sizeof(unsigned char));
    for (int i = 0; i < stroi_map_len; i++) {
        pstRoiAttr.roi_map_array[i] = 51;
    }
    s32Ret = HB_VENC_SetRoiAttr(VeChn, &pstRoiAttr);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetRoiAttr %d failed, %dx%d\n", VeChn, width, height);
        return -1;
    }
    return s32Ret;
}

int x3_venc_setroi(int VeChn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;
    /*VENC_CHN_ATTR_S vencChnAttr;*/

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = x3_venc_initattr(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_initattr failed\n");
        return -1;
    }

    // setup user gop
    // x3_venc_setgop(&vencChnAttr.stGopAttr, 10, 2);
    // setup refparam
    // x3_venc_setRefParam(VeChn, 4, 2);
    s32Ret = venc_setroi(VeChn, vencChnAttr->stVencAttr.u32PicWidth,
    	vencChnAttr->stVencAttr.u32PicHeight);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_setroi failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, vencChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    x3_venc_start(VeChn);

    return 0;
}

int x3_venc_seth264_profile(int VeChn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;
    /*VENC_CHN_ATTR_S vencChnAttr;*/

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = x3_venc_initattr(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_initattr failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, vencChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    x3_venc_start(VeChn);

    return 0;
}

int x3_venc_set_rcmodes(int VeChn)
{
    int s32Ret;
    VENC_RC_ATTR_S stRcParam;
    HB_VENC_GetRcParam(VeChn, &stRcParam);
    if (stRcParam.enRcMode == VENC_RC_MODE_H265CBR) {
        stRcParam.stH265Cbr.u32IntraPeriod = 15;
        ROS_printf("xxxxxxxxxh265xxxxxxxxx\n");
    } else if (stRcParam.enRcMode == VENC_RC_MODE_H264CBR) {  
        stRcParam.stH264Cbr.u32IntraPeriod = 15;
        ROS_printf("xxxxxxxxxh264xxxxxxxxx\n");
    }
    s32Ret = HB_VENC_SetRcParam(VeChn, &stRcParam);

    return s32Ret;
}

int x3_venc_set_rcmode(int VeChn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;
    /*VENC_CHN_ATTR_S vencChnAttr;*/

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = x3_venc_initattr(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_initattr failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, vencChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    s32Ret = x3_venc_start(VeChn);

    return s32Ret;
}

int x3_venc_start(int VeChn)
{
    int s32Ret = 0;

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StartRecvFrame failed\n");
        return -1;
    }

    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_FALSE);
    // ROS_printf("HB_VENC_EnableIDR: %x\n", s32Ret);
    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_TRUE);
    // ROS_printf("HB_VENC_EnableIDR %x\n", s32Ret);
    return s32Ret;
}

int x3_venc_stop(int VeChn)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StopRecvFrame failed\n");
        return -1;
    }
	ROS_printf("ok!\n");

    return s32Ret;
}

// int x3_venc_reset(int VeChn, )

int x3_set_vencfps(int VeChn, int InputFps, int OutputFps)
{
    VENC_CHN_PARAM_S chnparam;

    HB_VENC_GetChnParam(VeChn, &chnparam);
    chnparam.stFrameRate.s32InputFrameRate = InputFps;
    chnparam.stFrameRate.s32OutputFrameRate = OutputFps;
    HB_VENC_SetChnParam(VeChn, &chnparam);
    return 0;
}

int x3_venc_deinit(int VeChn)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_DestroyChn failed\n");
        return -1;
    }
	ROS_printf("ok!\n");

    return s32Ret;
}

int x3_venc_reinit(int Vechn, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(Vechn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_StopRecvFrame %d failed\n", Vechn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(Vechn);
    if (s32Ret != 0) {
        ROS_printf("HB_VENC_DestroyChn %d failed\n", Vechn);
        return -1;
    }

    s32Ret = x3_venc_init(Vechn, vencChnAttr);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_init failed\n");
        return -1;
    }

    s32Ret = x3_venc_start(Vechn);
    if (s32Ret != 0) {
        ROS_printf("x3_venc_start failed\n");
        return -1;
    }

    return s32Ret;
}

int x3_venc_setRefParam(int Vechn, uint32_t longterm_pic_period,
                    uint32_t longterm_pic_using_period)
{
    VENC_REF_PARAM_S stRefParam;
    HB_VENC_GetRefParam(Vechn, &stRefParam);
    stRefParam.use_longterm = HB_TRUE;
    stRefParam.longterm_pic_period = longterm_pic_period;
    stRefParam.longterm_pic_using_period = longterm_pic_using_period;
    HB_VENC_SetRefParam(Vechn, &stRefParam);
    return 0;
}

int x3_venc_h264cbr(VENC_RC_ATTR_S *pstRcParam, VENC_H264_CBR_S *pstH264Cbr)
{
    pstRcParam->stH264Cbr.u32BitRate = pstH264Cbr->u32BitRate;
    pstRcParam->stH264Cbr.u32FrameRate = pstH264Cbr->u32FrameRate;
    pstRcParam->stH264Cbr.u32IntraPeriod = pstH264Cbr->u32IntraPeriod;
    pstRcParam->stH264Cbr.u32VbvBufferSize = pstH264Cbr->u32VbvBufferSize;
    pstRcParam->stH264Cbr.u32IntraQp = pstH264Cbr->u32IntraQp;
    pstRcParam->stH264Cbr.u32InitialRcQp = pstH264Cbr->u32InitialRcQp;
    pstRcParam->stH264Cbr.bMbLevelRcEnable = pstH264Cbr->bMbLevelRcEnable;
    pstRcParam->stH264Cbr.u32MaxIQp = pstH264Cbr->u32MaxIQp;
    pstRcParam->stH264Cbr.u32MinIQp = pstH264Cbr->u32MinIQp;
    pstRcParam->stH264Cbr.u32MaxPQp = pstH264Cbr->u32MaxPQp;
    pstRcParam->stH264Cbr.u32MinPQp = pstH264Cbr->u32MinPQp;
    pstRcParam->stH264Cbr.u32MaxBQp = pstH264Cbr->u32MaxBQp;
    pstRcParam->stH264Cbr.u32MinBQp = pstH264Cbr->u32MinBQp;
    pstRcParam->stH264Cbr.bHvsQpEnable = pstH264Cbr->bHvsQpEnable;
    pstRcParam->stH264Cbr.s32HvsQpScale = pstH264Cbr->s32HvsQpScale;
    pstRcParam->stH264Cbr.u32MaxDeltaQp = pstH264Cbr->u32MaxDeltaQp;
    pstRcParam->stH264Cbr.bQpMapEnable = pstH264Cbr->bQpMapEnable;

    return 0;
}

int x3_venc_h264vbr(VENC_RC_ATTR_S *pstRcParam, VENC_H264_VBR_S *pstH264Vbr)
{
    pstRcParam->stH264Vbr.u32IntraPeriod = pstH264Vbr->u32IntraPeriod;
    pstRcParam->stH264Vbr.u32IntraQp = pstH264Vbr->u32IntraQp;
    pstRcParam->stH264Vbr.u32FrameRate = pstH264Vbr->u32FrameRate;
    pstRcParam->stH264Vbr.bQpMapEnable = pstH264Vbr->bQpMapEnable;

    return 0;
}

int x3_venc_h264avbr(VENC_RC_ATTR_S *pstRcParam,
                    VENC_H264_AVBR_S *pstH264AVbr)
{
    pstRcParam->stH264AVbr.u32BitRate = pstH264AVbr->u32BitRate;
    pstRcParam->stH264AVbr.u32FrameRate = pstH264AVbr->u32FrameRate;
    pstRcParam->stH264AVbr.u32IntraPeriod = pstH264AVbr->u32IntraPeriod;
    pstRcParam->stH264AVbr.u32VbvBufferSize = pstH264AVbr->u32VbvBufferSize;
    pstRcParam->stH264AVbr.u32IntraQp = pstH264AVbr->u32IntraQp;
    pstRcParam->stH264AVbr.u32InitialRcQp = pstH264AVbr->u32InitialRcQp;
    pstRcParam->stH264AVbr.bMbLevelRcEnable = pstH264AVbr->bMbLevelRcEnable;
    pstRcParam->stH264AVbr.u32MaxIQp = pstH264AVbr->u32MaxIQp;
    pstRcParam->stH264AVbr.u32MinIQp = pstH264AVbr->u32MinIQp;
    pstRcParam->stH264AVbr.u32MaxPQp = pstH264AVbr->u32MaxPQp;
    pstRcParam->stH264AVbr.u32MinPQp = pstH264AVbr->u32MinPQp;
    pstRcParam->stH264AVbr.u32MaxBQp = pstH264AVbr->u32MaxBQp;
    pstRcParam->stH264AVbr.u32MinBQp = pstH264AVbr->u32MinBQp;
    pstRcParam->stH264AVbr.bHvsQpEnable = pstH264AVbr->bHvsQpEnable;
    pstRcParam->stH264AVbr.s32HvsQpScale = pstH264AVbr->s32HvsQpScale;
    pstRcParam->stH264AVbr.u32MaxDeltaQp = pstH264AVbr->u32MaxDeltaQp;
    pstRcParam->stH264AVbr.bQpMapEnable = pstH264AVbr->bQpMapEnable;

    return 0;
}

int x3_venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam,
                VENC_H264_FIXQP_S *pstH264FixQp)
{
    pstRcParam->stH264FixQp.u32FrameRate = pstH264FixQp->u32FrameRate;
    pstRcParam->stH264FixQp.u32IntraPeriod = pstH264FixQp->u32IntraPeriod;
    pstRcParam->stH264FixQp.u32IQp = pstH264FixQp->u32IQp;
    pstRcParam->stH264FixQp.u32PQp = pstH264FixQp->u32PQp;
    pstRcParam->stH264FixQp.u32BQp = pstH264FixQp->u32BQp;

    return 0;
}

int x3_venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam,
        VENC_H264_QPMAP_S *pstH264QpMap)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = pstH264QpMap->u32IntraPeriod;
    pstRcParam->stH264QpMap.u32FrameRate = pstH264QpMap->u32FrameRate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
                                        pstH264QpMap->u32QpMapArrayCount;
    pstRcParam->stH264QpMap.u32QpMapArray = pstH264QpMap->u32QpMapArray;
#if 0
	(unsigned char*)malloc((pstRcParam->stH264QpMap. \
		u32QpMapArrayCount)*sizeof(unsigned char));
		memset(pstRcParam->stH264QpMap.u32QpMapArray, 30,
			pstRcParam->stH264QpMap.u32QpMapArrayCount);
#endif

    return 0;
}

int x3_venc_h265cbr(VENC_RC_ATTR_S *pstRcParam, VENC_H265_CBR_S *pstH265Cbr)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = pstH265Cbr->u32IntraPeriod;
    pstRcParam->stH265Cbr.u32IntraQp = pstH265Cbr->u32IntraQp;
    pstRcParam->stH265Cbr.u32BitRate = pstH265Cbr->u32BitRate;
    pstRcParam->stH265Cbr.u32FrameRate = pstH265Cbr->u32FrameRate;
    pstRcParam->stH265Cbr.u32InitialRcQp = pstH265Cbr->u32InitialRcQp;
    pstRcParam->stH265Cbr.u32VbvBufferSize = pstH265Cbr->u32VbvBufferSize;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = pstH265Cbr->bCtuLevelRcEnable;
    pstRcParam->stH265Cbr.u32MaxIQp = pstH265Cbr->u32MaxIQp;
    pstRcParam->stH265Cbr.u32MinIQp = pstH265Cbr->u32MinIQp;
    pstRcParam->stH265Cbr.u32MaxPQp = pstH265Cbr->u32MaxPQp;
    pstRcParam->stH265Cbr.u32MinPQp = pstH265Cbr->u32MinPQp;
    pstRcParam->stH265Cbr.u32MaxBQp = pstH265Cbr->u32MaxBQp;
    pstRcParam->stH265Cbr.u32MinBQp = pstH265Cbr->u32MinBQp;
    pstRcParam->stH265Cbr.bHvsQpEnable = pstH265Cbr->bHvsQpEnable;
    pstRcParam->stH265Cbr.s32HvsQpScale = pstH265Cbr->s32HvsQpScale;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = pstH265Cbr->u32MaxDeltaQp;
    pstRcParam->stH265Cbr.bQpMapEnable = pstH265Cbr->bQpMapEnable;

    return 0;
}

int x3_venc_h265vbr(VENC_RC_ATTR_S *pstRcParam, VENC_H265_VBR_S *pstH265Vbr)
{
    pstRcParam->stH265Vbr.u32IntraPeriod = pstH265Vbr->u32IntraPeriod;
    pstRcParam->stH265Vbr.u32IntraQp = pstH265Vbr->u32IntraQp;
    pstRcParam->stH265Vbr.u32FrameRate = pstH265Vbr->u32FrameRate;
    pstRcParam->stH265Vbr.bQpMapEnable = pstH265Vbr->bQpMapEnable;

    // pstRcParam->stH265Vbr.u32IntraPeriod = intraperiod;
    // pstRcParam->stH265Vbr.u32IntraQp = intraqp;
    // pstRcParam->stH265Vbr.u32FrameRate = framerate;
    // pstRcParam->stH265Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int x3_venc_h265avbr(VENC_RC_ATTR_S *pstRcParam,
            VENC_H265_AVBR_S *pstH265AVbr)
{
    pstRcParam->stH265AVbr.u32IntraPeriod = pstH265AVbr->u32IntraPeriod;
    pstRcParam->stH265AVbr.u32IntraQp = pstH265AVbr->u32IntraQp;
    pstRcParam->stH265AVbr.u32BitRate = pstH265AVbr->u32BitRate;
    pstRcParam->stH265AVbr.u32FrameRate = pstH265AVbr->u32FrameRate;
    pstRcParam->stH265AVbr.u32InitialRcQp = pstH265AVbr->u32InitialRcQp;
    pstRcParam->stH265AVbr.u32VbvBufferSize = pstH265AVbr->u32VbvBufferSize;
    pstRcParam->stH265AVbr.bCtuLevelRcEnable = pstH265AVbr->bCtuLevelRcEnable;
    pstRcParam->stH265AVbr.u32MaxIQp = pstH265AVbr->u32MaxIQp;
    pstRcParam->stH265AVbr.u32MinIQp = pstH265AVbr->u32MinIQp;
    pstRcParam->stH265AVbr.u32MaxPQp = pstH265AVbr->u32MaxPQp;
    pstRcParam->stH265AVbr.u32MinPQp = pstH265AVbr->u32MinPQp;
    pstRcParam->stH265AVbr.u32MaxBQp = pstH265AVbr->u32MaxBQp;
    pstRcParam->stH265AVbr.u32MinBQp = pstH265AVbr->u32MinBQp;
    pstRcParam->stH265AVbr.bHvsQpEnable = pstH265AVbr->bHvsQpEnable;
    pstRcParam->stH265AVbr.s32HvsQpScale = pstH265AVbr->s32HvsQpScale;
    pstRcParam->stH265AVbr.u32MaxDeltaQp = pstH265AVbr->u32MaxDeltaQp;
    pstRcParam->stH265AVbr.bQpMapEnable = pstH265AVbr->bQpMapEnable;

    return 0;
}

int x3_venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam,
                VENC_H265_FIXQP_S *pstH265FixQp)
{
    pstRcParam->stH265FixQp.u32FrameRate = pstH265FixQp->u32FrameRate;
    pstRcParam->stH265FixQp.u32IntraPeriod = pstH265FixQp->u32IntraPeriod;
    pstRcParam->stH265FixQp.u32IQp = pstH265FixQp->u32IQp;
    pstRcParam->stH265FixQp.u32PQp = pstH265FixQp->u32PQp;
    pstRcParam->stH265FixQp.u32BQp = pstH265FixQp->u32BQp;

    return 0;
}

int x3_venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam,
                    VENC_H265_QPMAP_S *pstH265QpMap)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = pstH265QpMap->u32IntraPeriod;
    pstRcParam->stH264QpMap.u32FrameRate = pstH265QpMap->u32FrameRate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
                                pstH265QpMap->u32QpMapArrayCount;
    pstRcParam->stH264QpMap.u32QpMapArray =
                                pstH265QpMap->u32QpMapArray;

    return 0;
}

int x3_venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam,
            VENC_MJPEG_FIXQP_S *pstMjpegFixQp)
{
    pstRcParam->stMjpegFixQp.u32FrameRate = pstMjpegFixQp->u32FrameRate;
    pstRcParam->stMjpegFixQp.u32QualityFactort =
                                        pstMjpegFixQp->u32QualityFactort;

    return 0;
}

int x3_venc_setgop(VENC_GOP_ATTR_S *pstGopAttr, int presetidx,
                        int refreshtype, int qp, int intraqp)
{
    pstGopAttr->u32GopPresetIdx = presetidx;
    pstGopAttr->s32DecodingRefreshType = refreshtype;
    if (presetidx == 10) {
        // I-P2-P1-P2-P0-P2-P1-P2-P0
        pstGopAttr->stCustomGopParam.u32CustomGopSize = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32PocOffset = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32PocOffset = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32PocOffset = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL0 = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32PocOffset = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL1 = 0;
        pstGopAttr->u32GopPresetIdx = 0;
    } else if (presetidx == 11) {
        // I-P3-P2-P3-P1-P3-P2-P3-P0 -- P3-P2-P3-P1-P3-P2-P3-P0
        pstGopAttr->stCustomGopParam.u32CustomGopSize = 8;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32PocOffset = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32PocOffset = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureQp = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32PocOffset = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL0 = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32PocOffset = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureQp = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32TemporalId = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32PocOffset = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32RefPocL0 = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32PocOffset = 6;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32PictureQp = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32RefPocL0 = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32PocOffset = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32RefPocL0 = 6;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32PocOffset = 8;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32PictureQp = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32RefPocL1 = 0;
        pstGopAttr->u32GopPresetIdx = 0;
    }
	return 0;
}

uint32_t x3_venc_get_framerate(VENC_CHN_ATTR_S *vencChnAttr)
{
	if (vencChnAttr->stVencAttr.enType == PT_H264){
		switch (vencChnAttr->stRcAttr.enRcMode) {
			case VENC_RC_MODE_H264CBR:
			{
				return vencChnAttr->stRcAttr.stH264Cbr.u32FrameRate;
			}
			case VENC_RC_MODE_H264AVBR:
			{
				return vencChnAttr->stRcAttr.stH264AVbr.u32FrameRate;
			}
			case VENC_RC_MODE_H264QPMAP:
			{
				return vencChnAttr->stRcAttr.stH264QpMap.u32FrameRate;
			}
			default:
				return 0;
		}
	}

	return 0;
}

uint32_t x3_venc_get_bitrate(VENC_CHN_ATTR_S *vencChnAttr)
{
	if (vencChnAttr->stVencAttr.enType == PT_H264){
		switch (vencChnAttr->stRcAttr.enRcMode) {
			case VENC_RC_MODE_H264CBR:
			{
				return vencChnAttr->stRcAttr.stH264Cbr.u32BitRate;
			}
			case VENC_RC_MODE_H264AVBR:
			{
				return vencChnAttr->stRcAttr.stH264AVbr.u32BitRate;
			}
			default:
				return 0;
		}
	}

	return 0;
}

int x3_venc_set_bitrate(int VeChn, int bitrate)
{
	int ret = 0;
	VENC_RC_ATTR_S vencRcAttr;
	ROS_printf("bitrate = %d", bitrate);

    ret = HB_VENC_GetRcParam(VeChn, &vencRcAttr);
	if (ret) {
		ROS_printf("HB_VENC_GetRcParam failed: %d", ret);
		return ret;
	}
	switch (vencRcAttr.enRcMode) {
		case VENC_RC_MODE_H264CBR:
		{
			vencRcAttr.stH264Cbr.u32BitRate = bitrate;
			break;
		}
		case VENC_RC_MODE_H264AVBR:
		{
			vencRcAttr.stH264AVbr.u32BitRate = bitrate;
			break;
		}
		default:
			break;
	}
    ret = HB_VENC_SetRcParam(VeChn, &vencRcAttr);
	if (ret) {
		ROS_printf("HB_VENC_SetRcParam failed: %d", ret);
	}
    return ret;
}

uint32_t x3_venc_get_gop(VENC_CHN_ATTR_S *vencChnAttr)
{
	if (vencChnAttr->stVencAttr.enType == PT_H264){
		switch (vencChnAttr->stRcAttr.enRcMode) {
			case VENC_RC_MODE_H264CBR:
			{
				return vencChnAttr->stRcAttr.stH264Cbr.u32IntraPeriod;
			}
			case VENC_RC_MODE_H264AVBR:
			{
				return vencChnAttr->stRcAttr.stH264AVbr.u32IntraPeriod;
			}
			default:
				return 0;
		}
	}

	return 0;
}




/******************************************************************************
* funciton : get file postfix according palyload_type.
******************************************************************************/
uint32_t x3_venc_get_file_postfix(PAYLOAD_TYPE_E enPayload, char *szFilePostfix)
{
    if (PT_H264 == enPayload)
    {
        strcpy(szFilePostfix, ".h264");
    }
    else if (PT_H265 == enPayload)
    {
        strcpy(szFilePostfix, ".h265");
    }
    else if (PT_JPEG == enPayload)
    {
        strcpy(szFilePostfix, ".jpg");
    }
    else if (PT_MJPEG == enPayload)
    {
        strcpy(szFilePostfix, ".mjp");
    }
    else if (PT_MP4VIDEO == enPayload)
    {
        strcpy(szFilePostfix, ".mp4");
    }
    else
    {
        ROS_printf("payload type err!\n");
        return -1;
    }
    return 0;
}

/******************************************************************************
* funciton : save H264 stream
******************************************************************************/
uint32_t x3_venc_save_h264(FILE* fpH264File, VIDEO_STREAM_S *pstStream)
{
	fwrite(pstStream->pstPack.vir_ptr,
						pstStream->pstPack.size, 1, fpH264File);
	fflush(fpH264File);

    return 0;
}

/******************************************************************************
* funciton : save stream
******************************************************************************/
uint32_t x3_venc_save_stream(PAYLOAD_TYPE_E enType,FILE *pFd, VIDEO_STREAM_S *pstStream)
{
    uint32_t s32Ret;

    if (PT_H264 == enType)
    {
        s32Ret = x3_venc_save_h264(pFd, pstStream);
    }
    else if (PT_MJPEG == enType)
    {
        return -1;
    }
    else if (PT_H265 == enType)
    {
        return -1;
    }
    else
    {
        return -1;
    }
    return s32Ret;
}

int x3_venc_get_chn_attr(int Vechn, VENC_CHN_ATTR_S *vencChnAttr)
{
	int s32Ret = 0;
	s32Ret = HB_VENC_GetChnAttr(Vechn, vencChnAttr);
    if(s32Ret)
    {
        /*ROS_printf("HB_VENC_GetChnAttr chn[%d] failed with %d!\n", \*/
               /*Vechn, s32Ret);*/
        return s32Ret;
    }
	return s32Ret;
}
