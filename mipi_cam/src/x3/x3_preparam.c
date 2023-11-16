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

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include "utils/utils_log.h"

#include "sensor_f37_config.h"
//#include "sensor_os8a10_config.h"
#include "sensor_imx415_config.h"
//#include "sensor_ov8856_config.h"
//#include "sensor_sc031gs_config.h"
#include "sensor_imx586_config.h"
#include "sensor_gc4c33_config.h"

// sunrise camare 封装的头文件
#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
//#include "x3_vio_rgn.h"
#include "x3_utils.h"
//#include "x3_bpu.h"
//#include "x3_config.h"

/******************************* F37 方案 **********************************/
int f37_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_F37_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_F37_LINEAR_BASE;
	vin_info->ldcinfo = LDC_ATTR_F37_LINEAR_BASE;
	vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}

int f37_dol2_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_DOL2_INFO;
	vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_DOL2_ATTR;
	vin_info->devinfo = DEV_ATTR_F37_DOL2_BASE;
	vin_info->pipeinfo = PIPE_ATTR_F37_DOL2_BASE;
	vin_info->disinfo = DIS_ATTR_F37_DOL2_BASE;
	vin_info->ldcinfo = LDC_ATTR_F37_DOL2_BASE;
	vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}
/******************************** OS8a10 4K 方案 ******************************/
/*
int os8a10_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_OS8A10_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_OS8A10_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_OS8A10_BASE;
	vin_info->ldcinfo = LDC_ATTR_OS8A10_BASE;
	vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

	// 单目 dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}

int os8a10_dol2_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO;
	vin_info->mipi_attr = MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR;
	vin_info->devinfo = DEV_ATTR_OS8A10_DOL2_BASE;
	vin_info->pipeinfo = PIPE_ATTR_OS8A10_DOL2_BASE;
	vin_info->disinfo = DIS_ATTR_OS8A10_BASE;
	vin_info->ldcinfo = LDC_ATTR_OS8A10_BASE;
	vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

	// 单目 dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}

int os8a10_2k_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_OS8A10_2K_25FPS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_SENSOR_OS8A10_2K_25FPS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_OS8A10_2K_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_OS8A10_2K_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_OS8A10_2K_BASE;
	vin_info->ldcinfo = LDC_ATTR_OS8A10_2K_BASE;
	vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}
*/
/******************************** IMX415 4K 方案 ******************************/
int imx415_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_4LANE_IMX415_30FPS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_4LANE_SENSOR_IMX415_30FPS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_IMX415_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_IMX415_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_IMX415_BASE;
	vin_info->ldcinfo = LDC_ATTR_IMX415_BASE;
	vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}
/******************************* IMX586 方案 **********************************/
int imx586_linear_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_IMX586_25FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_SENSOR_IMX586_25FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_IMX586_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_IMX586_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_IMX586_BASE;
    vin_info->ldcinfo = LDC_ATTR_IMX586_BASE;
    vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = 0;
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}


/******************************* GC4C33 方案 **********************************/
int gc4c33_linear_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_GC4C33_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_SENSOR_GC4C33_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_GC4C33_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_GC4C33_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_GC4C33_BASE;
    vin_info->ldcinfo = LDC_ATTR_GC4C33_BASE;
    vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = 0;
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

/******************************** OV8856 ******************************/
/*int ov8856_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_2LANE_OV8856_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_2LANE_OV8856_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_2LANE_OV8856_10BIT_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_2LANE_OV8856_10BIT_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_OV8856_BASE;
	vin_info->ldcinfo = LDC_ATTR_OV8856_BASE;
	//vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;
	vin_info->vin_vps_mode = VIN_ONLINE_VPS_ONLINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}
*/
/******************************** SC031GS ******************************/
/*int sc031gs_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_2LANE_SC031GS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_2LANE_SC031GS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_2LANE_SC031GS_10BIT_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_2LANE_SC031GS_10BIT_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_SC031GS_BASE;
	vin_info->ldcinfo = LDC_ATTR_SC031GS_BASE;
	//vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;
	vin_info->vin_vps_mode = VIN_ONLINE_VPS_ONLINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
	vin_info->enable_dev_attr_ex = 0;

	return 0;
}*/
// vps的输入参数
int vps_grp_param_init(x3_vps_info_t *vps_info, int width, int height)
{
	// 默认group id 0
	vps_info->m_vps_grp_id = 0;
	vps_info->m_vps_grp_attr.maxW = width;
	vps_info->m_vps_grp_attr.maxH = height;
	/* vps grp的framedepth是gdc的buf深度，如果只用groupRotate，配成1就行 */
	vps_info->m_vps_grp_attr.frameDepth = 8;// 原先是 1 
	/*.pixelFormat 像素格式只支持nv12 */
	/*vps_info->m_vps_grp_attr.pixelFormat = HB_PIXEL_FORMAT_NV12;*/

	return 0;
}

// vps的通道参数，为什么要grp和chn分开两个调用，目的是为了更加灵活的配置一个group的输出
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr, int chn_id, int width, int height, int fps)
{
	vps_chn_attr->m_chn_id = chn_id; // vps 通道，只有chn2 和 chn5 支持4K输出
	vps_chn_attr->m_chn_enable = 1;
	vps_chn_attr->m_chn_attr.width = width;
	vps_chn_attr->m_chn_attr.height = height;
	
	vps_chn_attr->m_chn_attr.enMirror = 0;
	vps_chn_attr->m_chn_attr.enFlip = 0;
	vps_chn_attr->m_chn_attr.enScale = 1;
	vps_chn_attr->m_chn_attr.frameDepth = 8;//1;/* 图像队列长度, 最大32 */
	vps_chn_attr->m_chn_attr.frameRate.srcFrameRate = fps;
	vps_chn_attr->m_chn_attr.frameRate.dstFrameRate = fps;

	return 0;
}

int venc_chn_param_init(x3_venc_chn_info_t *venc_chn_info, int chn_id, int width, int height, int fps, int bitrate)
{
	VENC_CHN_ATTR_S *venc_chn_attr = &venc_chn_info->m_chn_attr;

	venc_chn_attr->stVencAttr.enType = PT_H264;
	venc_chn_attr->stVencAttr.u32PicWidth = width;
	venc_chn_attr->stVencAttr.u32PicHeight = height;

	venc_chn_attr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
	venc_chn_attr->stVencAttr.enRotation = CODEC_ROTATION_0;
	venc_chn_attr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
	venc_chn_attr->stVencAttr.bEnableUserPts = HB_TRUE;
	venc_chn_attr->stVencAttr.s32BufJoint = 0;
	venc_chn_attr->stVencAttr.s32BufJointSize = 8000000;

	venc_chn_attr->stVencAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
	venc_chn_attr->stVencAttr.u32BitStreamBufferCount = 3;
	venc_chn_attr->stVencAttr.u32FrameBufferCount = 3;
	venc_chn_attr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
	if (width * height > 2688 * 1522) {
        venc_chn_attr->stVencAttr.vlc_buf_size = 8*1024*1024;
    } else if (width * height > 1920 * 1080) {
        venc_chn_attr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (width * height > 1280 * 720) {
        venc_chn_attr->stVencAttr.vlc_buf_size = 2100*1024;
    } else if (width * height > 704 * 576) {
       venc_chn_attr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        venc_chn_attr->stVencAttr.vlc_buf_size = 2048*1024;
    }
	venc_chn_attr->stVencAttr.u32BitStreamBufSize = (width*height)&0xfffff000;

	venc_chn_attr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
	venc_chn_attr->stRcAttr.stH264Cbr.u32BitRate = bitrate;
	venc_chn_attr->stRcAttr.stH264Cbr.u32FrameRate = fps;
	venc_chn_attr->stRcAttr.stH264Cbr.u32IntraPeriod = 30;
	venc_chn_attr->stRcAttr.stH264Cbr.u32VbvBufferSize = 3000;
	venc_chn_attr->stRcAttr.stH264Cbr.u32IntraQp = 30;
	venc_chn_attr->stRcAttr.stH264Cbr.u32InitialRcQp = 30;
	venc_chn_attr->stRcAttr.stH264Cbr.bMbLevelRcEnable = HB_FALSE;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MaxIQp = 51;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MinIQp = 10;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MaxPQp = 51;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MinPQp = 10;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MaxBQp = 51;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MinBQp = 10;
	venc_chn_attr->stRcAttr.stH264Cbr.bHvsQpEnable = HB_FALSE;
	venc_chn_attr->stRcAttr.stH264Cbr.s32HvsQpScale = 2;
	venc_chn_attr->stRcAttr.stH264Cbr.u32MaxDeltaQp = 3;
	venc_chn_attr->stRcAttr.stH264Cbr.bQpMapEnable = HB_FALSE;

	venc_chn_attr->stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_UNSPECIFIED;
	venc_chn_attr->stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL_UNSPECIFIED;

	venc_chn_attr->stGopAttr.u32GopPresetIdx = 6;
	venc_chn_attr->stGopAttr.s32DecodingRefreshType = 2;

	// 这里默认用 vps group 0 做为输出，如不同需要修改掉
	venc_chn_info->m_vps_grp_id = 0;
	venc_chn_info->m_vps_chn_id = 2; /* 只有chn2 和 chn5 支持4K输出 */
	venc_chn_info->m_is_bind = 1;
	venc_chn_info->m_venc_chn_id = chn_id; // 编码通道
	venc_chn_info->m_chn_enable = 1;
	venc_chn_info->m_enable_transform = 1;
	venc_chn_info->m_is_save_to_file = 0;

	return 0;
}

int vot_param_init(x3_vot_info_t *vot_info)
{
	// devAttr
	vot_info->m_devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
	vot_info->m_devAttr.u32BgColor = 0x108080;
	vot_info->m_devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;

	// layerAttr
	vot_info->m_stLayerAttr.stImageSize.u32Width = 1920;
    vot_info->m_stLayerAttr.stImageSize.u32Height = 1080;
	vot_info->m_stLayerAttr.big_endian = 0;
    vot_info->m_stLayerAttr.display_addr_type = 2;
    vot_info->m_stLayerAttr.display_addr_type_layer1 = 2;

	vot_info->m_stLayerAttr.dithering_flag = 0;
    vot_info->m_stLayerAttr.dithering_en = 0;
    vot_info->m_stLayerAttr.gamma_en = 0;
    vot_info->m_stLayerAttr.hue_en = 0;
    vot_info->m_stLayerAttr.sat_en = 0;
    vot_info->m_stLayerAttr.con_en = 0;
    vot_info->m_stLayerAttr.bright_en = 0;
    vot_info->m_stLayerAttr.theta_sign = 0;
    vot_info->m_stLayerAttr.contrast = 0;

    vot_info->m_stLayerAttr.theta_abs = 0;
    vot_info->m_stLayerAttr.saturation = 0;
    vot_info->m_stLayerAttr.off_contrast = 0;
    vot_info->m_stLayerAttr.off_bright = 0;
	
    vot_info->m_stLayerAttr.panel_type = 0;
    vot_info->m_stLayerAttr.rotate = 0;
    vot_info->m_stLayerAttr.user_control_disp = 0;

	// ChnAttr
	vot_info->m_stChnAttr.u32Priority = 2;
    vot_info->m_stChnAttr.s32X = 0;
    vot_info->m_stChnAttr.s32Y = 0;
    vot_info->m_stChnAttr.u32SrcWidth = 1920;
    vot_info->m_stChnAttr.u32SrcHeight = 1080;
    vot_info->m_stChnAttr.u32DstWidth = 1920;
    vot_info->m_stChnAttr.u32DstHeight = 1080;

	// cropAttr
	vot_info->m_cropAttrs.u32Width = vot_info->m_stChnAttr.u32DstWidth;
    vot_info->m_cropAttrs.u32Height = vot_info->m_stChnAttr.u32DstHeight;
	return 0;
}

// 只解码H264
int vdec_chn_param_init(x3_vdec_chn_info_t *vdec_chn_info, int chn_id, int width, int height, char *stream_src)
{
	VDEC_CHN_ATTR_S *vdec_chn_attr = &vdec_chn_info->m_chn_attr;

	vdec_chn_attr->enType = PT_H264;
    vdec_chn_attr->enMode = VIDEO_MODE_FRAME;
    vdec_chn_attr->enPixelFormat = HB_PIXEL_FORMAT_NV12;
    vdec_chn_attr->u32FrameBufCnt = 3;
    vdec_chn_attr->u32StreamBufCnt = 3;
    vdec_chn_attr->u32StreamBufSize = width * height * 3 / 2;
    vdec_chn_attr->bExternalBitStreamBuf  = HB_TRUE;
    vdec_chn_attr->stAttrH264.bandwidth_Opt = HB_TRUE;
    vdec_chn_attr->stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
    vdec_chn_attr->stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;

	sprintf(vdec_chn_info->m_stream_src, stream_src);
	vdec_chn_info->m_vdec_chn_id = chn_id;
	vdec_chn_info->m_chn_enable = 1;
	
	vdec_chn_info->vp_param.mmz_cnt = 5; // 根据vp_param结构体的定义，最大支持32个
	vdec_chn_info->vp_param.mmz_size = width * height * 3 / 2;
	return 0;
}

// rgn的配置建议和vps group和通道放在一块设置，这样每个group的每个通道都能单独设置自己的osd
// 此处使用osd仅用来演示osd接口功能和基本的接口调用流程
/*
int x3_rgn_timestamp_param_init(x3_rgn_info_t *rgn_info, int vps_grp_id, int vps_chn_id)
{
	rgn_info->m_rgn_handle = vps_grp_id;
	rgn_info->m_rgn_chn.s32PipelineId = vps_grp_id; // 这个应该是vps的group号
	// 把osd的句柄对应到相应的通道上去
	rgn_info->m_rgn_chn.enChnId = CHN_DS2; // chn2

	// ipu chn 和 硬件通道的对应关系如下
	//
	// chn0 -> CHN_DS0
	// chn1 -> CHN_DS1
	// chn2 -> CHN_DS2
	// chn3 -> CHN_DS3
	// chn4 -> CHN_DS4
	// chn5 -> CHN_US
	//

	rgn_info->m_rgn_attr.enType = OVERLAY_RGN;
    rgn_info->m_rgn_attr.stOverlayAttr.stSize.u32Width = 480;
    rgn_info->m_rgn_attr.stOverlayAttr.stSize.u32Height = 100;
    rgn_info->m_rgn_attr.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;
    rgn_info->m_rgn_attr.stOverlayAttr.enBgColor  = 16;

    rgn_info->m_rgn_chn_attr.bShow = true;
    rgn_info->m_rgn_chn_attr.bInvertEn = false;
    // rgn_info->m_chn_attr.enType = OVERLAY_RGN;
    rgn_info->m_rgn_chn_attr.unChnAttr.stOverlayChn.stPoint.u32X = 50;
    rgn_info->m_rgn_chn_attr.unChnAttr.stOverlayChn.stPoint.u32Y = 50;
    // rgn_info->m_chn_attr.unChnAttr.stOverlayChn.u32Layer = 3;

	return 0;
}
// 叠加OSD的线程
// 支持时间、文字、画线、位图
void* x3_rgn_set_timestamp_thread(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
    int ret = 0;
    time_t tt,tt_last=0;
	RGN_BITMAP_S bitmapAttr;
    RGN_DRAW_WORD_S drawWord;
	RGN_ATTR_S pstRegion;
    RGN_HANDLE Handle = *(RGN_HANDLE*)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	// 获取区域配置
	LOGI_print("Handle: %d", Handle);
	ret = HB_RGN_GetAttr(Handle, &pstRegion);

	bitmapAttr.enPixelFormat = PIXEL_FORMAT_VGA_4;
    bitmapAttr.stSize = pstRegion.stOverlayAttr.stSize;
    bitmapAttr.pAddr = malloc(bitmapAttr.stSize.u32Width * bitmapAttr.stSize.u32Height / 2);
    memset(bitmapAttr.pAddr, 0xff, bitmapAttr.stSize.u32Width * bitmapAttr.stSize.u32Height / 2);

	// drawWord.bInvertEn = 1;
    drawWord.enFontSize = FONT_SIZE_LARGE;
    drawWord.enFontColor = FONT_COLOR_WHITE;
    drawWord.stPoint.u32X = 0;
    drawWord.stPoint.u32Y = 0;
    drawWord.bFlushEn = false;
    // unsigned char str[10] = {0xce, 0xd2, 0xce, 0xd2,
    //		 0xce, 0xd2, 0xce, 0xd2, 0xce, 0xd2};

    uint8_t str[32] = {0xb5, 0xd8, 0xb5, 0xe3, 0xca, 0xbe, 0xc0, 0xfd};
    drawWord.pu8Str = str;
    drawWord.pAddr = bitmapAttr.pAddr;
    drawWord.stSize = bitmapAttr.stSize;

    while(privThread->eState == E_THREAD_RUNNING) {
        tt = time(0);
        if (tt>tt_last) {
            char str[32];
            strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&tt));
            drawWord.pu8Str = (uint8_t *)str;
			//ROS_printf("Handle:%d str: %s\n", Handle, str);
			// 把文件转换成位图
            ret = HB_RGN_DrawWord(Handle, &drawWord);
            if (ret) {
                ROS_printf("HB_RGN_DrawWord failed\n");
				usleep(100000);
                continue;
            }
			// 把位图叠加到图像数据上
			ret = HB_RGN_SetBitMap(Handle, &bitmapAttr);
            if (ret) {
                ROS_printf("HB_RGN_SetBitMap failed\n");
                usleep(100000);
                continue;
            }
            tt_last = tt;
        }
        usleep(100000); // 每秒设置一次
    }
	mThreadFinish(privThread);
    return NULL;
}
*/

