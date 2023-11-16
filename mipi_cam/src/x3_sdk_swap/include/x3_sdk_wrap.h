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

#ifndef X3_SDK_WRAP_H_
#define X3_SDK_WRAP_H_

// SDK 提供的接口都经过这里封装好向上提供
/*#include "vio/hb_vin_api.h"
#include "vio/hb_mipi_api.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_comm_venc.h"
#include "vio/hb_comm_vdec.h"
#include "vio/hb_common_vot.h"
#include "vio/hb_vps_api.h"
#include "vio/hb_rgn.h"*/
#include "vio/hb_vin_api.h"
#include "vio/hb_mipi_api.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_comm_venc.h"
#include "vio/hb_comm_vdec.h"
#include "vio/hb_common_vot.h"
#include "vio/hb_vps_api.h"
#include "vio/hb_rgn.h"

//#include "x3_bpu.h"
#define LOGE_print ROS_printf
#define LOGI_print ROS_printf
#define LOGD_print ROS_printf

// 单个通道的属性配置
typedef struct {
	// 输入数据来源于哪个grp.chn
	int m_vps_grp_id;
	int m_vps_chn_id;
	// vps chn到venc的数据是否bind，如果bind，vps chn输出的数据自动发送给编码器
	int m_is_bind;
	// 编码器属性，内部通过union支持H264、H265、Mjpeg、Jpeg
	int m_venc_chn_id;		 /* 编码通道 */
	int m_chn_enable;
	int m_enable_transform; // 是否启用改善编码呼吸效应的配置参数
	VENC_CHN_ATTR_S m_chn_attr;

	// 调试字段，把编码后的的h264、h265写到文件中
	int m_is_save_to_file;
	int m_file_name[128];
} x3_venc_chn_info_t;

// 定义一组编码器，h264、h265最大32路编码器，jpeg、mjpeg 支持64路
typedef struct {
	x3_venc_chn_info_t m_venc_chn_info[32];
	int m_chn_num; // 使能多少个通道
} x3_venc_info_t;


typedef struct {
	RGN_HANDLE m_rgn_handle;
	RGN_CHN_S m_rgn_chn;
    RGN_ATTR_S m_rgn_attr;
	RGN_CHN_ATTR_S m_rgn_chn_attr;
} x3_rgn_info_t;
/*
typedef struct {
	int m_alog_id;
	int m_pipeline_id;	// 当前算法是给那一路数据用的
	bpu_server_t *m_bpu_handle;; // 用来传给线程，帮助定位是用的哪个算法任务
} x3_bpu_info_t;
*/
typedef struct {
	/* gdc 配置文件 */
	char m_gdc_config[256];
	ROTATION_E m_rotation;
} x3_gdc_info_t;

// 以下两个结构体用来描述模块使用的内存buff信息
typedef struct vp_param {
    uint64_t mmz_paddr[32];
    char *mmz_vaddr[32];
    int mmz_cnt;
    int mmz_size;
} vp_param_t;

typedef struct av_param {
    int count;
    int videoIndex;
    int bufSize;
    int firstPacket;
} av_param_t;


// 单个解码通道的属性配置
typedef struct {
	// 解码数据源支持：
	// 1、 h264码流文件（已支持）
	// 2、 rtsp h264 码流 （需要重新编译ffmpeg库支持）（暂不支持）
	char m_stream_src[128];
	// 解码器属性，内部通过union支持H264、H265、Mjpeg、Jpeg
	int m_vdec_chn_id;		 /* 编码通道 */
	int m_chn_enable;
	VDEC_CHN_ATTR_S m_chn_attr;
	
	vp_param_t vp_param;
    av_param_t av_param;
} x3_vdec_chn_info_t;

// 定义一组解码器，h264、h265最大32路编码器，jpeg、mjpeg 支持64路
typedef struct {
	x3_vdec_chn_info_t m_vdec_chn_info[8]; // 这是个数组
	int m_chn_num; // 使能多少个通道
} 
x3_vdec_info_t;

// 定义x3输出通道的配置结构体
typedef struct {
	VOT_VIDEO_LAYER_ATTR_S m_stLayerAttr;
    VOT_CHN_ATTR_S m_stChnAttr;
    VOT_WB_ATTR_S m_stWbAttr;
    VOT_CROP_INFO_S m_cropAttrs;
    VOT_PUB_ATTR_S m_devAttr;
} x3_vot_info_t;

// 一个vps 通道的属性配置
typedef struct{
	int m_chn_id; /* vps 输出通道 */
	int m_chn_enable; /* 是否使能通道 */
	/* 通道输出尺寸限制请查阅《X3J3平台AIOT媒体系统接口手册.pdf》5.3 功能描述 */
	VPS_CHN_ATTR_S m_chn_attr; /* 通道配置 */
} x3_vps_chn_attr_t;

// 一个ipu group的配置，包含一个输入配置和7个输出通道
typedef struct {
	/* VPS 输入配置*/
	int m_vps_grp_id;
	VPS_GRP_ATTR_S 	m_vps_grp_attr;
	/* 以下是vps输出通道配置，最多支持7个通道，第7个通道需要从通道2 online给到pym */
	int m_chn_num; // 使能几个通道
	x3_vps_chn_attr_t m_vps_chn_attrs[7];
	int m_need_gdc;
  x3_gdc_info_t m_gdc_info;
} x3_vps_info_t;

// 多个VPS group的配置
typedef struct {
	/* VPS group 数量*/
	int m_group_num;
	x3_vps_info_t m_vps_info[8];
} x3_vps_infos_t;

typedef struct x3_vin_info {
     /*定义 sensor   初始化的属性信息 */
    MIPI_SENSOR_INFO_S snsinfo;
     /*定义 mipi 初始化参数信息 */
    MIPI_ATTR_S mipi_attr;
     /*定义 dev 初始化的属性信息 */
    VIN_DEV_ATTR_S devinfo;
     /*定义 pipe 属性信息 */
    VIN_PIPE_ATTR_S pipeinfo;
     /*定义 DIS(gdc) 属性信息 */
    VIN_DIS_ATTR_S disinfo;
     /*定义 LDC 属性信息 */
    VIN_LDC_ATTR_S ldcinfo;

    int dev_id;       /* 通路索引，范围 0~7 */
    int pipe_id;      /* PipeLine 号， 对应每路输入，范围 0~7 */

    /* 配置vin到vps是online还是offline模式 */
    SYS_VIN_VPS_MODE_E vin_vps_mode;

	/* dev扩展属性 */
	int enable_dev_attr_ex;
    VIN_DEV_ATTR_EX_S devexinfo;
} x3_vin_info_t;

typedef struct {
	int m_chn_num;
	VENC_CHN m_enable_chn_idx[VENC_MAX_CHN_NUM];
}x3_venc_en_chns_info_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void print_debug_infos(void);
extern int ROS_printf(char *fmt, ...);

int x3_venc_get_en_chn_info_wrap(x3_venc_info_t* venc_info, x3_venc_en_chns_info_t *venc_en_chns_info);

int x3_venc_init_wrap(x3_venc_info_t* venc_info);
int x3_venc_uninit_wrap(x3_venc_info_t* venc_info);
int x3_vps_init_wrap(x3_vps_info_t *vps_info);
void x3_vps_uninit_wrap(x3_vps_info_t *vps_info);
int x3_vdec_init_wrap(x3_vdec_info_t* vdec_info);
int x3_vdec_uninit_wrap(x3_vdec_info_t* vdec_info);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_SDK_WRAP_H_

