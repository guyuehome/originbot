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

//#include "utils/utils_log.h"

#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_venc.h"
#include "x3_vio_vp.h"
#include "x3_vio_vdec.h"
#include "x3_sdk_wrap.h"

// 打印 vin isp vpu venc等模块的调试信息
void print_debug_infos(void)
{
	ROS_printf("========================= SIF ==========================\n");
	system("cat /sys/devices/platform/soc/a4001000.sif/cfg_info");
	ROS_printf("========================= ISP ==========================\n");
	system("cat /sys/devices/platform/soc/b3000000.isp/isp_status");
	ROS_printf("========================= IPU PIPE enable ==============\n");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/enabled_pipeline");
	ROS_printf("========================= IPU PIPE config ==============\n");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info");
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline1_info");
	ROS_printf("========================= VENC =========================\n");
	system("cat /sys/kernel/debug/vpu/venc");
	ROS_printf("========================= VDEC =========================\n");
	system("cat /sys/kernel/debug/vpu/vdec");
	ROS_printf("========================= JENC =========================\n");
	system("cat /sys/kernel/debug/jpu/jenc");
	ROS_printf("========================= IAR ==========================\n");
	system("cat /sys/kernel/debug/iar");
	ROS_printf("========================= ION ==========================\n");
	system("cat /sys/kernel/debug/ion/heaps/carveout");
	system("cat /sys/kernel/debug/ion/heaps/ion_cma");
	ROS_printf("========================= END ===========================\n");
}


int x3_venc_get_en_chn_info_wrap(x3_venc_info_t* venc_info,
    x3_venc_en_chns_info_t *venc_en_chns_info)
{
    int i = 0;
    for (i = 0; i < venc_info->m_chn_num; i++){
        if (venc_info->m_venc_chn_info[i].m_chn_enable) {
            venc_en_chns_info->m_chn_num++;
            venc_en_chns_info->m_enable_chn_idx[i] =
                venc_info->m_venc_chn_info[i].m_venc_chn_id;
        }
    }
    return 0;
}

int x3_venc_init_wrap(x3_venc_info_t* venc_info)
{
	int ret = 0;
	int i = 0;
	for (i = 0; i < venc_info->m_chn_num; i++){
		ret = x3_venc_init(venc_info->m_venc_chn_info[i].m_venc_chn_id,
			&venc_info->m_venc_chn_info[i].m_chn_attr);
		if (ret) {
			ROS_printf("x3_venc_init failed, %d\n", ret);
			return -1;
		}
	}
	LOGI_print("ok!");
	return 0;
}

int x3_venc_uninit_wrap(x3_venc_info_t* venc_info)
{
	int ret = 0;
	int i = 0;
	for (i = 0; i < venc_info->m_chn_num; i++){
		ret = x3_venc_deinit(venc_info->m_venc_chn_info[i].m_venc_chn_id);
		if (ret) {
			ROS_printf("x3_venc_deinit chn%d failed, %d\n",
				venc_info->m_venc_chn_info[i].m_venc_chn_id, ret);
			return -1;
		}
	}
	return 0;
}


/* 默认只使能一个输出通道，需要多输出通道的场景，另外再调用x3_vps_init_chn初始化和使能 */
int x3_vps_init_wrap(x3_vps_info_t *vps_info)
{
	int ret = 0;
	int i = 0;
	// 创建group
	ret = x3_vps_group_init(vps_info->m_vps_grp_id, &vps_info->m_vps_grp_attr);
	if (ret) return ret;
	// 初始化gdc
	if (vps_info->m_need_gdc) {
		ret = x3_setpu_gdc(vps_info->m_vps_grp_id, vps_info->m_vps_chn_attrs->m_chn_id,
		        vps_info->m_gdc_info.m_gdc_config, vps_info->m_gdc_info.m_rotation);
		if (ret) {
			HB_VPS_DestroyGrp(vps_info->m_vps_grp_id);
			return ret;
		}
	}
	// 初始化配置的vps channal
	for (i = 0; i < vps_info->m_chn_num; i++){
		if (vps_info->m_vps_chn_attrs[i].m_chn_enable) {
			ret = x3_vps_chn_init(vps_info->m_vps_grp_id, vps_info->m_vps_chn_attrs[i].m_chn_id,
				&vps_info->m_vps_chn_attrs[i].m_chn_attr, vps_info->m_gdc_info.m_rotation);
			LOGI_print("[%s]->vps chn%d/%d init ret=%d.\n",
				__func__, vps_info->m_vps_chn_attrs[i].m_chn_id, vps_info->m_chn_num,ret);
			if (ret) {
				HB_VPS_DestroyGrp(vps_info->m_vps_grp_id);
				return ret;
			}
		}
	}
	return ret;

}

void x3_vps_uninit_wrap(x3_vps_info_t *vps_info)
{
	x3_vps_deinit(vps_info->m_vps_grp_id);
}


int x3_vdec_init_wrap(x3_vdec_info_t* vdec_info)
{
	int ret = 0;
	int i = 0;
#ifdef SW_VDEC
	for (i = 0; i < vdec_info->m_chn_num; i++){
		// 创建内存buff
		x3_vp_alloc(&vdec_info->m_vdec_chn_info[i].vp_param);
		// 初始化解码器
		ret = x3_vdec_init(vdec_info->m_vdec_chn_info[i].m_vdec_chn_id,
			&vdec_info->m_vdec_chn_info[i].m_chn_attr);
		if (ret) {
			ROS_printf("x3_vdec_init failed, %d\n", ret);
			return -1;
		}
		ROS_printf("start vdec chn%d ok!\n", vdec_info->m_vdec_chn_info[i].m_vdec_chn_id);
	}
#endif
	return 0;
}

int x3_vdec_uninit_wrap(x3_vdec_info_t* vdec_info)
{
	int ret = 0;
	int i = 0;
#ifdef SW_VDEC
	for (i = 0; i < vdec_info->m_chn_num; i++){
		ret = x3_vdec_deinit(vdec_info->m_vdec_chn_info[i].m_vdec_chn_id);
		if (ret) {
			ROS_printf("x3_vdec_deinit failed, %d\n", ret);
			return -1;
		}
		// 释放内存buff
		x3_vp_free(&vdec_info->m_vdec_chn_info[i].vp_param);
	}
#endif
	return 0;
}

