#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>

#include "vio/hb_rgn.h"

int x3_rgn_init(RGN_HANDLE Handle, RGN_CHN_S *chn, RGN_ATTR_S *pstRegion, RGN_CHN_ATTR_S *pstChnAttr)
{
	int ret = 0;
	// 创建区域
	ret = HB_RGN_Create(Handle, pstRegion);
    if (ret) {
        ROS_printf("HB_RGN_Create failed\n");
        return ret;
    }

	// 把该区域附加通道上
	// 通道结构体的pipeid、chnid是什么意思？和vin、isp的pipeid和chnid是什么关系？
	ret = HB_RGN_AttachToChn(Handle, chn, pstChnAttr);
    if (ret) {
        ROS_printf("HB_RGN_AttachToChn failed\n");
        return ret;
    }
	return ret;
}

int x3_rgn_uninit(RGN_HANDLE Handle, RGN_CHN_S *chn)
{
	int ret = 0;
	ret = HB_RGN_DetachFromChn(Handle, chn);
    if (ret) {
        ROS_printf("HB_RGN_DetachFromChn failed\n");
        return ret;
    }

	ret = HB_RGN_Destroy(Handle);
    if (ret) {
        ROS_printf("HB_RGN_Destroy failed\n");
        return ret;
    }
	ROS_printf("[%s][%d] ok!\n", __func__, __LINE__);
	return ret;
}
