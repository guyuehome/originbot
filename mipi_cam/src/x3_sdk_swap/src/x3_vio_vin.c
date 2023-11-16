/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <sys/stat.h>

//#include "utils/utils_log.h"
//#include "logging.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"

#define HW_TIMER 24000
#define MAX_PLANE 4

typedef struct {
	uint32_t frame_id;
	uint32_t plane_count;
	uint32_t xres[MAX_PLANE];
	uint32_t yres[MAX_PLANE];
	char *addr[MAX_PLANE];
	uint32_t size[MAX_PLANE];
} raw_t;

typedef struct {
	uint8_t ctx_id;
    raw_t raw;
} dump_info_t;

void print_sensor_dev_info(VIN_DEV_ATTR_S *devinfo)
{
	ROS_printf("devinfo->stSize.format %d\n", devinfo->stSize.format);
	ROS_printf("devinfo->stSize.height %d\n", devinfo->stSize.height);
	ROS_printf("devinfo->stSize.width %d\n", devinfo->stSize.width);
	ROS_printf("devinfo->stSize.pix_length %d\n", devinfo->stSize.pix_length);
	ROS_printf("devinfo->mipiAttr.enable_frame_id %d\n", devinfo->mipiAttr.enable_frame_id);
	ROS_printf("devinfo->mipiAttr.enable_mux_out %d\n", devinfo->mipiAttr.enable_mux_out);
	ROS_printf("devinfo->mipiAttr.set_init_frame_id %d\n", devinfo->mipiAttr.set_init_frame_id);
	ROS_printf("devinfo->mipiAttr.ipi_channels %d\n", devinfo->mipiAttr.ipi_channels);
	ROS_printf("devinfo->mipiAttr.enable_line_shift %d\n", devinfo->mipiAttr.enable_line_shift);
	ROS_printf("devinfo->mipiAttr.enable_id_decoder %d\n", devinfo->mipiAttr.enable_id_decoder);
	ROS_printf("devinfo->mipiAttr.set_bypass_channels %d\n", devinfo->mipiAttr.set_bypass_channels);
	ROS_printf("devinfo->mipiAttr.enable_bypass %d\n", devinfo->mipiAttr.enable_bypass);
	ROS_printf("devinfo->mipiAttr.set_line_shift_count %d\n", devinfo->mipiAttr.set_line_shift_count);
	ROS_printf("devinfo->mipiAttr.enable_pattern %d\n", devinfo->mipiAttr.enable_pattern);

	ROS_printf("devinfo->outDdrAttr.stride %d\n", devinfo->outDdrAttr.stride);
	ROS_printf("devinfo->outDdrAttr.buffer_num %d\n", devinfo->outDdrAttr.buffer_num);
	return;
}

void print_sensor_pipe_info(VIN_PIPE_ATTR_S *pipeinfo)
{
	ROS_printf("isp_out ddr_out_buf_num %d\n", pipeinfo->ddrOutBufNum);
	ROS_printf("isp_out width %d\n", pipeinfo->stSize.width);
	ROS_printf("isp_out height %d\n", pipeinfo->stSize.height);
	ROS_printf("isp_out sensor_mode %d\n", pipeinfo->snsMode);
	ROS_printf("isp_out format %d\n", pipeinfo->stSize.format);
	return;
}

void print_sensor_info(MIPI_SENSOR_INFO_S *snsinfo)
{
	ROS_printf("bus_num %d\n", snsinfo->sensorInfo.bus_num);
	ROS_printf("bus_type %d\n", snsinfo->sensorInfo.bus_type);
	ROS_printf("sensor_name %s\n", snsinfo->sensorInfo.sensor_name);
	ROS_printf("reg_width %d\n", snsinfo->sensorInfo.reg_width);
	ROS_printf("sensor_mode %d\n", snsinfo->sensorInfo.sensor_mode);
	ROS_printf("sensor_addr 0x%x\n", snsinfo->sensorInfo.sensor_addr);
	ROS_printf("serial_addr 0x%x\n", snsinfo->sensorInfo.serial_addr);
	ROS_printf("resolution %d\n", snsinfo->sensorInfo.resolution);
    return;
}

void x3_vio_buf_info_print(hb_vio_buffer_t * buf)
{
	ROS_printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
			buf->img_info.pipeline_id,
			buf->img_info.data_type,
			buf->img_info.frame_id,
			buf->img_info.buf_index,
			buf->img_addr.width,
			buf->img_addr.height,
			buf->img_info.data_type,
			buf->img_info.img_format);			
}

int x3_sensor_init(int devId, x3_vin_info_t *vin_info)
{
    int ret = 0;
    HB_MIPI_SensorBindSerdes(&vin_info->snsinfo, vin_info->snsinfo.sensorInfo.deserial_index, vin_info->snsinfo.sensorInfo.deserial_port);
    HB_MIPI_SensorBindMipi(&vin_info->snsinfo, vin_info->snsinfo.sensorInfo.entry_index);
	print_sensor_info(&vin_info->snsinfo);
    ROS_printf("devId is %d\n", devId);
    ROS_printf("lane [%d]\n", vin_info->mipi_attr.mipi_host_cfg.lane);
    ret = HB_MIPI_InitSensor(devId, &vin_info->snsinfo);
    if (ret < 0) {
        ROS_printf("hb mipi init sensor error!, ret: %d\n", ret);
        return ret;
    }
    ROS_printf("hb sensor init success...\n");

    return 0;
}

int x3_mipi_init(x3_vin_info_t *vin_info)
{
    int ret = 0;
    ret = HB_MIPI_SetMipiAttr(vin_info->snsinfo.sensorInfo.entry_index, &vin_info->mipi_attr);
    if (ret < 0) {
        ROS_printf("hb mipi set mipi attr error!\n");
        return ret;
    }
    ROS_printf("hb mipi init success...\n");

    return 0;
}

static int x3_sensor_start(int devId)
{
	int ret = 0;
	ret = HB_MIPI_ResetSensor(devId);
	if (ret < 0)
	{
		ROS_printf("HB_MIPI_ResetSensor error!\n");
		return ret;
	}
	return ret;
}

static int x3_mipi_start(int mipiIdx)
{
	int ret = 0;
    ret = HB_MIPI_ResetMipi(mipiIdx);
    if (ret < 0)
	{
        ROS_printf("HB_MIPI_ResetMipi error, ret= %d\n", ret);
		return ret;
	}
	return ret;
}

static int x3_sensor_stop(int devId)
{
	int ret = 0;
	ret = HB_MIPI_UnresetSensor(devId);
	if (ret < 0)
	{
		ROS_printf("HB_MIPI_UnresetSensor error!\n");
		return ret;
	}
	return ret;
}

static int x3_mipi_stop(int mipiIdx)
{
	int ret = 0;
	ROS_printf("x3_mipi_stop======\n");
	ret = HB_MIPI_UnresetMipi(mipiIdx);
	if (ret < 0)
	{
		ROS_printf("HB_MIPI_UnresetMipi error!\n");
		return ret;
	}
	return ret;
}

static int x3_sensor_deinit(int devId)
{
	int ret = 0;
	ret = HB_MIPI_DeinitSensor(devId);
	if (ret < 0)
	{
		ROS_printf("HB_MIPI_DeinitSensor error!\n");
		return ret;
	}
	return ret;
}

static int x3_mipi_deinit(int mipiIdx)
{
	int ret = 0;
	ROS_printf("x3_sensor_deinit end==mipiIdx %d====\n", mipiIdx);
	ret = HB_MIPI_Clear(mipiIdx);
	if (ret < 0)
	{
		ROS_printf("HB_MIPI_Clear error!\n");
		return ret;
	}
	ROS_printf("x3_mipi_deinit end======\n");
	return ret;
}

void dis_crop_set(uint32_t pipe_id, uint32_t event, VIN_DIS_MV_INFO_S *data,
        void *userdata)
{
    ROS_printf("dis_crop_set callback come in\n");
    ROS_printf("data gmvX %d\n", data->gmvX);
    ROS_printf("data gmvY %d\n", data->gmvY);
    ROS_printf("data xUpdate %d\n", data->xUpdate);
    ROS_printf("data yUpdate %d\n", data->yUpdate);
    return;
}


/* 绑定关系：
 * mipi(vc) -> dev(sif) -> isp(pipeline)
 */
int x3_vin_init(x3_vin_info_t *vin_info)
{
    int ret = 0;
    /*int ipu_ds2_en = 0;*/
    /*int ipu_ds2_crop_en = 0;*/
	int pipeId = vin_info->pipe_id;
	int devId = vin_info->dev_id;

	// 优先初始化sensor，因为sensor与外设相关，容易因为各种硬件问题初始化失败
	/* snsinfo 中的port	id 必须等于pipeId，原因后面再查			 */
	vin_info->snsinfo.sensorInfo.dev_port = pipeId;
	ret = x3_sensor_init(devId, vin_info);
	if (ret < 0) {
		ROS_printf("x3_sensor_init error!\n");
		return -1;
	}
	ROS_printf("x3_sensor_init ok!\n");
	ret = x3_mipi_init(vin_info);
	if (ret < 0) {
		ROS_printf("x3_mipi_init error!\n");
		x3_sensor_deinit(devId);
	}

    VIN_DIS_CALLBACK_S pstDISCallback;
    pstDISCallback.VIN_DIS_DATA_CB = dis_crop_set;

    ret = HB_SYS_SetVINVPSMode(pipeId, vin_info->vin_vps_mode);
    if (ret < 0) {
        ROS_printf("HB_SYS_SetVINVPSMode%d error!\n", vin_info->vin_vps_mode);
        return ret;
    }
    ROS_printf("===>HB_SYS_SetVINVPSMode%d ret =%d!\n", vin_info->vin_vps_mode,ret);
    ret = HB_VIN_CreatePipe(pipeId, &vin_info->pipeinfo);  // isp init
    if (ret < 0) {
        ROS_printf("HB_VIN_CreatePipe error!\n");
        return ret;
    }
    ret = HB_VIN_SetMipiBindDev(devId, vin_info->snsinfo.sensorInfo.entry_index); // mipi和vin(sif) dev 绑定 
    if (ret < 0) {
        ROS_printf("HB_VIN_SetMipiBindDev error!\n");
        return ret;
    }
	ROS_printf("vin_info->mipi_attr.mipi_host_cfg.channel_num: %d\n", vin_info->mipi_attr.mipi_host_cfg.channel_num);
    ret = HB_VIN_SetDevVCNumber(devId, vin_info->mipi_attr.mipi_host_cfg.channel_sel[0]); // 确定使用哪几个虚拟通道作为mipi的输入 
    if (ret < 0) {
        ROS_printf("HB_VIN_SetDevVCNumber error!\n");
        return ret;
    }
	// dol2
    if (2 <= vin_info->mipi_attr.mipi_host_cfg.channel_num) {
		ROS_printf("enable dol2\n");
        ret = HB_VIN_AddDevVCNumber(devId, vin_info->mipi_attr.mipi_host_cfg.channel_sel[1]);
        if (ret < 0) {
            ROS_printf("HB_VIN_AddDevVCNumber error!\n");
            return ret;
        }
    }
	// dol3
	if (3 <= vin_info->mipi_attr.mipi_host_cfg.channel_num) {
        ret = HB_VIN_AddDevVCNumber(devId, vin_info->mipi_attr.mipi_host_cfg.channel_sel[2]);
        if (ret < 0) {
            ROS_printf("HB_VIN_AddDevVCNumber error!\n");
            return ret;
        }
    }

	ROS_printf("start HB_VIN_SetDevAttr \n");
    ret = HB_VIN_SetDevAttr(devId, &vin_info->devinfo);  // sif init
    if (ret < 0) {
        ROS_printf("HB_VIN_SetDevAttr error!\n");
        return ret;
    }
    if (1 == vin_info->enable_dev_attr_ex) {
        ret = HB_VIN_SetDevAttrEx(devId, &vin_info->devexinfo);
        if (ret < 0) {
            ROS_printf("HB_VIN_SetDevAttrEx error!\n");
            return ret;
        }
    }
	ROS_printf("HB_VIN_SetDevAttr ok!\n");
	// 这个接口耗时2.5秒，神奇的一腿
    ret = HB_VIN_SetPipeAttr(pipeId, &vin_info->pipeinfo);  // isp init
    if (ret < 0) {
        ROS_printf("HB_VIN_SetPipeAttr error!\n");
        goto pipe_err;
    }
	ROS_printf("HB_VIN_SetPipeAttr ok!\n");
	// chn指的是dwe,属于chn1,所以这里初始化传1 
	// DWE 主要是将 LDC 和 DIS 集成在一起，包括 LDC 的畸变矫正和 DIS 的统计结果 
    ret = HB_VIN_SetChnDISAttr(pipeId, 1, &vin_info->disinfo);  //  dis init
    if (ret < 0) {
        ROS_printf("HB_VIN_SetChnDISAttr error!\n");
        goto pipe_err;
    }
    if (1 == vin_info->disinfo.disPath.rg_dis_enable) { // 这个地方判断是否使能dis就很奇怪
        HB_VIN_RegisterDisCallback(pipeId, &pstDISCallback);
    }
    ret = HB_VIN_SetChnLDCAttr(pipeId, 1, &vin_info->ldcinfo);  //  ldc init
    if (ret < 0) {
        ROS_printf("HB_VIN_SetChnLDCAttr error!\n");
        goto pipe_err;
    }
	ROS_printf("HB_VIN_SetChnLDCAttr ok!\n");
    ret = HB_VIN_SetChnAttr(pipeId, 1);  //  dwe init
    if (ret < 0) {
        ROS_printf("HB_VIN_SetChnAttr error!\n");
        goto pipe_err;
    }
	ROS_printf("HB_VIN_SetChnAttr ok!\n");
    ret = HB_VIN_SetDevBindPipe(devId, pipeId);  // vin(sif) dev 和 isp pipeline绑定 
    if (ret < 0) {
        ROS_printf("HB_VIN_SetDevBindPipe error!\n");
        goto chn_err;
    }
	ROS_printf("HB_VIN_SetDevBindPipe ok!\n");
	ROS_printf("x3_vin_init ok\n");
    return ret;

chn_err:
    HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
pipe_err:
    HB_VIN_DestroyDev(pipeId);   // sif deinit
    ROS_printf("iot_vin_init failed\n");
    return ret;
}

int x3_vin_start(x3_vin_info_t *vin_info)
{
	int ret = 0;
	ret = HB_VIN_EnableChn(vin_info->pipe_id, 1); // dwe start
	if (ret < 0)
	{
		ROS_printf("HB_VIN_EnableChn error!\n");
		return ret;
	}
	ret = HB_VIN_StartPipe(vin_info->pipe_id); // isp start
	if (ret < 0)
	{
		ROS_printf("HB_VIN_StartPipe error!\n");
		return ret;
	}
	ret = HB_VIN_EnableDev(vin_info->dev_id); // sif start && start thread
    //printf("dev_id:[%d],pip_id:[%d]\n", vin_info->dev_id, vin_info->pipe_id);
    if (ret < 0) {
        ROS_printf("HB_VIN_EnableDev error!\n");
		return ret;
    }
    if (vin_info->vin_vps_mode != VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE) {
		ret = x3_sensor_start(vin_info->dev_id);
		if (ret < 0)
		{
			ROS_printf("x3_sensor_start error!\n");
			return ret;
		}
		ret = x3_mipi_start(vin_info->snsinfo.sensorInfo.entry_index);
		if (ret < 0)
		{
			ROS_printf("x3_sensor_start error!\n");
			return ret;
		}
	}	
	return ret;
}

void x3_vin_stop(x3_vin_info_t *vin_info)
{
	if (vin_info->vin_vps_mode != VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE) {
		x3_sensor_stop(vin_info->dev_id);
		x3_mipi_stop(vin_info->snsinfo.sensorInfo.entry_index);
	}
	HB_VIN_DisableDev(vin_info->dev_id);	  // thread stop && sif stop
	HB_VIN_StopPipe(vin_info->pipe_id);	  // isp stop
	HB_VIN_DisableChn(vin_info->pipe_id, 1); // dwe stop
    ROS_printf("x3_vin_stop ok!\n");
}

void x3_vin_deinit(x3_vin_info_t *vin_info) {
	HB_VIN_DestroyDev(vin_info->dev_id);	  // sif deinit && destroy
	HB_VIN_DestroyChn(vin_info->pipe_id, 1); // dwe deinit
	HB_VIN_DestroyPipe(vin_info->pipe_id); // isp deinit && destroy

	x3_mipi_deinit(vin_info->snsinfo.sensorInfo.entry_index);
	x3_sensor_deinit(vin_info->dev_id);
    ROS_printf("x3_vin_deinit ok!");
}

int x3_vin_feedback(int pipeId, hb_vio_buffer_t *feedback_buf) {
	int ret = 0;
	ret = HB_VIN_SendPipeRaw(pipeId, feedback_buf, 1000);
	return ret;
}

int x3_vin_get_ouput(int pipeId, hb_vio_buffer_t *buffer) {
	int ret = 0;
	ret = HB_VIN_GetChnFrame(pipeId, 0, buffer, 2000);
	return ret;
}

int x3_vin_output_release(int pipeId, hb_vio_buffer_t *buffer) {
	int ret = 0;
	ret = HB_VIN_ReleaseChnFrame(pipeId, 0, buffer);
	if (ret < 0) {
		//ROS_printf("HB_VIN_ReleaseChnFrame error=%d!!!\n",ret);
	}
	return ret;
}


int time_cost_ms(struct timeval *start, struct timeval *end)
{
	int time_ms = -1;
	time_ms = ((end->tv_sec * 1000 + end->tv_usec /1000) -
		(start->tv_sec * 1000 + start->tv_usec /1000));
	ROS_printf("time cost %d ms \n", time_ms);
	return time_ms;
}

int x3_vin_sif_raw_dump(int pipeId, char *file_name) // HB_VIN_GetDevFrame 需要的参数是devID
{
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;
	dump_info_t dump_info = {0};
	int ret = 0;
	hb_vio_buffer_t *sif_raw = NULL;

	sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(sif_raw, 0, sizeof(hb_vio_buffer_t));

	ret = HB_VIN_GetDevFrame(pipeId, 0, sif_raw, 2000);
	if (ret < 0) {
		ROS_printf("HB_VIN_GetDevFrame error!!!\n");
	} else {
		x3_normal_buf_info_print(sif_raw);
		size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
		ROS_printf("stride_size(%u) w x h%u x %u  size %d\n",
			sif_raw->img_addr.stride_size,
			sif_raw->img_addr.width, sif_raw->img_addr.height, size);

		if (size == 0) // 有可能是因为模式不是offline的，所以获取不到raw图
			return -1;
		if (sif_raw->img_info.planeCount == 1) {
			dump_info.ctx_id = pipeId;
			dump_info.raw.frame_id = sif_raw->img_info.frame_id;
			dump_info.raw.plane_count = sif_raw->img_info.planeCount;
			dump_info.raw.xres[0] = sif_raw->img_addr.width;
			dump_info.raw.yres[0] = sif_raw->img_addr.height;
			dump_info.raw.addr[0] = sif_raw->img_addr.addr[0];
			dump_info.raw.size[0] = size;

		ROS_printf("pipe(%d)dump normal raw frame id(%d),plane(%d)size(%d)\n",
			dump_info.ctx_id, dump_info.raw.frame_id,
			dump_info.raw.plane_count, size);
		} else if (sif_raw->img_info.planeCount == 2) {
			dump_info.ctx_id = pipeId;
			dump_info.raw.frame_id = sif_raw->img_info.frame_id;
			dump_info.raw.plane_count = sif_raw->img_info.planeCount;
			for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
				dump_info.raw.xres[i] = sif_raw->img_addr.width;
				dump_info.raw.yres[i] = sif_raw->img_addr.height;
				dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
				dump_info.raw.size[i] = size;
			}
			if(sif_raw->img_info.img_format == 0) {
		ROS_printf("pipe(%d)dump dol2 raw frame id(%d),plane(%d)size(%d)\n",
			dump_info.ctx_id, dump_info.raw.frame_id,
			dump_info.raw.plane_count, size);
			}
		} else if (sif_raw->img_info.planeCount == 3) {
			dump_info.ctx_id = pipeId;
			dump_info.raw.frame_id = sif_raw->img_info.frame_id;
			dump_info.raw.plane_count = sif_raw->img_info.planeCount;
			for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
				dump_info.raw.xres[i] = sif_raw->img_addr.width;
				dump_info.raw.yres[i] = sif_raw->img_addr.height;
				dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
				dump_info.raw.size[i] = size;
		}
		ROS_printf("pipe(%d)dump dol3 raw frame id(%d),plane(%d)size(%d)\n",
			dump_info.ctx_id, dump_info.raw.frame_id,
			dump_info.raw.plane_count, size);
		} else {
			ROS_printf("pipe(%d)raw buf planeCount wrong !!!\n", pipeId);
		}

		for (int i = 0; i < dump_info.raw.plane_count; i ++) {
				if(sif_raw->img_info.img_format == 0) {
				sprintf(file_name, "/tmp/pipe%d_plane%d_%ux%u_frame_%03d.raw",
								dump_info.ctx_id,
								i,
								dump_info.raw.xres[i],
								dump_info.raw.yres[i],
								dump_info.raw.frame_id);

				gettimeofday(&time_now, NULL);
				x3_dumpToFile(file_name,  dump_info.raw.addr[i], dump_info.raw.size[i]);
				gettimeofday(&time_next, NULL);
				int time_cost = time_cost_ms(&time_now, &time_next);
				ROS_printf("dumpToFile raw cost time %d ms", time_cost);
			}
			if(sif_raw->img_info.img_format == 8) {
				sprintf(file_name, "/tmp/pipe%d_%ux%u_frame_%03d.yuv",
								dump_info.ctx_id,
								dump_info.raw.xres[i],
								dump_info.raw.yres[i],
								dump_info.raw.frame_id);
				gettimeofday(&time_now, NULL);
				x3_dump_vio_buf_to_nv12(file_name, sif_raw);
				gettimeofday(&time_next, NULL);
				int time_cost = time_cost_ms(&time_now, &time_next);
				ROS_printf("dumpToFile yuv cost time %d ms", time_cost);
			}
		}
	 ret = HB_VIN_ReleaseDevFrame(pipeId, 0, sif_raw);
	 if (ret < 0) {
		ROS_printf("HB_VIN_ReleaseDevFrame error!!!\n");
	 }
  }
	free(sif_raw);
	sif_raw = NULL;

	return ret;
}

int x3_vin_isp_yuv_dump(int pipeId, char *file_name)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	int time_ms = 0;
	struct timeval select_timeout = {0};

	select_timeout.tv_usec = 500 * 1000;
	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));

	gettimeofday(&time_now, NULL);
	time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;
	ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, 2000);
	if (ret < 0) {
		ROS_printf("HB_VIN_GetPipeFrame error!!!\n");
	} else {
		x3_normal_buf_info_print(isp_yuv);
		size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
		ROS_printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
					isp_yuv->img_addr.stride_size,
					isp_yuv->img_addr.width, isp_yuv->img_addr.height, size);
		sprintf(file_name,
					"/tmp/isp_pipeId%d_yuv_%d_index%d.yuv", pipeId, time_ms,
												isp_yuv->img_info.buf_index);
		gettimeofday(&time_now, NULL);
		x3_dump_vio_buf_to_nv12(file_name, isp_yuv);
		gettimeofday(&time_next, NULL);
		int time_cost = time_cost_ms(&time_now, &time_next);
		ROS_printf("dumpToFile yuv cost time %d ms", time_cost);
		ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
		if (ret < 0) {
			ROS_printf("HB_VIN_ReleaseChnFrame error!!!\n");
		}
	}
	free(isp_yuv);
	isp_yuv = NULL;

	return ret;
}

static int save_jpeg(char *filename, char *srcBuf, unsigned int size)
{
	FILE *fd = NULL;
	fd = fopen(filename, "w+");
	if (fd == NULL) {
		ROS_printf("open(%s) fail", filename);
		return -1;
	}
	fflush(stdout);
	fwrite(srcBuf, 1, size, fd);
	fflush(fd);
	if (fd)
		fclose(fd);
	ROS_printf("DEBUG:save jpeg(%s, size(%d) is successed!!\n", filename, size);

	return 0;
}

int x3_vin_isp_yuv_dump_to_jpeg(VENC_CHN VeChn, int pipeId, char *file_name)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	int size = -1, ret = 0;
	VENC_CHN_ATTR_S vencChnAttr;
	VIDEO_FRAME_S pstFrame;
	VIDEO_STREAM_S pstStream;

	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));

	ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, 2000);
	if (ret < 0) {
		ROS_printf("HB_VIN_GetPipeFrame error!!!\n");
	} else {
		x3_normal_buf_info_print(isp_yuv);
#if 0
		// 获取jpeg编码通道当前的配置
		ret = x3_venc_get_chn_attr(VeChn, &vencChnAttr);
		if (ret < 0) {
			ROS_printf("x3_venc_get_chn_attr error!!!\n");
		}
		// 根据yuv的size设置编码分辨率属性
		vencChnAttr.stVencAttr.u32PicWidth = isp_yuv->img_addr.width;
		vencChnAttr.stVencAttr.u32PicHeight = isp_yuv->img_addr.height;

		HB_VENC_SetChnAttr(VeChn, &vencChnAttr);
#endif
		// 编码成 jpg 图像
		pstFrame.stVFrame.phy_ptr[0] = isp_yuv->img_addr.paddr[0];
		pstFrame.stVFrame.phy_ptr[1] = isp_yuv->img_addr.paddr[1];

		pstFrame.stVFrame.vir_ptr[0] = isp_yuv->img_addr.addr[0];
		pstFrame.stVFrame.vir_ptr[1] = isp_yuv->img_addr.addr[1];
		pstFrame.stVFrame.pts = isp_yuv->img_info.frame_id;
		pstFrame.stVFrame.vstride = isp_yuv->img_addr.stride_size;
		pstFrame.stVFrame.stride = isp_yuv->img_addr.stride_size;
		ret = HB_VENC_SendFrame(VeChn, &pstFrame, 0);
		if (ret < 0) {
			ROS_printf("HB_VENC_SendFrame error, %x\n", ret);
			usleep(100);
			return ret;
		}

		ret = HB_VENC_GetStream(VeChn, &pstStream, 2000);
		if (ret < 0) {
			ROS_printf("HB_VENC_GetStream error, %x\n", ret);
		} else {
			sprintf(file_name,
					"/tmp/isp_pipeId%d_%d_%d_index%d.jpg", pipeId,
					isp_yuv->img_addr.width,
					isp_yuv->img_addr.height, isp_yuv->img_info.buf_index);
			save_jpeg(file_name, pstStream.pstPack.vir_ptr, pstStream.pstPack.size);
			ret = HB_VENC_ReleaseStream(VeChn, &pstStream);
			if (ret < 0) {
				ROS_printf("HB_VENC_ReleaseStream error!!!\n");
			}
		}
			ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
			if (ret < 0) {
				ROS_printf("HB_VIN_ReleaseChnFrame error!!!\n");
			}
		}
	free(isp_yuv);
	isp_yuv = NULL;

	return ret;
}
