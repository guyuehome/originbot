/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

//#include "utils/utils_log.h"
//#include "logging.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
extern int ROS_printf(char *fmt, ...);

void print_vps_chn_attr(VPS_CHN_ATTR_S *chn_attr)
{
	ROS_printf("VPS chn Attr:\n");
	ROS_printf("width %d\n", chn_attr->width);
	ROS_printf("height %d\n", chn_attr->height);
	ROS_printf("pixelFormat %d\n", chn_attr->pixelFormat);
	ROS_printf("enMirror %d\n", chn_attr->enMirror);
	ROS_printf("enFlip %d\n", chn_attr->enFlip);
	ROS_printf("enScale %d\n", chn_attr->enScale);
	ROS_printf("frameDepth %d\n", chn_attr->frameDepth);
	ROS_printf("srcFrameRate %d\n", chn_attr->frameRate.srcFrameRate);
	ROS_printf("dstFrameRate %d\n", chn_attr->frameRate.dstFrameRate);

	return;
}

int x3_vps_group_init(int vps_grp_id, VPS_GRP_ATTR_S *vps_grp_attr)
{
	int ret = 0;
	ret = HB_VPS_CreateGrp(vps_grp_id, vps_grp_attr);
	if (ret) {
		ROS_printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		ROS_printf("created a group ok:vps_grp_id = %d\n", vps_grp_id);
        printf("created a group ok:vps_grp_id = %d\n", vps_grp_id);
    }
	return ret;
}

int x3_setpu_gdc(int vps_grp_id, int vps_chn_id,
        char *gdc_config_file, ROTATION_E enRotation)
{
	FILE *gdc_fd = NULL;
	char *buf = NULL;
	int ret = 0;
	/* set group gdc */
	ROS_printf("start to set GDC!!!\n");
	gdc_fd = fopen(gdc_config_file, "rb");
	if (!gdc_fd) {
		ROS_printf("Can't open gdc bin file!\n");
		return -1;
	}
	fseek(gdc_fd, 0, SEEK_END);
	long len = ftell(gdc_fd);
	fseek(gdc_fd, 0, SEEK_SET);

	buf = (char *)malloc(len);
	if (!buf) {
		ROS_printf("Can't malloc buf for gdc bin\n");
		fclose(gdc_fd);
		return -2;
	}
	memset(buf, 0, len);
	fread(buf, 1, len, gdc_fd);
	fclose(gdc_fd);

	ret = HB_VPS_SetGrpGdc(vps_grp_id, buf, len, 0);
	if (ret) {
	    ROS_printf("HB_VPS_SetGrpGdc error!!!\n");
      free(buf);
      return -3;
	} else {
	    ROS_printf("HB_VPS_SetGrpGdc ok: vps_grp_id = %d\n", vps_grp_id);
	}
	free(buf);
	return 0;
}

int x3_vps_chn_init(int vps_grp_id, int vps_chn_id, VPS_CHN_ATTR_S *chn_attr,
                    ROTATION_E enRotation) {
    /*VPS_CHN_ATTR_S chn_attr;*/
    int ret = 0;
	ret = HB_VPS_SetChnAttr(vps_grp_id, vps_chn_id, chn_attr);
  if (enRotation != ROTATION_0) {
    HB_VPS_SetChnRotate(vps_grp_id, vps_chn_id, enRotation);
  }
	if (ret) {
		ROS_printf("[%s]->HB_VPS_SetChnAttr gID=%d,cID=%d,w:h=%d:%d error, ret:%d.\n",
			__func__,vps_grp_id, vps_chn_id, chn_attr->width,chn_attr->height,ret);
	} else {
		ROS_printf("[%s]->set ipu chn Attr ok: gID=%d, cID=%d,w:h=%d:%d.\n",
			__func__,vps_grp_id, vps_chn_id, chn_attr->width,chn_attr->height);
		HB_VPS_EnableChn(vps_grp_id, vps_chn_id);
	}
	return ret;
}

int x3_vps_start(uint32_t vpsGrpId) {
	int ret = 0;
	ROS_printf("x3_vps_start\n");
	ret = HB_VPS_StartGrp(vpsGrpId);
	if (ret) {
		ROS_printf("HB_VPS_StartGrp error, vpsGrpId: %d\n", vpsGrpId);
	}
	else {
		ROS_printf("start grp ok: grp_id = %d\n", vpsGrpId);
	}
	return ret;
}

void x3_vps_stop(int vpsGrpId) {
	HB_VPS_StopGrp(vpsGrpId);
    ROS_printf("x3_vps_stop ok!\n");
}

void x3_vps_deinit(int vpsGrpId) {
	HB_VPS_DestroyGrp(vpsGrpId);
    ROS_printf("x3_vps_stop ok!");
}

int x3_vps_input(uint32_t vpsGrpId, hb_vio_buffer_t *buffer) {
	int ret = 0;
	ret = HB_VPS_SendFrame(vpsGrpId, buffer, 1000);
	if (ret != 0) {
		ROS_printf("HB_VPS_SendFrame Failed. vpsGrpId=%d ret=%d\n", vpsGrpId, ret);
	}
	return ret;
}

int x3_vps_get_output(uint32_t vpsGrpId, int channel, hb_vio_buffer_t *buffer)
{
	int ret = 0;
	ret = HB_VPS_GetChnFrame(vpsGrpId, channel, buffer, 1000);
	if (ret != 0) {
		ROS_printf("HB_VPS_GetChnFrame Failed. ret = %d\n", ret);
	}

	return ret;
}

int x3_vps_output_release(uint32_t vpsGrpId, int channel, hb_vio_buffer_t *buffer)
{
	int ret = 0;	
	ret = HB_VPS_ReleaseChnFrame(vpsGrpId, channel, buffer);
	if (ret != 0)
	{
		ROS_printf("HB_VPS_ReleaseChnFrame Failed. ret = %d\n", ret);
	}
	return ret;
}

void x3_normal_buf_info_print(hb_vio_buffer_t * buf)
{
	ROS_printf("[x3_normal_buf_info_print]->pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
			buf->img_info.pipeline_id,
			buf->img_info.data_type,
			buf->img_info.frame_id,
			buf->img_info.buf_index,
			buf->img_addr.width,
			buf->img_addr.height,
			buf->img_info.data_type,
			buf->img_info.img_format);
}

int x3_dump_nv12(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");
	if (yuvFd == NULL) {
		ROS_printf("open(%s) fail", filename);
		return -1;
	}
	buffer = (char *)malloc(size + size1);
	if (buffer == NULL) {
		ROS_printf("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}
	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);
	fwrite(buffer, 1, size + size1, yuvFd);
	fflush(yuvFd);
	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);
	ROS_printf("DEBUG:filedump(%s, size(%d) is successed!!\n", filename, size);

	return 0;
}

int x3_dump_vio_buf_to_nv12(char *filename, hb_vio_buffer_t *vio_buf)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;
	int i = 0;
	int stride = 0, width = 0, height = 0;
	if (filename == NULL || vio_buf == NULL)
		return -1;

	stride = vio_buf->img_addr.stride_size;
	width = vio_buf->img_addr.width;
	height = vio_buf->img_addr.height;
	yuvFd = fopen(filename, "w+");
	if (yuvFd == NULL) {
		ROS_printf("open(%s) fail", filename);
		return -1;
	}
	buffer = (char *)malloc(width * height * 3 / 2);
	if (buffer == NULL) {
		ROS_printf("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}

	if (stride == width) {
		memcpy(buffer, vio_buf->img_addr.addr[0], width * height);
		memcpy(buffer + width * height, vio_buf->img_addr.addr[1], width * height / 2);
	} else {
		//jump over stride - width Y
		for (i = 0; i < height; i++) {
			memcpy(buffer+i*width, vio_buf->img_addr.addr[0]+i*stride, width);
		}

		//jump over stride - width UV
		for (i = 0; i < height/2; i++) {
			memcpy(buffer+width * height+i*width, vio_buf->img_addr.addr[1]+i*stride, width);
		}
	}

	fflush(stdout);
	fwrite(buffer, 1, width * height * 3 / 2, yuvFd);
	fflush(yuvFd);
	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);
	ROS_printf("filedump(%s, size(%d) is successed\n", filename, width * height * 3 / 2);
	
	return 0;
}

int x3_save_jpeg(char *filename, char *srcBuf, unsigned int size)
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

int x3_dumpToFile(char *filename, char *srcBuf, unsigned int size)
{
        FILE *yuvFd = NULL;
        char *buffer = NULL; 
        yuvFd = fopen(filename, "w+"); 
        if (yuvFd == NULL) {
                ROS_printf("ERRopen(%s) fail", filename);
                return -1; 
        }   
 
        buffer = (char *)malloc(size); 
        if (buffer == NULL) {
                ROS_printf(":malloc file");
                fclose(yuvFd);
                return -1; 
        }   
 
        memcpy(buffer, srcBuf, size); 
        fflush(stdout); 
        fwrite(buffer, 1, size, yuvFd); 
        fflush(yuvFd); 
        if (yuvFd)
                fclose(yuvFd);
        if (buffer)
                free(buffer);
 
        ROS_printf("filedump(%s, size(%d) is successed\n", filename, size);
 
        return 0;
}


