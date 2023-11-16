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
#include <errno.h>

//#include "utils/common_utils.h"
#include "x3_utils.h"
//#include "utils/cJSON.h"

// 获取主芯片类型
E_CHIP_TYPE x3_get_chip_type(void)
{
	int ret = 0;
	FILE *stream;
	char chip_id[16] = {0};

	stream = fopen("/sys/class/socinfo/chip_id", "r");
	if (!stream) {
		ROS_printf("open fail: %s\n", strerror(errno));
		return -1; 
	}   
	ret = fread(chip_id, sizeof(char), 9, stream);
	if (ret != 9) {
		ROS_printf("read fail: %s\n", strerror(errno));
		fclose(stream);
		return -1; 
	}   
	fclose(stream);

	if (strncmp("0xab36300", chip_id, 9) == 0)
		return E_CHIP_X3M; // x3m
	else if (strncmp("0xab37300", chip_id, 9) == 0)
		return E_CHIP_X3E; // x3e
	else return E_CHIP_X3M;
}

typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char sensor_name[10];  // sensor名字
} sensor_id_t;

#define I2C_ADDR_8		1
#define I2C_ADDR_16		2

// mipi sensor的信息数组
sensor_id_t sensor_id_list[] = {
    {2, 0x40, I2C_ADDR_8, 0x0B, "f37"},  // F37
    {1, 0x40, I2C_ADDR_8, 0x0B, "f37"},  // F37
    {1, 0x30, I2C_ADDR_16, 0x0317, "sc132gs"},
    {1, 0x29, I2C_ADDR_16, 0x03f0, "gc4663"},  // GC4663
    {2, 0x29, I2C_ADDR_16, 0x03f0, "gc4663"},  // GC4663
};
#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))

// 获取连接的video_device
// 成功返回sensor_id_list中对应的sensor_name，失败返回空字符串
char *x3_get_video_device() {
  int i = 0;
  char cmd[128] = {0};
  char result[1024] = {0};
  int length = ARRAY_SIZE(sensor_id_list);
  for (i = 0; i < length; i++) {
    /* 通过i2ctransfer命令读取sensor寄存器，判断是否读取成功来判断是否支持相应的sensor
     */
    memset(cmd, '\0', sizeof(cmd));
    memset(result, '\0', sizeof(result));
    if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_8) {
      sprintf(cmd,
              "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
              sensor_id_list[i].i2c_bus,
              sensor_id_list[i].i2c_dev_addr,
              sensor_id_list[i].det_reg);
    } else if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_16) {
      if (0x30 == sensor_id_list[i].i2c_dev_addr) {
          sprintf(cmd,
                  "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
                  1,
                  0x30,
                  0x31,
                  0x7);
      } else {
		sprintf(cmd,
				"i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
				sensor_id_list[i].i2c_bus,
				sensor_id_list[i].i2c_dev_addr,
				sensor_id_list[i].det_reg >> 8,
				sensor_id_list[i].det_reg & 0xFF);
				} 
	}else {
			ROS_printf("error, invalid i2c_addr_width: %d!!", sensor_id_list[i].i2c_addr_width);
      continue;
    }
    exec_cmd_ex(cmd, result, 1024);
    if (strstr(result, "Error") == NULL) {  // 读取sensor寄存器失败的时候返回结果中会有Error
				//返回结果中不带Error, 说明sensor找到了
      ROS_printf("match sensor:%s\n", sensor_id_list[i].sensor_name);
      return sensor_id_list[i].sensor_name;
    } else {
			// do nothing 
			// 当前检测的摄像头不是连接的摄像头，继续检测下一个
		}
  }
  return "";
}

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char* res, int max)
{
	if(cmd == NULL || res == NULL || max <= 0)
		return -1;
	FILE *pp = popen(cmd, "r");
	if(!pp)
	{
		ROS_printf("error, cannot popen cmd: %s\n", cmd);
		return -1;
	}
	int length;
	char tmp[1024] = {0};
	length = max;
	if(max > 1024) length = 1024;
	ROS_printf("[%s]->cmd %s ,fp=0x%x,len=%d.\n",__func__,cmd,pp,max);
	while(fgets(tmp, length, pp) != NULL)
	{
		sscanf(tmp, "%s", res);
	}
	pclose(pp);

	return strlen(res);
}
