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

#ifndef X3_UTILS_H_
#define X3_UTILS_H_

typedef enum {
	E_CHIP_X3M,
	E_CHIP_X3E,
	E_CHIP_UNKNOW,
} E_CHIP_TYPE;


#define SENSOR_F37_SUPPORT			1
#define SENSOR_IMX415_SUPPORT		2
#define SENSOR_OS8A10_SUPPORT		4
#define SENSOR_OV8856_SUPPORT		8
#define SENSOR_SC031GS_SUPPORT		16
#define SENSOR_IMX586_SUPPORT		32
#define SENSOR_GC4C33_SUPPORT		64

typedef struct {
	E_CHIP_TYPE m_chip_type; // 芯片类型
	unsigned int m_sensor_list; // 每个bit对应一种sensor
}hard_capability_t; // 硬件能力

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

E_CHIP_TYPE x3_get_chip_type(void);
int x3_get_hard_capability(hard_capability_t *capability);

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char *res, int max);

// 获取连接的video_device
// 成功返回sensor_id_list中对应的sensor_name，失败返回空字符串
char *x3_get_video_device();

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_UTILS_H_
