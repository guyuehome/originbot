#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include "serial/nvilidar_serial.h"
#include <string>
#include <vector>
#include <stdint.h>
#include "myconsole.h"
#include "mytimer.h"

//serial port 
#include "serial/nvilidar_serial.h"
#include "nvilidar_driver_serialport.h"

//---定义库信息 VS系列的生成库文件  
#ifdef WIN32
	#define NVILIDAR_API __declspec(dllexport)
#else
	#define NVILIDAR_API
#endif // ifdef WIN32

namespace vp100_lidar
{
    //lidar driver 
	class  NVILIDAR_API LidarProcess
    {
		public:
			LidarProcess(std::string serialport_name,uint32_t baudrate);		//para     name - serial_name   baud - serial_baudrate
			~LidarProcess();

			bool LidarInitialialize();			//雷达初始化 包括读及写参数等等功能 
			bool LidarSamplingProcess(LidarScan &scan, uint32_t timeout = NVILIDAR_POINT_TIMEOUT);
			bool LidarTurnOn();					//雷达启动扫描 
			bool LidarTurnOff();				//雷达停止扫描 
			void LidarCloseHandle();			//关掉串口及网络  并退出雷达 
			bool LidarAutoReconnect();			//自动重连 

			//其它接口 有需要可以调用 
			std::string LidarGetSerialList();	
			void LidarReloadPara(Nvilidar_UserConfigTypeDef cfg);

		private:
			LidarDriverSerialport	lidar_serial;	//SERIALPORT
			bool  auto_reconnect_flag = false;		//auto reconnect 

			void LidarParaSync(Nvilidar_UserConfigTypeDef &cfg);		//同步参数信息  主要用于ros 
			void LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg);		//获取默认参数  可以在此修改
    };
}
