#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include "serial/nvilidar_serial.h"
#include <string>
#include <vector>
#include <stdint.h>

//serial port 
#include "serial/nvilidar_serial.h"
#include "nvilidar_driver_serialport.h"

//---vs library 
#ifdef WIN32
	#define NVILIDAR_API __declspec(dllexport)
#else
	#define NVILIDAR_API
#endif // ifdef WIN32

namespace vp100_lidar{
	typedef uint64_t (*get_timestamp_function)(void);   //timestamp callback 

    //lidar driver 
	class  NVILIDAR_API LidarProcess{
		public:
			LidarProcess(std::string serialport_name,uint32_t baudrate,get_timestamp_function func,uint64_t unit = 1e9);		//para     name - serial_name   baud - serial_baudrate
			~LidarProcess();

			bool LidarInitialialize();			//lidar init 
			bool LidarSamplingProcess(LidarScan &scan, uint32_t timeout = NVILIDAR_POINT_TIMEOUT);
			bool LidarTurnOn();					//lidar turn on 
			bool LidarTurnOff();				//lidar turn off 
			void LidarCloseHandle();			//close the serialport
			bool LidarAutoReconnect();			//auto connect 

			//其它接口 有需要可以调用 
			std::string LidarGetSerialList();	
			void LidarReloadPara(Nvilidar_UserConfigTypeDef cfg);

		private:
			LidarDriverSerialport	lidar_serial;	//SERIALPORT
			bool  auto_reconnect_flag = false;		//auto reconnect 
			//---------------------callback function----------------
			get_timestamp_function  get_timestamp = nullptr;  //stamp 
			uint64_t                time_unit = 1e9;		  //unit 

			uint32_t  no_response_times = 0;			//cannot receive data times 
			uint32_t  auto_reconnect_times = 0;			//auto reconnect times 

			void LidarParaSync(Nvilidar_UserConfigTypeDef &cfg);		//同步参数信息  主要用于ros 
			void LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg);		//获取默认参数  可以在此修改
    };
}
