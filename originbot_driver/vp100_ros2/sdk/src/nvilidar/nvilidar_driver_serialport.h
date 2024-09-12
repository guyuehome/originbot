#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include "serial/nvilidar_serial.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <math.h>

#if defined(_WIN32)
#include <conio.h>
#include <WinSock2.h>
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>
#else
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#endif

//serialport info 
typedef struct 
{
	std::string portName;
	std::string description;
}NvilidarSerialPortInfo;

//---vs lib 
#ifdef WIN32
	#define NVILIDAR_DRIVER_SERIAL_API __declspec(dllexport)
#else
	#define NVILIDAR_DRIVER_SERIAL_API
#endif // ifdef WIN32

namespace vp100_lidar
{
    //lidar driver 
	class  NVILIDAR_DRIVER_SERIAL_API LidarDriverSerialport
    {
		public:
			LidarDriverSerialport();                
			~LidarDriverSerialport();          

			void LidarLoadConfig(Nvilidar_UserConfigTypeDef cfg);
			bool LidarIsConnected();			//is lidar connected 
			bool LidarGetScanState();			//is lidar transing data 
			bool LidarInitialialize();			//lidar init 
			bool LidarCloseHandle();			//lidar quit and disconnect 
			bool LidarTurnOn();					//start scan    
			bool LidarTurnOff();				//stop scan 


			std::string getSDKVersion();										//get current sdk version 
			static std::vector<NvilidarSerialPortInfo> getPortList();			//get serialport list 
			bool StartScan(void);                           //start scan 
			bool StopScan(void);                            //stop scan 

			bool LidarSamplingProcess(LidarScan &scan, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);  //lidar data output 

			Nvilidar_PackageStateTypeDef   lidar_state;											//lidar state 

		private:
			uint16_t LidarCheckCRC(uint8_t *data, uint16_t len);		//check crc value  
			bool LidarConnect(std::string portname, uint32_t baud = 921600);  //serialport init 
			void LidarDisconnect();      //close serialport 
			bool SendSerial(const uint8_t *data, size_t size);      //send data to serail 
			void FlushSerial();		//flush serialport data 
			bool SendCommand(uint8_t cmd, uint8_t *payload = NULL,uint16_t payloadsize = 0);
			void PointDataUnpack(uint8_t *byte, uint16_t len);		//unpack（point cloud）
			void PointDataAnalysis(Nvilidar_PointViewerPackageInfoTypeDef data);	
			
			//thread  
			bool createThread();		//create thread 
			void closeThread();			//close thread 
			bool waitNormalResponse(uint32_t timeout = NVILIDAR_POINT_TIMEOUT);	//wait for lidar response nomal data 
			void setNormalResponseUnlock();	//unlock nomal data  
			void setCircleResponseUnlock();	//unlock point data 
			void LidarSamplingData(CircleDataInfoTypeDef info, LidarScan &outscan);		//interface for lidar point data 

			//----------------------serialport---------------------------

			vp100_serial::Nvilidar_Serial serialport;

			//----------------------value ----------------------------
			Nvilidar_UserConfigTypeDef     lidar_cfg;				//lidar config data 
			CircleDataInfoTypeDef		   circleDataInfo;			//lida circle data  

			uint32_t    m_0cIndex = 0;                  //0 index
			int32_t     m_last0cIndex = 0;              //0 index
			uint32_t    m_differ0cIndex = 0;            //0 index
			bool        m_first_circle_finish = false;  //first circle finish,case calc fault
			uint64_t	m_run_circles = 0;				//has send data   

			//---------------------thread---------------------------
			#if defined(_WIN32)
				HANDLE  _thread = NULL;
				HANDLE  _event_analysis;		
				HANDLE  _event_circle;			

				DWORD static WINAPI periodThread(LPVOID lpParameter);		//thread  
			#else 
				pthread_t _thread = -1;
				pthread_cond_t _cond_analysis;
				pthread_mutex_t _mutex_analysis;
				pthread_cond_t _cond_point;
				pthread_mutex_t _mutex_point;
				static void *periodThread(void *lpParameter) ;
			#endif
    };
}
