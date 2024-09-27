#include <stdio.h>
#include <iostream>
#include "nvilidar_process.h"


using namespace std;
using namespace nvilidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif


int main()
{
	printf(" _   ___      _______ _      _____ _____          _____ \n");
	printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
	printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
	printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
	printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
	printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
	printf("\n");
	fflush(stdout);

	//init signal,for ctrl+c
	nvilidar::sigInit();

	//version 
	printf("\nthe Current Lidar's SDK Version is: %s\n\n",NVILIDAR_SDKVerision);

	//introduce 
	printf("Note:\n");
	printf("Current sdk supports 2 types of lidar,include roc300,vp350 \n");
	printf("The roc300 is a USB interface lidar, supporting various baud rates, 921600bps is recommended.\n");
	printf("vp350 is a serial interface lidar, supporting only 512000bps.\n\n");

	//roc300 lidar is USB-CDC,can use any baudrate.best 921600bps,
	//vp350 must be 512000bps
	//init para,the network para is different with serialport  
	#if 1
		nvilidar::LidarProcess lidar(USE_SERIALPORT,"/dev/nvilidar",512000);
	#else 
		nvilidar::LidarProcess lidar(USE_SOCKET, "192.168.1.200", 8100); 
	#endif 

	//init lidar,include sync lidar para 
	if (false == lidar.LidarInitialialize())		
	{
		return 0;
	}

	//open lidar to output point 
	bool ret = lidar.LidarTurnOn();

	//point data analysis 
	while (ret && (nvilidar::isOK()))
	{
		LidarScan scan;

		if (lidar.LidarSamplingProcess(scan))
		{
			if(scan.points.size() > 0)
			{
				for (size_t i = 0; i < scan.points.size(); i++)
				{
					//float angle = scan.points.at(i).angle;
					//float dis = scan.points.at(i).range;
					//printf("a:%f,d:%f\n", angle, dis);
				}
				nvilidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
					scan.stamp, (unsigned int)scan.points.size(),
					1.0 / scan.config.scan_time);
			}
			else 
			{
				nvilidar::console.warning("Lidar Data Invalid!");
			}
		}
		else
		{
			nvilidar::console.error("Failed to get Lidar Data!");
			break;
		}

		delayMS(5);		//add sleep,otherwise high cpu 
	}
	lidar.LidarTurnOff();       //stop scan 
	nvilidar::console.message("Lidar is Stopping......");
	lidar.LidarCloseHandle();   //close connect 

	delayMS(100);

	return 0;
}
