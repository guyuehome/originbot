#include "nvilidar_process.h"
#include <list>
#include <string>
#include "myconsole.h"
#include "mytimer.h"
#include "mystring.h"
#include <iostream> 
#include <istream> 
#include <sstream>

namespace vp100_lidar
{
	LidarProcess::LidarProcess(std::string serialport_name,uint32_t baudrate)
	{
		//lidar para 
		Nvilidar_UserConfigTypeDef  cfg;
		//get the default para 
		LidarDefaultUserConfig(cfg);

		cfg.serialport_name = serialport_name;		//serialport name 
		cfg.serialport_baud = baudrate;	//serialport baud  

		lidar_serial.LidarLoadConfig(cfg);	//serialport  
	}
	LidarProcess::~LidarProcess()
	{

	}

	//lidar init,for sync para ,get communicate state 
	bool LidarProcess::LidarInitialialize(){
		return lidar_serial.LidarInitialialize();
	}

	//turn on the lidar  
	bool LidarProcess::LidarTurnOn(){
		return lidar_serial.LidarTurnOn();
	}

	//ture off the lidar 
	bool LidarProcess::LidarTurnOff(){
		return lidar_serial.LidarTurnOff();
	}

	//get lidar one circle data   
	bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout){
		bool ret_state = false;							//return states 
		bool get_point_state = false;					//get point states 
		static uint32_t  no_response_times = 0;			//cannot receive data times 
		static uint32_t  auto_reconnect_times = 0;		//auto reconnect times 

		//get point from serialport or socket 	
		get_point_state = lidar_serial.LidarSamplingProcess(scan, timeout);

		//get no res times 
		if(auto_reconnect_flag)			//auto reconnect 
		{
			ret_state = true;

			if(get_point_state)
			{
				no_response_times = 0;  	
			}
			else 
			{
				scan.points.clear();		  //clear points

				no_response_times++;
				if (no_response_times >= 10)  //max 20 seconds 
				{
					no_response_times = 0;

					//auto reconnect 
					bool reconnect = LidarAutoReconnect();
					if(true == reconnect)
					{
						auto_reconnect_times = 0;
					}
					else 
					{
						auto_reconnect_times ++;
						vp100_lidar::console.warning("Auto Reconnect %d......",auto_reconnect_times);
					}
				}	
			}
		}
		else 
		{
			ret_state = true;	

			if(get_point_state)
			{
				no_response_times = 0; 			 
			}
			else 
			{
				scan.points.clear();		  //clear points 

				no_response_times++;
				if (no_response_times >= 10)  //max 20 seconds 
				{
					ret_state = false;			  //no connect,return false,quit the point state 
					no_response_times = 0;
				}		
			}
		}

		return ret_state;
	}

	//quit  
	void LidarProcess::LidarCloseHandle(){
		lidar_serial.LidarCloseHandle();
	}

	//auto reconnect 
	bool LidarProcess::LidarAutoReconnect(){
		LidarCloseHandle();			//first,close the connect 
		delayMS(500);				//delay for thread close 
		if(false == LidarInitialialize())		//thread init 
		{
			return false;
		}
		if(false == LidarTurnOn())			//open lidar output point 
		{
			return false;
		}
		return true;
	}
	

	//=========================parameter sync=================================

	//lidar data  sync 
	void  LidarProcess::LidarParaSync(Nvilidar_UserConfigTypeDef &cfg)
	{
		//ingnore array apart 
		std::vector<float> elems;
		std::stringstream ss(cfg.ignore_array_string);
		std::string number;
		while (std::getline(ss, number, ',')) {
			elems.push_back(atof(number.c_str()));
		}
		cfg.ignore_array = elems;

		//data to filter 
		if (cfg.ignore_array.size() % 2)
		{
			vp100_lidar::console.error("ignore array is odd need be even");
		}
		for (uint16_t i = 0; i < cfg.ignore_array.size(); i++)
		{
			if (cfg.ignore_array[i] < -180.0 && cfg.ignore_array[i] > 180.0)
			{
				vp100_lidar::console.error("ignore array should be between 0 and 360");
			}
		}

		//get auto connect state 
		auto_reconnect_flag = cfg.auto_reconnect;
	}

	//origin data 
	void  LidarProcess::LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg)
	{
		//lidar model 
		cfg.lidar_model_name = NVILIDAR_VP100;
		//para to config 
		cfg.serialport_baud = 115200;
		cfg.serialport_name = "/dev/nvilidar";
		cfg.frame_id = "laser_frame";
		cfg.resolution_fixed = false;		//one circle same points  
		cfg.auto_reconnect = true;			//auto connect  
		cfg.reversion = false;				//add 180.0 state 
		cfg.inverted = false;				//mirror 
		cfg.angle_max = 180.0;
		cfg.angle_min = -180.0;
		cfg.range_max = 64.0;
		cfg.range_min = 0;
		cfg.aim_speed = 10.0;				//10Hz
		cfg.sampling_rate = 10;				//10k
		cfg.angle_offset_change_flag = false;	//change angle offset flag
		cfg.angle_offset = 0.0;				//angle offset 
		cfg.ignore_array_string = "";		//filter some angle 

		LidarParaSync(cfg);
	}

	//==========================get serialport list=======================================
	std::string LidarProcess::LidarGetSerialList(){
		std::string port;       
		std::vector<NvilidarSerialPortInfo> ports = vp100_lidar::LidarDriverSerialport::getPortList();      
		std::vector<NvilidarSerialPortInfo>::iterator it;

		//列表信息
		if (ports.empty())
		{
			vp100_lidar::console.show("Not Lidar was detected.");
			return 0;
		}
		else if (1 == ports.size())
		{
			it = ports.begin();
			port = (*it).portName;
		}
		else
		{
			int id = 0;
			for (it = ports.begin(); it != ports.end(); it++)
			{
				vp100_lidar::console.show("%d. %s  %s\n", id, it->portName.c_str(), it->description.c_str());
				id++;
			}
			while (1)
			{
				vp100_lidar::console.show("Please select the lidar port:");
				std::string number;
				std::cin >> number;

				//参数不合法 
				if ((size_t)atoi(number.c_str()) >= ports.size())
				{
					continue;
				}
				//参数配置 
				it = ports.begin();
				id = atoi(number.c_str());

				//查找  
				port = ports.at(id).portName;

				break;
			}
		}

		return port;
	}

	//=============================ROS interface,for reload the parameter ===========================================
	void LidarProcess::LidarReloadPara(Nvilidar_UserConfigTypeDef cfg){
		LidarParaSync(cfg);
		lidar_serial.LidarLoadConfig(cfg);	//serialport  
	}
}





