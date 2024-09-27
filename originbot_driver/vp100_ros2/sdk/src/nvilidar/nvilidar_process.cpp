#include "nvilidar_process.h"
#include <list>
#include <string>
#include "mystring.hpp"
#include <iostream> 
#include <istream> 
#include <sstream>
#include "myconsole.hpp"
#include "mytimer.hpp"

namespace vp100_lidar
{
	LidarProcess::LidarProcess(std::string serialport_name,uint32_t baudrate,get_timestamp_function func,uint64_t unit){
		//lidar para 
		Nvilidar_UserConfigTypeDef  cfg;
		//get the default para 
		LidarDefaultUserConfig(cfg);

		cfg.serialport_name = serialport_name;		//serialport name 
		cfg.serialport_baud = baudrate;	//serialport baud  
		get_timestamp = func;
		time_unit = unit;

		if(func != nullptr){
			lidar_serial.LidarLoadConfig(cfg,get_timestamp,unit);	//serialport  
		}
	}
	LidarProcess::~LidarProcess(){

	}

	//lidar init,for sync para ,get communicate state 
	bool LidarProcess::LidarInitialialize(){
		bool state = false;
		state = lidar_serial.LidarInitialialize();
		//get the state info 
		if(false == state){
			return false;
		}
		return true;
	}

	//turn on the lidar  
	bool LidarProcess::LidarTurnOn(){
		bool state = false;
		state = lidar_serial.LidarTurnOn();

		if(false == state){
			return false;
		}
		return true;
	}

	//ture off the lidar 
	bool LidarProcess::LidarTurnOff(){
		bool state = false;
		state = lidar_serial.LidarTurnOff();

		if(false == state){
			return false;
		}
		return true;
	}

	//get lidar one circle data   
	bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout){
		bool ret_state = false;							//return states 
		bool get_point_state = false;					//get point states 
		std::string  log_string = "";					//string log 

		//get point from serialport or socket 	
		get_point_state = lidar_serial.LidarSamplingProcess(scan, timeout);

		//judge the error code 
		if(scan.error_code != VP100_ERROR_CODE_NORMAL){
			switch (scan.error_code){
				case VP100_ERROR_CODE_RESET:{
					vp100_lidar::Console::error("VP100 Lidar Resetting...");
					break;
				}
				case VP100_ERROR_CODE_MOTOR_LOCK:{
					vp100_lidar::Console::error("VP100 Lidar Motor Stall!");
					break;
				}
				case VP100_ERROR_CODE_UP_NO_POINT:{
					vp100_lidar::Console::error("VP100 Lidar UpBoard No Data!");
					break;
				}
				default:
					break;
			}
		}

		//get no res times 
		if(auto_reconnect_flag){			//auto reconnect 
			ret_state = true;

			if(get_point_state){
				no_response_times = 0;  	
			}
			else{
				scan.points.clear();		  //clear points

				no_response_times++;
				if (no_response_times >= 10) {  //max 20 seconds 
					no_response_times = 0;
					//auto reconnect 
					bool reconnect = LidarAutoReconnect();
					if(true == reconnect) {
						auto_reconnect_times = 0;
					}
					else {
						auto_reconnect_times ++;
						//write auto connect log 
						log_string = "VP100 Auto Reconnect {}......";
						log_string += std::to_string(auto_reconnect_times);
						//show the warnnig for the control 
						vp100_lidar::Console::warning("Auto Reconnect %d......",auto_reconnect_times);
					}
				}	
			}
		}
		else {
			ret_state = true;	

			if(get_point_state){			
				no_response_times = 0; 			 
			}
			else{
				scan.points.clear();		  //clear points 

				no_response_times++;
				if (no_response_times >= 10){  //max 20 seconds 
					ret_state = false;			  //no connect,return false,quit the point state 
					no_response_times = 0;
				}		
			}
		}

		if((0 == scan.points.size()) || (false == ret_state)){
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
		vp100_lidar::TimeStamp::sleepMS(500);				//delay for thread close 
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
	void  LidarProcess::LidarParaSync(Nvilidar_UserConfigTypeDef &cfg){
		//ingnore array apart 
		std::vector<float> elems;
		std::stringstream ss(cfg.ignore_array_string);
		std::string number;
		while (std::getline(ss, number, ',')) {
			elems.push_back(atof(number.c_str()));
		}
		cfg.ignore_array = elems;

		//data to filter 
		if (cfg.ignore_array.size() % 2){
			vp100_lidar::Console::error("ignore array is odd need be even");
		}
		for (uint16_t i = 0; i < cfg.ignore_array.size(); i++){
			if (cfg.ignore_array[i] < -180.0 && cfg.ignore_array[i] > 180.0){
				vp100_lidar::Console::error("ignore array should be between 0 and 360");
			}
		}

		//get auto connect state 
		auto_reconnect_flag = cfg.auto_reconnect;
	}

	//origin data 
	void  LidarProcess::LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg){
		//para to config ,this para willbe change 
		cfg.serialport_baud = 115200;
		cfg.serialport_name = "/dev/nvilidar";
		//this para no change 
		cfg.frame_id = "laser_frame";
		cfg.resolution_fixed = false;		//one circle same points  
		cfg.auto_reconnect = true;			//auto connect  
		cfg.reversion = false;				//add 180.0 state 
		cfg.inverted = false;				//mirror 
		cfg.angle_max = 180.0;
		cfg.angle_min = -180.0;
		cfg.range_max = 64.0;
		cfg.range_min = 0;
		cfg.aim_speed = 6.0;				//6Hz
		cfg.sampling_rate = 3;				//3k
		cfg.angle_offset_change_flag = false;	//change angle offset flag
		cfg.angle_offset = 0.0;				//angle offset 
		cfg.ignore_array_string = "";		//filter some angle 

		cfg.log_enable_flag = true;			//default use log 

		LidarParaSync(cfg);
	}

	//==========================get serialport list=======================================
	std::string LidarProcess::LidarGetSerialList(){
		std::string port;       
		std::vector<NvilidarSerialPortInfo> ports = vp100_lidar::LidarDriverSerialport::getPortList();      
		std::vector<NvilidarSerialPortInfo>::iterator it;

		//列表信息
		if (ports.empty()){
			vp100_lidar::Console::show("Not Lidar was detected.");
			return 0;
		}
		else if (1 == ports.size()){
			it = ports.begin();
			port = (*it).portName;
		}
		else{
			int id = 0;
			for (it = ports.begin(); it != ports.end(); it++){
				vp100_lidar::Console::show("%d. %s  %s\n", id, it->portName.c_str(), it->description.c_str());
				id++;
			}
			while (1){
				vp100_lidar::Console::show("Please select the lidar port:");
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
		if(get_timestamp != nullptr){
			//lidar loading para 
			lidar_serial.LidarLoadConfig(cfg,get_timestamp,time_unit);	//serialport  
		}
	}
}





