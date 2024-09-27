#include "nvilidar_driver_serialport.h"
#include "serial/nvilidar_serial.h"
#include <list>
#include <string>
#include "mystring.hpp"
#include "myconsole.hpp"
#include "mytimer.hpp"
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>

namespace vp100_lidar
{
	LidarDriverSerialport::LidarDriverSerialport(){
		lidar_state.m_CommOpen = false;
		//default value     
		circleDataInfo.differStamp = 0;
		circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;
		circleDataInfo.lidarCircleNodePoints.clear();
		circleDataInfo.startStamp = 0;
		circleDataInfo.stopStamp = 0;	
	}

	LidarDriverSerialport::~LidarDriverSerialport(){
		LidarDisconnect();
	}

	//load para 
	void LidarDriverSerialport::LidarLoadConfig(Nvilidar_UserConfigTypeDef cfg,get_timestamp_serial_function func,uint64_t unit){
		lidar_cfg = cfg;  
		get_timestamp = func;  
		time_unit = unit;                     
	}

	//is lidar connected 
	bool LidarDriverSerialport::LidarIsConnected(){
		if(lidar_state.m_CommOpen){
			return true;
		}
		return false;
	}

	//lidar init 
	bool LidarDriverSerialport::LidarInitialialize(){
		VP100_Head_LidarInfo_TypeDef vp100_info;		
		//para is valid?
		if ((lidar_cfg.serialport_name.length() == 0) || (lidar_cfg.serialport_baud == 0)){
			return false;		
		}

		//start to connect serialport 
		if (!LidarConnect(lidar_cfg.serialport_name, lidar_cfg.serialport_baud)){
			vp100_lidar::Console::error("VP100 Connect Serialport failed!");
			return false;		//serialport connect fail 
		}

		//send stop cmd 
		StopScan();
		//sleep 
		vp100_lidar::TimeStamp::sleepMS(100); 
		//create thread to read serialport data   
		createThread();	
		//sleep 
		vp100_lidar::TimeStamp::sleepMS(300); 
		// //get the information
		// if(false == LidarGetInfo(vp100_info)){
		// 	vp100_lidar::Console::error("Get VP100 Info Error!");
		// 	return false;
		// }
		// vp100_lidar::Console::show("\nVP100 Lidar Device Info:\n");
		// vp100_lidar::Console::show("Model        : %s", vp100_info.upBoard_Model.c_str());
		// vp100_lidar::Console::show("Soft Version : %s", vp100_info.upBoard_SoftVersion.c_str());
		// vp100_lidar::Console::show("Hard Version : %s", vp100_info.upBoard_HardVersion.c_str());
		// vp100_lidar::Console::show("ID           : %s", vp100_info.upBoard_ID.c_str());
		// vp100_lidar::Console::show("Date         : %s", vp100_info.upBoard_Date.c_str());
		// vp100_lidar::Console::show("\n");

		printf("p:%s,b:%d\n",lidar_cfg.serialport_name.c_str(),lidar_cfg.serialport_baud);
		return true;
	}

	//lidar start  
	bool LidarDriverSerialport::LidarTurnOn(){
		//success 
		vp100_lidar::Console::message("Now NVILIDAR is scanning ......");

		return true;
	}

	//lidar stop 
	bool LidarDriverSerialport::LidarTurnOff(){
		return true;
	}

	//close handle and close serialport 
	bool LidarDriverSerialport::LidarCloseHandle()
	{
		LidarDisconnect();
		return true;
	}

	//---------------------------------------private---------------------------------

	//check crc 
	uint16_t LidarDriverSerialport::LidarCheckCRC(uint8_t *data, uint16_t len) {
		uint32_t temp_crc_value = 0;
		uint32_t crc_value = 0;

		if (len % 2 != 0) {
			return 0;
		}

		for (int i = 0; i < len/2; i++) {
			uint16_t temp_u16;
			uint16_t temp_u16_high = 0;
			uint16_t temp_u16_low = 0;

			temp_u16_high = data[i * 2 + 1];
			temp_u16_high <<= 8;
			temp_u16_low = data[2 * i];

			temp_u16 = temp_u16_high | temp_u16_low;
			//printf("uint16_t:%04x\n", temp_u16);

			temp_crc_value = (temp_crc_value << 1) + temp_u16;
		}
		crc_value = (temp_crc_value & 0x7FFF) + (temp_crc_value >> 15);
		crc_value = crc_value & 0x7FFF;

		return crc_value;
	}

	//T10 check crc 
	uint8_t LidarDriverSerialport::T10CheckCrc(uint8_t *data, uint16_t len) {
		uint8_t crc = 0;
		for (uint16_t i = 0; i < len; i++){
			crc = T10_CrcTable[(crc ^ *data++) & 0xff];
		}
		return crc;
	}

	//check add sum 
	uint8_t LidarDriverSerialport::checkAddSum(uint8_t *data, uint16_t len) {
		uint8_t check_sum_value = 0;

		if(len < 1){
			return 0;
		}
		for (uint16_t i = 0; i < len - 1; i++) {
			check_sum_value += data[i];
		}
		return check_sum_value;
	}

	//add hex byte to string 
	std::string LidarDriverSerialport::hexBytesToString(const char* data, size_t length) {
		std::stringstream ss;
		ss << std::hex << std::uppercase << std::setfill('0');
		for (size_t i = 0; i < length; ++i) {
			ss << std::setw(2) << static_cast<unsigned>(static_cast<unsigned char>(data[i]));
		}
		return ss.str();
	}

	//lidar start 
	bool LidarDriverSerialport::StartScan(){
		//serialport state 
		if (!lidar_state.m_CommOpen){
			return false;
		}

		//first circle false
		m_first_circle_finish = false;

		return true;
	}

	//lidar stop 
	bool  LidarDriverSerialport::StopScan(){
		//flush data  
		FlushSerial();

		return true;
	}

	//lidar reset 
	bool  LidarDriverSerialport::LidarReset(){
		//send the command
		if(false == SendCommand(0x03)){
        	return false;
    	}
		//delay for 300ms 
		vp100_lidar::TimeStamp::sleepMS(300);
		return true;
	}

	//lidar reset and get the lidar info 
	bool  LidarDriverSerialport::LidarGetInfo(VP100_Head_LidarInfo_TypeDef &info,uint32_t timeout){
		protocol_receive_flag = false;
		//send the command
		if(false == SendCommand(0x03)){
        	return false;
    	}
		//wait for ack
		if (waitNormalResponse(timeout)){
			if (protocol_receive_flag){
				info = vp100_lidar_info;
				return true;
			}
		}
		return false;
	}

	//lidar connect via serialport 
	bool LidarDriverSerialport::LidarConnect(const std::string portname, const uint32_t baud){
		serialport.serialInit(portname,baud);
		serialport.serialOpen();
		
		if (serialport.isSerialOpen()){
			lidar_state.m_CommOpen = true;

			return true;
		}
		else{
			lidar_state.m_CommOpen = false;

			return false;
		}
		return false;
	}

	//close serialport 
	void LidarDriverSerialport::LidarDisconnect(){
		lidar_state.m_CommOpen = false;
		serialport.serialClose();	
	}

	//send serial 
	bool LidarDriverSerialport::SendSerial(const uint8_t *data, size_t size)
	{
		if (!lidar_state.m_CommOpen){
			return false;
		}

		if (data == NULL || size == 0){
			return false;
		}
	
		//write data   
		size_t r;
		while (size){
			r = serialport.serialWriteData(data,size);
			if (r < 1){
				return false;
			}

			size -= r;
			data += r;
		}

		return true;
	}

	//flush serial data 
	void LidarDriverSerialport::FlushSerial(){
		if (!lidar_state.m_CommOpen)
		{
			return;
		}

		serialport.serialFlush();
	}

	//send data 
	bool LidarDriverSerialport::SendCommand(uint8_t cmd, uint8_t *payload, uint16_t payloadsize){
		uint8_t temp_buf[256];
		uint8_t temp_crc = 0;

		//serialport not open 
		if (!lidar_state.m_CommOpen){
			return false;
		}

		if((payload == nullptr) && (payloadsize == 0)){
			temp_buf[0] = 0xA5;
			temp_buf[1] = 0x5A;
			temp_buf[2] = cmd;
			temp_buf[3] = 0x00;
			for(int i = 0; i<4; i++){
				temp_crc += temp_buf[i];
			}
			temp_buf[4] = temp_crc;
			SendSerial((uint8_t *)temp_buf,5);

		}else if((payload != nullptr) && (payloadsize > 0)){
			temp_buf[0] = 0xA5;
			temp_buf[1] = 0x5A;
			temp_buf[2] = cmd;
			temp_buf[3] = payloadsize;
			for(int i = 0; i< payloadsize; i++){
				temp_buf[4+i] = payload[i];
			}
			for(int i = 0; i<4+payloadsize; i++){
				temp_crc += payload[i];
			}
			temp_buf[4+payloadsize] = temp_crc;

			SendSerial((uint8_t *)temp_buf,5+payloadsize);
		}

		return true;
	}

	//analysis point 
	void LidarDriverSerialport::PointDataUnpack(uint8_t *buf,uint16_t len){

		//check the size is zero?
		if(0 == len){
			return;
		}

		//loop 
		for(uint16_t pos = 0; pos<len; pos++){
			//get the byte
			uint8_t byte = buf[pos];

			//add to buffer 
			if(recv_pos < sizeof(VP100_Node_Package_Union)){
				pack_buf_union.buf[recv_pos] = byte;
			}else {
				recv_pos = 0;
			}

			switch (recv_pos){
				case 0: {    		//first byte 
					if (byte == (uint8_t)(NVILIDAR_POINT_HEADER & 0xFF)){	
						recv_pos++;   
					}else if(byte == (uint8_t)(NVILIDAR_T10_POINT_HEADER & 0xFF)){
						recv_pos++;
					}else if(0xA5 == byte){
                        recv_pos++;
                    }else {        	//check failed 		
						recv_pos = 0;
						break;
					}
					break;
				}
				case 1: {    		//second byte 
					if(0x55 == pack_buf_union.buf[0]){
						//point cloud protocol 
						if (byte == (uint8_t)(NVILIDAR_POINT_HEADER >> 8)){
							recv_pos++;  
						}else if( (0xAB == byte) || (0xAC == byte) || (0xAD == byte) || (0xAE == byte) || 
								  (0xAF == byte) || (0xB0 == byte) || (0xB6 == byte) || (0xB7 == byte) ||
							      (0xBA == byte)) {
							recv_pos++;
						}else{
							recv_pos = 0;
						}
					}
					//t10 protocol
					else if((uint8_t)(NVILIDAR_T10_POINT_HEADER & 0xFF) == pack_buf_union.buf[0]){
						if(byte == (uint8_t)(NVILIDAR_T10_POINT_HEADER >> 8)){
							recv_pos++;
						}else{
							recv_pos = 0;
						}
					}else if(0xA5 == pack_buf_union.buf[0]) {
						if(0xAB == byte){
							recv_pos++;
						}else{
							recv_pos = 0;
						}
					}else{
						recv_pos = 0;
					}
					break;
				}
				case 2: {    		//information 
					if((0x55 == pack_buf_union.buf[0]) && (0xAA != pack_buf_union.buf[1])){	   		//0x55 0xXX ===> downboard info 
						if(0 != byte){
							recv_pos++;
							pack_size = byte + 4;  //package info（3byte head  + 1byte crc）
						}else{
							recv_pos = 0;
						}
					}else if((0xA5 == pack_buf_union.buf[0]) && (0xAB == pack_buf_union.buf[1])){	//0xA5 0xAB ===> upboard info 
						if(0 != byte){
							recv_pos++;
						}else{
							recv_pos = 0;
						}
					}else if((0x55 == pack_buf_union.buf[0]) && (0xAA == pack_buf_union.buf[1])) {   //0x55 0xAA ===> package head 
						if( (((PROTOCOL_VP100_NORMAL_NO_QUALITY >> 8)& 0xFF) == byte) ||
							(((PROTOCOL_VP100_NORMAL_QUALITY >> 8)& 0xFF) == byte) ||
							(((PROTOCOL_VP100_YW_QUALITY >> 8)& 0xFF) == byte) ||
							(((PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY >> 8)& 0xFF) == byte) || 
							(((PROTOCOL_VP100_ERROR_FAULT >> 8)& 0xFF) == byte) ) {
								lidar_model_code = (int)(byte << 8);		//high 8 bit 
								recv_pos++;
							}
                    }else if( ((uint8_t)(NVILIDAR_T10_POINT_HEADER & 0xFF) == pack_buf_union.buf[0]) && 
							  ((uint8_t)(NVILIDAR_T10_POINT_HEADER >> 8) == pack_buf_union.buf[1]) ){
						recv_pos++;
					}else{
                        recv_pos = 0;
                    }
					break;
				}
				case 3: { 			//data number 
					if((0x55 == pack_buf_union.buf[0]) && 0xAA != pack_buf_union.buf[1]){			//0x55 0xXX down info 
						recv_pos++;
					}else if((0xA5 == pack_buf_union.buf[0]) && 0xAB == pack_buf_union.buf[1]){
						if(0 != byte){
							recv_pos++;
							pack_size = byte + 5;		//packagehead 4 bytes + 1 byte crc 
						}else{
							recv_pos = 0;
						}
					}else if((0x55 == pack_buf_union.buf[0]) && (0xAA == pack_buf_union.buf[1])){
						if( ((PROTOCOL_VP100_NORMAL_NO_QUALITY & 0xFF) == byte) ||
							((PROTOCOL_VP100_NORMAL_QUALITY & 0xFF) == byte) ||
							((PROTOCOL_VP100_YW_QUALITY & 0xFF) == byte) ||
							((PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY & 0xFF) == byte) ||
							((PROTOCOL_VP100_ERROR_FAULT & 0xFF) == byte) ){
								lidar_model_code |= byte;
								//取到当前包字节总长
								pack_size = GET_LIDAR_DATA_SIZE(lidar_model_code);
								recv_pos++;
						}else{
							pack_size = 0;
							recv_pos = 0;
						}
					}else if( ((uint8_t)(NVILIDAR_T10_POINT_HEADER & 0xFF) == pack_buf_union.buf[0]) && 
							  ((uint8_t)(NVILIDAR_T10_POINT_HEADER >> 8) == pack_buf_union.buf[1]) ){
						lidar_model_code = PROTOCOL_T10_MODEL_QUAILIY;   //T10 lidar
						pack_size = GET_LIDAR_DATA_SIZE(lidar_model_code);	//取到当前包字节总长
						recv_pos++;
					}
					break;
                }	
				default:{
					//if can not get the right size,return 
					if(0 == pack_size){
						recv_pos = 0;
						break;
                	}
					//get the value 
					if(pack_size - 1 == recv_pos){
						if((0x55 == pack_buf_union.buf[0]) && (0xAA != pack_buf_union.buf[1])){			//0xA5 0xXX
							LidarInfoAnalysis(&pack_buf_union);		//downboard info 
						}else if((0xA5 == pack_buf_union.buf[0]) && (0xAB == pack_buf_union.buf[1])){	//0xA5 0xAB
							LidarInfoAnalysis(&pack_buf_union);		//upboard info 
						}else if((0x55 == pack_buf_union.buf[0]) && (0xAA == pack_buf_union.buf[1])) {  	//0x55 0xAA
							switch(lidar_model_code){
								case PROTOCOL_VP100_NORMAL_NO_QUALITY:{
									PointCloudAnalysis_Normal_NoQuality(&pack_buf_union);
									break;
								}
								case PROTOCOL_VP100_NORMAL_QUALITY:{
								    PointCloudAnalysis_Normal_Quality(&pack_buf_union);
									break;
								}
								case PROTOCOL_VP100_YW_QUALITY:{
									PointCloudAnalysis_YW_Quality(&pack_buf_union);
									break;
								}
								case PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY:{
								    PointCloudAnalysis_FS_Test_Quality(&pack_buf_union);
									break;
								}
								case PROTOCOL_VP100_ERROR_FAULT:{
									PointCloudAnalysis_ErrorCode(&pack_buf_union);
									break;
								}
								default:{
									break;
								}
							}
						}else if( ((uint8_t)(NVILIDAR_T10_POINT_HEADER & 0xFF) == pack_buf_union.buf[0]) && 
							  ((uint8_t)(NVILIDAR_T10_POINT_HEADER >> 8) == pack_buf_union.buf[1]) ){
							switch(lidar_model_code){
								case PROTOCOL_T10_MODEL_QUAILIY:{
									PointCloudAnalysis_T10_Quality(&pack_buf_union);
									break;
								}
								default:{
									break;
								}
							}		
						}
						//clear the buffer 
						memset(pack_buf_union.buf,0x00,sizeof(VP100_Node_Package_Union));
						recv_pos = 0;
						break;
					}else{
						recv_pos++;
					}
					break;
				}
			}
		}
	}

	//normal no quality 
	void LidarDriverSerialport::PointCloudAnalysis_Normal_NoQuality(VP100_Node_Package_Union *pack){
		uint64_t packageStamp = 0;			//the time stamp 
		//=====crc compare 
		uint16_t crc_get = pack_buf_union.vp100_normal_no_quality.package_checkSum;
		uint16_t crc_calc = LidarCheckCRC(pack_buf_union.buf,sizeof(VP100_Normal_Node_Package_No_Quality)-2);
		if(crc_calc != crc_get) {
			return;
		}

		//=====calculate angle & distance 
		//angle apart 
		double angle_differ = 0.0;

		uint16_t first_angle = pack_buf_union.vp100_normal_no_quality.package_firstSampleAngle - 0xA000;
		uint16_t last_angle =  pack_buf_union.vp100_normal_no_quality.package_lastSampleAngle - 0xA000;
		if(last_angle >= first_angle){      //start angle > end angle 
			angle_differ = ((double)(last_angle - first_angle)/(NORMAL_NO_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}else {
			angle_differ = ((double)(last_angle + (360*64) - first_angle)/(NORMAL_NO_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}
		double first_angle_true = (double)(first_angle)/64.0;

		//distance apart 
    	for(int j = 0; j<NORMAL_NO_QUALITY_PACK_MAX_POINTS; j++) {
			//the node info 
			Nvilidar_Node_Info node;
			//get the angle 
			double cur_angle = first_angle_true + angle_differ*j;
			if(cur_angle >= 360.0){
				cur_angle -= 360.0;
			}
			//get distance  
			uint16_t cur_distance_u16 = pack->vp100_normal_no_quality.package_Sample[j].PakageSampleDistance;
			double cur_distance = 0;
			if((cur_distance_u16 & 0x8000) != 0){
				cur_distance = 0;
			}else {
				cur_distance = (double)(cur_distance_u16);
			}
			//get quality 
			uint16_t cur_quality = 0;
			//speed 
			double cur_speed = (double)(pack_buf_union.vp100_normal_no_quality.package_speed) / 64.0;

			//find circle start 
			if(cur_angle < last_angle_point){
				if(get_timestamp != nullptr){
					packageStamp = get_timestamp();   //get current timestamp
				}
				node.lidar_angle_zero_flag = true;
				curr_pack_count = 0;

			}else{
				node.lidar_angle_zero_flag = false;
				curr_pack_count++;
			}
			last_angle_point = cur_angle;

			//points analysis 
			node.lidar_distance = 	cur_distance;   //distance
			node.lidar_angle =  	cur_angle;		//angle 
			node.lidar_quality = 	cur_quality;	//cur quality 
			node.lidar_speed = 		cur_speed;  	//speed 
			node.lidar_index = 		j;              //index	
			//add to vector  
			if(node.lidar_angle_zero_flag){
				curr_circle_count = node_point_list.size();		//add to vector  
			}
			node_point_list.push_back(node);	

			//data process 
			if(node.lidar_angle_zero_flag){
				circleDataInfo.lidarCircleNodePoints = node_point_list;	//get time stamp 

				circleDataInfo.lidarCircleNodePoints.assign(node_point_list.begin(),node_point_list.begin() + curr_circle_count);		//get pre data 	
				node_point_list.erase(node_point_list.begin(),node_point_list.begin() + curr_circle_count);								//after data 
				curr_circle_count = 0;

				//get time stamp 
				circleDataInfo.startStamp = circleDataInfo.stopStamp;
				circleDataInfo.stopStamp = packageStamp;

				circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;		//normal/no fault 

				if(false == m_first_circle_finish){
					m_first_circle_finish = true;
				}else{
					setCircleResponseUnlock();		//thread unlock 
				}
			}
    	}
	}

	//normal has quality 
	void LidarDriverSerialport::PointCloudAnalysis_Normal_Quality(VP100_Node_Package_Union *pack){
		uint64_t packageStamp = 0;			//the time stamp 
		//=====crc compare 
		uint16_t crc_get = pack_buf_union.vp100_normal_quality.package_checkSum;
		uint16_t crc_calc = LidarCheckCRC(pack_buf_union.buf,sizeof(VP100_Normal_Node_Package_Quality)-2);
		if(crc_calc != crc_get) {
			return;
		}

		//=====calculate angle & distance 
		//angle apart 
		double angle_differ = 0.0;

		uint16_t first_angle = pack_buf_union.vp100_normal_quality.package_firstSampleAngle - 0xA000;
		uint16_t last_angle =  pack_buf_union.vp100_normal_quality.package_lastSampleAngle - 0xA000;
		if(last_angle >= first_angle){      //start angle > end angle 
			angle_differ = ((double)(last_angle - first_angle)/(NORMAL_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}else {
			angle_differ = ((double)(last_angle + (360*64) - first_angle)/(NORMAL_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}
		double first_angle_true = (double)(first_angle)/64.0;

		//distance apart 
    	for(int j = 0; j<NORMAL_HAS_QUALITY_PACK_MAX_POINTS; j++) {
			//the node info 
			Nvilidar_Node_Info node;
			//get the angle 
			double cur_angle = first_angle_true + angle_differ*j;
			if(cur_angle >= 360.0){
				cur_angle -= 360.0;
			}
			//get distance  
			uint16_t cur_distance_u16 = pack->vp100_normal_quality.package_Sample[j].PakageSampleDistance;
			double cur_distance = 0;
			if((cur_distance_u16 & 0x8000) != 0){
				cur_distance = 0;
			}else {
				cur_distance = (double)(cur_distance_u16);
			}
			//get quality 
			uint16_t cur_quality = pack_buf_union.vp100_normal_quality.package_Sample[j].PakageSampleQuality;
			//speed 
			double cur_speed = (double)(pack_buf_union.vp100_normal_quality.package_speed) / 64.0;

			//find circle start 
			if(cur_angle < last_angle_point){
				if(get_timestamp != nullptr){
					packageStamp =  get_timestamp();
				}
				node.lidar_angle_zero_flag = true;
				curr_pack_count = 0;

			}else{
				node.lidar_angle_zero_flag = false;
				curr_pack_count++;
			}
			last_angle_point = cur_angle;

			//points analysis 
			node.lidar_distance = 	cur_distance;   //distance
			node.lidar_angle =  	cur_angle;		//angle 
			node.lidar_quality = 	cur_quality;	//cur quality 
			node.lidar_speed = 		cur_speed;  	//speed 
			node.lidar_index = 		j;              //index	
			//add to vector  
			if(node.lidar_angle_zero_flag){
				curr_circle_count = node_point_list.size();		//add to vector  
			}
			node_point_list.push_back(node);	

			//data process 
			if(node.lidar_angle_zero_flag){
				circleDataInfo.lidarCircleNodePoints = node_point_list;	//get time stamp 

				circleDataInfo.lidarCircleNodePoints.assign(node_point_list.begin(),node_point_list.begin() + curr_circle_count);		//get pre data 	
				node_point_list.erase(node_point_list.begin(),node_point_list.begin() + curr_circle_count);								//after data 
				curr_circle_count = 0;

				//get time stamp 
				circleDataInfo.startStamp = circleDataInfo.stopStamp;
				circleDataInfo.stopStamp = packageStamp;

				circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;		//normal/no fault 

				if(false == m_first_circle_finish){
					m_first_circle_finish = true;
				}else{
					setCircleResponseUnlock();		//thread unlock 
				}
			}
    	}
	}

	//yw has quality 
	void LidarDriverSerialport::PointCloudAnalysis_YW_Quality(VP100_Node_Package_Union *pack){
		uint64_t packageStamp = 0;			//the time stamp 
		//=====crc compare 
		uint16_t crc_get = pack_buf_union.vp100_yw_quality.package_checkSum;
		uint16_t crc_calc = LidarCheckCRC(pack_buf_union.buf,sizeof(VP100_YW_Node_Package_Quality)-2);
		if(crc_calc != crc_get) {
			return;
		}

		//=====calculate angle & distance 
		//angle apart 
		double angle_differ = 0.0;

		uint16_t first_angle = pack_buf_union.vp100_yw_quality.package_firstSampleAngle - 0xA000;
		uint16_t last_angle =  pack_buf_union.vp100_yw_quality.package_lastSampleAngle - 0xA000;
		if(last_angle >= first_angle){      //start angle > end angle 
			angle_differ = ((double)(last_angle - first_angle)/(YW_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}else {
			angle_differ = ((double)(last_angle + (360*64) - first_angle)/(YW_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}
		double first_angle_true = (double)(first_angle)/64.0;

		//distance apart 
    	for(int j = 0; j<YW_HAS_QUALITY_PACK_MAX_POINTS; j++) {
			//the node info 
			Nvilidar_Node_Info node;
			//get the angle 
			double cur_angle = first_angle_true + angle_differ*j;
			if(cur_angle >= 360.0){
				cur_angle -= 360.0;
			}
			//get distance  
			uint16_t cur_distance_u16 = pack->vp100_yw_quality.package_Sample[j].PakageSampleDistance;
			double cur_distance = 0;
			if((cur_distance_u16 & 0x8000) != 0){
				cur_distance = 0;
			}else {
				cur_distance = (double)(cur_distance_u16);
			}
			//get quality 
			uint16_t cur_quality = pack->vp100_yw_quality.package_Sample[j].PakageSampleQuality / 48;
			//speed 
			double cur_speed = (double)(pack_buf_union.vp100_yw_quality.package_speed) / 64.0;

			//find circle start 
			if(cur_angle < last_angle_point){
				if(get_timestamp != nullptr){
					packageStamp = get_timestamp();
				}
				node.lidar_angle_zero_flag = true;
				curr_pack_count = 0;

			}else{
				node.lidar_angle_zero_flag = false;
				curr_pack_count++;
			}
			last_angle_point = cur_angle;

			//points analysis 
			node.lidar_distance = 	cur_distance;   //distance
			node.lidar_angle =  	cur_angle;		//angle 
			node.lidar_quality = 	cur_quality;	//cur quality 
			node.lidar_speed = 		cur_speed;  	//speed 
			node.lidar_index = 		j;              //index	
			//add to vector  
			if(node.lidar_angle_zero_flag){
				curr_circle_count = node_point_list.size();		//add to vector  
			}
			node_point_list.push_back(node);	

			//data process 
			if(node.lidar_angle_zero_flag){
				circleDataInfo.lidarCircleNodePoints = node_point_list;	//get time stamp 

				circleDataInfo.lidarCircleNodePoints.assign(node_point_list.begin(),node_point_list.begin() + curr_circle_count);		//get pre data 	
				node_point_list.erase(node_point_list.begin(),node_point_list.begin() + curr_circle_count);								//after data 
				curr_circle_count = 0;

				//get time stamp 
				circleDataInfo.startStamp = circleDataInfo.stopStamp;
				circleDataInfo.stopStamp = packageStamp;

				circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;		//normal/no fault 

				if(false == m_first_circle_finish){
					m_first_circle_finish = true;
				}else{
					setCircleResponseUnlock();		//thread unlock 
				}
			}
    	}
	}

	//FS has quality 
	void LidarDriverSerialport::PointCloudAnalysis_FS_Test_Quality(VP100_Node_Package_Union *pack){
		uint64_t packageStamp = 0;			//the time stamp 
		//=====crc compare 
		uint16_t crc_get = pack_buf_union.vp100_fs_test_quality.package_checkSum;
		uint16_t crc_calc = LidarCheckCRC(pack_buf_union.buf,sizeof(VP100_FS_Test_Node_Package_Quality)-2);
		if(crc_calc != crc_get) {
			return;
		}

		//=====calculate angle & distance 
		//angle apart 
		double angle_differ = 0.0;

		uint16_t first_angle = pack_buf_union.vp100_fs_test_quality.package_firstSampleAngle - 0xA000;
		uint16_t last_angle =  pack_buf_union.vp100_fs_test_quality.package_lastSampleAngle - 0xA000;
		if(last_angle >= first_angle){      //start angle > end angle 
			angle_differ = ((double)(last_angle - first_angle)/(FS_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}else {
			angle_differ = ((double)(last_angle + (360*64) - first_angle)/(FS_HAS_QUALITY_PACK_MAX_POINTS - 1))/64.0;
		}
		double first_angle_true = (double)(first_angle)/64.0;

		//distance apart 
    	for(int j = 0; j<FS_HAS_QUALITY_PACK_MAX_POINTS; j++) {
			//the node info 
			Nvilidar_Node_Info node;
			//get the angle 
			double cur_angle = first_angle_true + angle_differ*j;
			if(cur_angle >= 360.0){
				cur_angle -= 360.0;
			}
			//get distance  
			uint16_t cur_distance_u16 = pack->vp100_fs_test_quality.package_Sample[j].PakageSampleDistance;
			double cur_distance = 0;
			if((cur_distance_u16 & 0x8000) != 0){
				cur_distance = 0;
			}else {
				cur_distance = (double)(cur_distance_u16);
			}
			//get quality 
			uint16_t cur_quality = pack->vp100_fs_test_quality.package_Sample[j].PakageSampleQuality;
			//speed 
			double cur_speed = (double)(pack_buf_union.vp100_fs_test_quality.package_speed) / 64.0;

			//find circle start 
			if(cur_angle < last_angle_point){
				if(get_timestamp != nullptr){
					packageStamp = get_timestamp();
				}
				node.lidar_angle_zero_flag = true;
				curr_pack_count = 0;

			}else{
				node.lidar_angle_zero_flag = false;
				curr_pack_count++;
			}
			last_angle_point = cur_angle;

			//points analysis 
			node.lidar_distance = 	cur_distance;   //distance
			node.lidar_angle =  	cur_angle;		//angle 
			node.lidar_quality = 	cur_quality;	//cur quality 
			node.lidar_speed = 		cur_speed;  	//speed 
			node.lidar_index = 		j;              //index	
			//add to vector  
			if(node.lidar_angle_zero_flag){
				curr_circle_count = node_point_list.size();		//add to vector  
			}
			node_point_list.push_back(node);	

			//data process 
			if(node.lidar_angle_zero_flag){
				circleDataInfo.lidarCircleNodePoints = node_point_list;	//get time stamp 

				circleDataInfo.lidarCircleNodePoints.assign(node_point_list.begin(),node_point_list.begin() + curr_circle_count);		//get pre data 	
				node_point_list.erase(node_point_list.begin(),node_point_list.begin() + curr_circle_count);								//after data 
				curr_circle_count = 0;

				//get time stamp 
				circleDataInfo.startStamp = circleDataInfo.stopStamp;
				circleDataInfo.stopStamp = packageStamp;

				circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;		//normal/no fault 

				if(false == m_first_circle_finish){
					m_first_circle_finish = true;
				}else{
					setCircleResponseUnlock();		//thread unlock 
				}
			}
    	}
	}

	//T10 has quality 
	void LidarDriverSerialport::PointCloudAnalysis_T10_Quality(VP100_Node_Package_Union *pack){
		uint64_t packageStamp = 0;			//the time stamp 
		//=====crc compare 
		uint8_t crc_get = pack_buf_union.vp100_t10_quality.package_checkSum;
		uint16_t crc_calc = T10CheckCrc(pack_buf_union.buf,sizeof(VP100_T10_Node_Package_Quality)-1);
		if(crc_calc != crc_get) {
			return;
		}

		//=====calculate angle & distance 
		//angle apart 
		double angle_differ = 0.0;

		uint16_t first_angle = pack_buf_union.vp100_t10_quality.package_firstSampleAngle;
		uint16_t last_angle =  pack_buf_union.vp100_t10_quality.package_lastSampleAngle;
		if(last_angle >= first_angle){      //start angle > end angle 
			angle_differ = ((double)(last_angle - first_angle)/(T10_HAS_QUALITY_PACK_MAX_POINTS - 1))/100.0;
		}else {
			angle_differ = ((double)(last_angle + (360*100) - first_angle)/(T10_HAS_QUALITY_PACK_MAX_POINTS - 1))/100.0;
		}
		double first_angle_true = (double)(first_angle)/100.0;

		//distance apart 
    	for(int j = 0; j<T10_HAS_QUALITY_PACK_MAX_POINTS; j++) {
			//the node info 
			Nvilidar_Node_Info node;
			//get the angle 
			double cur_angle = first_angle_true + angle_differ*j;
			if(cur_angle >= 360.0){
				cur_angle -= 360.0;
			}
			//get distance  
			uint16_t cur_distance_u16 = pack->vp100_t10_quality.package_Sample[j].PakageSampleDistance;
			double cur_distance = (double)(cur_distance_u16);
			//get quality 
			uint8_t cur_quality = pack->vp100_t10_quality.package_Sample[j].PakageSampleQuality;
			//speed 
			double cur_speed = (double)(pack_buf_union.vp100_t10_quality.package_speed);

			//find circle start 
			if(cur_angle < last_angle_point){
				if(get_timestamp != nullptr){
					packageStamp = get_timestamp();
				}
				node.lidar_angle_zero_flag = true;
				curr_pack_count = 0;

			}else{
				node.lidar_angle_zero_flag = false;
				curr_pack_count++;
			}
			last_angle_point = cur_angle;

			//points analysis 
			node.lidar_distance = 	cur_distance;   //distance
			node.lidar_angle =  	cur_angle;		//angle 
			node.lidar_quality = 	cur_quality;	//cur quality 
			node.lidar_speed = 		cur_speed;  	//speed 
			node.lidar_index = 		j;              //index	
			//add to vector  
			if(node.lidar_angle_zero_flag){
				curr_circle_count = node_point_list.size();		//add to vector  
			}
			node_point_list.push_back(node);	

			//data process 
			if(node.lidar_angle_zero_flag){
				circleDataInfo.lidarCircleNodePoints = node_point_list;	//get time stamp 

				circleDataInfo.lidarCircleNodePoints.assign(node_point_list.begin(),node_point_list.begin() + curr_circle_count);		//get pre data 	
				node_point_list.erase(node_point_list.begin(),node_point_list.begin() + curr_circle_count);								//after data 
				curr_circle_count = 0;

				//get time stamp 
				circleDataInfo.startStamp = circleDataInfo.stopStamp;
				circleDataInfo.stopStamp = packageStamp;

				circleDataInfo.error_code = VP100_ERROR_CODE_NORMAL;		//normal/no fault 

				if(false == m_first_circle_finish){
					m_first_circle_finish = true;
				}else{
					setCircleResponseUnlock();		//thread unlock 
				}
			}
    	}
	}

	//Error Code 
	void LidarDriverSerialport::PointCloudAnalysis_ErrorCode(VP100_Node_Package_Union *pack_buf_union){
		uint8_t add_sum_value = 0;
		// calc the add sum 
		add_sum_value = checkAddSum(pack_buf_union->buf,sizeof(VP100_Error_Fault_TypeDef));
		if(add_sum_value != pack_buf_union->buf[sizeof(VP100_Error_Fault_TypeDef) - 1]){
			return;
		}
		// get the error infomation 
		circleDataInfo.error_code = (VP100Lidar_ErrorFlagEnumTypeDef)pack_buf_union->vp100_error_fault_info.package_errorCode;
		// printf("error code:%d\n",pack_buf_union->vp100_error_fault_info.package_errorCode);

		setCircleResponseUnlock();		//thread unlock 
	}

	//lidar version info get  
	void LidarDriverSerialport::LidarInfoAnalysis(VP100_Node_Package_Union *pack_buf_union){
		uint8_t add_sum_value = 0;
		if((0x55 == pack_buf_union->buf[0]) && (0xAA != pack_buf_union->buf[1])){     //0x55 0xXX downboard info 
			//calc the checksum 
			add_sum_value = checkAddSum(pack_buf_union->buf,pack_buf_union->vp100_lidar_info_downboard.package_length + 4);
			if(add_sum_value != pack_buf_union->buf[pack_buf_union->vp100_lidar_info_downboard.package_length + 3]){
				return;
			}
			//unpack 
			switch (pack_buf_union->vp100_lidar_info_downboard.package_cmd) {
				case 0xAB:{
					std::string str;
					vp100_lidar_info.downBoard_Model = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																	pack_buf_union->vp100_lidar_info_downboard.package_length);
	              	// printf("%s\n",vp100_lidar_info.downBoard_Model.c_str());
					break;
				}
				case 0xAC:{
					std::string str;
					vp100_lidar_info.downBoard_HardVersion = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																		pack_buf_union->vp100_lidar_info_downboard.package_length);
	                // printf(vp100_lidar_info.downBoard_HardVersion.c_str());
					break;
				}
				case 0xAD:{
					std::string str;
					vp100_lidar_info.downBoard_SoftVersion = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																		pack_buf_union->vp100_lidar_info_downboard.package_length);
					// printf(vp100_lidar_info.downBoard_SoftVersion);
					break;
				}
				case 0xAE:{
					vp100_lidar_info.downBoard_ID = hexBytesToString((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																		pack_buf_union->vp100_lidar_info_downboard.package_length);
	                // printf(vp100_lidar_info.downBoard_ID);
					break;
				}
				case 0xAF:{
					std::string str;
					vp100_lidar_info.downBoard_Date = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																 pack_buf_union->vp100_lidar_info_downboard.package_length);
	                // printf(vp100_lidar_info.downBoard_Date);
					break;
				}
				case 0xB0:{
					std::string str;
					vp100_lidar_info.downBoard_InputVoltage = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																			pack_buf_union->vp100_lidar_info_downboard.package_length);
					vp100_lidar_info.downBoard_InputVoltage.insert(1,1,'.');
	                // printf(vp100_lidar_info.downBoard_InputVoltage);
					break;
				}
				case 0xB6:{
					vp100_lidar_info.downBoard_ID = hexBytesToString((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																		pack_buf_union->vp100_lidar_info_downboard.package_length);
	             	// printf(vp100_lidar_info.downBoard_UID);
					break;
				}
				case 0xBA:{
					vp100_lidar_info.upBoard_VBD = pack_buf_union->vp100_lidar_info_downboard.package_data[0];
					vp100_lidar_info.upBoard_TDC = pack_buf_union->vp100_lidar_info_downboard.package_data[1];
					vp100_lidar_info.upBoard_Temperature = pack_buf_union->vp100_lidar_info_downboard.package_data[2];
					// printf("vbd:%d,tdc:%d,temp:%d\n",vp100_lidar_info.upBoard_VBD,vp100_lidar_info.upBoard_TDC,vp100_lidar_info.upBoard_Temperature);

					//unlock for the response 
					protocol_receive_flag = true;
					setNormalResponseUnlock();
					break;
				}
				case 0xB7:{
					std::string str;
					vp100_lidar_info.downBoard_BuildTime = str.assign((char *)pack_buf_union->vp100_lidar_info_downboard.package_data,
																		pack_buf_union->vp100_lidar_info_downboard.package_length);
					//  printf(vp100_lidar_info.downBoard_BuildTime);
					break;
				}
				default:{
					break;
				}
			}
		}else if((0xA5 == pack_buf_union->buf[0]) && (0xAB == pack_buf_union->buf[1])){  //0xA5 0xAB upboard info 
			//calc the checksum 
			add_sum_value = checkAddSum(pack_buf_union->buf,pack_buf_union->vp100_lidar_info_upboard.package_length + 5);
			if(add_sum_value != pack_buf_union->buf[pack_buf_union->vp100_lidar_info_upboard.package_length + 4]){
				return;
			}
			//unpack 
			switch (pack_buf_union->vp100_lidar_info_upboard.package_cmd) {
				case 0x13:{
					vp100_lidar_info.upBoard_ID = hexBytesToString((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																		pack_buf_union->vp100_lidar_info_upboard.package_length);
	            	// printf(vp100_lidar_info.upBoard_ID);
					break;
				}
				case 0x14:{
					std::string str;
					vp100_lidar_info.upBoard_Date = str.assign((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_Date);
					break;
				}
				case 0x16:{
					std::string str;
					vp100_lidar_info.upBoard_Model = str.assign((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																	pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_Model);
					break;
				}
				case 0x17:{
					std::string str;
					vp100_lidar_info.upBoard_HardVersion = str.assign((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																		pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_HardVersion);
					break;
				}
				case 0x18:{
					std::string str;
					vp100_lidar_info.upBoard_SoftVersion = str.assign((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																		pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_SoftVersion);
					break;
				}
				case 0x19:{
					hexBytesToString((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
										pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_UID);

					//last pack 
					//unlock for the response 
					//setNormalResponseUnlock();
					break;
				}
				case 0x1A:{
					std::string str;
					vp100_lidar_info.upBoard_BuildTime = str.assign((char *)pack_buf_union->vp100_lidar_info_upboard.package_data,
																		pack_buf_union->vp100_lidar_info_upboard.package_length);
					// printf(vp100_lidar_info.upBoard_BuildTime);
					break;
				}
				default:{
					break;
				}
			}
		}
	}


	//-------------------------------------对外接口信息-------------------------------------------

	//获取SDK版本号
	std::string LidarDriverSerialport::getSDKVersion()
	{
		return NVILIDAR_SDKVerision;
	}

	//获取串口列表信息
	std::vector<NvilidarSerialPortInfo> LidarDriverSerialport::getPortList()
	{
		std::vector<NvilidarSerialPortInfo> nvi_serial_list;
		NvilidarSerialPortInfo nvi_serial;

		nvi_serial_list.clear();

	   std::vector<NvilidarPortInfo> lst = nvilidar_list_ports();

		#if defined (_WIN32)
			for (std::vector<NvilidarPortInfo>::iterator it = lst.begin(); it != lst.end(); it++)
			{
				nvi_serial.portName = (*it).port;
				nvi_serial.description = (*it).description;

				nvi_serial_list.push_back(nvi_serial);
			}
		#else
			for (std::vector<NvilidarPortInfo>::iterator it = lst.begin(); it != lst.end(); it++)
			{
				//printf("port:%s,des:%s\n",(*it).port.c_str(),(*it).description.c_str());
				//usb check
				if ((*it).port.find("ttyACM") != std::string::npos)      //linux ttyacm
				{
					nvi_serial.portName = (*it).port;
					nvi_serial.description = (*it).description;

					nvi_serial_list.push_back(nvi_serial);
				}
			}
		#endif

		return nvi_serial_list;
	}

	#if NVILIDAR_COMMUNICATE_TWO_WAY
		//获取设备类型信息
		bool LidarDriverSerialport::GetDeviceInfo(Nvilidar_DeviceInfo &info, uint32_t timeout)
		{
			recv_info.recvFinishFlag = false;

			//先停止雷达 如果雷达在运行 
			if(lidar_state.m_Scanning)
			{
				StopScan();
			}

			//发送命令
			if (!SendCommand(NVILIDAR_CMD_GET_DEVICE_INFO))
			{
				return false;
			}
		
			//等待线程同步 超时 
			if (waitNormalResponse(timeout))
			{
				if(recv_info.recvFinishFlag)
				{
					uint8_t productNameTemp[6] = { 0 };
					memcpy(productNameTemp, recv_info.lidar_device_info.MODEL_NUM,5);

					//生成字符信息
					info.m_SoftVer = formatString("V%d.%d", recv_info.lidar_device_info.SW_V[0], recv_info.lidar_device_info.SW_V[1]);
					info.m_HardVer = formatString("V%d.%d", recv_info.lidar_device_info.HW_V[0], recv_info.lidar_device_info.HW_V[1]);
					info.m_ProductName = formatString("%s", productNameTemp);
					info.m_SerialNum = formatString("%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d",
						recv_info.lidar_device_info.serialnum[0], recv_info.lidar_device_info.serialnum[1], recv_info.lidar_device_info.serialnum[2], recv_info.lidar_device_info.serialnum[3],
						recv_info.lidar_device_info.serialnum[4], recv_info.lidar_device_info.serialnum[5], recv_info.lidar_device_info.serialnum[6], recv_info.lidar_device_info.serialnum[7],
						recv_info.lidar_device_info.serialnum[8], recv_info.lidar_device_info.serialnum[9], recv_info.lidar_device_info.serialnum[10], recv_info.lidar_device_info.serialnum[11],
						recv_info.lidar_device_info.serialnum[12], recv_info.lidar_device_info.serialnum[13], recv_info.lidar_device_info.serialnum[14], recv_info.lidar_device_info.serialnum[15]);


					return true;
				}
			}

			return false;
		}
	#endif 


	//---------------------------------------------thread API----------------------------------------------
	//init thread 
	bool LidarDriverSerialport::createThread()
	{
		#if	defined(_WIN32)
			/*
			创建线程函数详细说明
			*HANDLE WINAPI CreateThread(
			__in_opt  LPSECURITY_ATTRIBUTES lpThreadAttributes,
			__in      SIZE_T dwStackSize,
			__in      LPTHREAD_START_ROUTINE lpStartAddress,
			__in_opt __deref __drv_aliasesMem LPVOID lpParameter,
			__in      DWORD dwCreationFlags,
			__out_opt LPDWORD lpThreadId
			);
			*返回值：函数成功，返回线程句柄；函数失败返回false。若不想返回线程ID,设置值为NULL
			*参数说明：
			*lpThreadAttributes	线程安全性，使用缺省安全性,一般缺省null
			*dwStackSize	堆栈大小，0为缺省大小
			*lpStartAddress	线程要执行的函数指针，即入口函数
			*lpParameter	线程参数
			*dwCreationFlags	线程标记，如为0，则创建后立即运行
			*lpThreadId	LPDWORD为返回值类型，一般传递地址去接收线程的标识符，一般设为null
			*/
			_thread = CreateThread(NULL, 0, LidarDriverSerialport::periodThread, this, 0, NULL);
			if (_thread == NULL)
			{
				return false;
			}



			/*
			*创建事件对象
			HANDLE WINAPI CreateEventA(
			__in_opt LPSECURITY_ATTRIBUTES lpEventAttributes,
			__in     BOOL bManualReset,
			__in     BOOL bInitialState,
			__in_opt LPCSTR lpName
			);
			*返回值：如果函数调用成功，函数返回事件对象的句柄。如果对于命名的对象，在函数调用前已经被创建，函数将返回存在的事件对象的句柄，而且在GetLastError函数中返回ERROR_ALREADY_EXISTS。
			如果函数失败，函数返回值为NULL，如果需要获得详细的错误信息，需要调用GetLastError。
			*参数说明：
			*lpEventAttributes	安全性，采用null默认安全性。
			*bManualReset	（TRUE）人工重置或（FALSE）自动重置事件对象为非信号状态，若设为人工重置，则当事件为有信号状态时，所有等待的线程都变为可调度线程。
			*bInitialState		指定事件对象的初始化状态，TRUE：初始为有信号状态。
			*lpName	事件对象的名字，一般null匿名即可
			*/
			_event_analysis = CreateEvent(NULL, false, false, NULL);;
			if (_event_analysis == NULL)
			{
				return false;
			}
			ResetEvent(_event_analysis);

			//一圈点的数据信息 信息同步 
			_event_circle = CreateEvent(NULL, false, false, NULL);;
			if (_event_circle == NULL)
			{
				return false;
			}
			ResetEvent(_event_circle);

			return true;
		#else 
			//sync connect  
			pthread_cond_init(&_cond_analysis, NULL);
    		pthread_mutex_init(&_mutex_analysis, NULL);
			pthread_cond_init(&_cond_point, NULL);
    		pthread_mutex_init(&_mutex_point, NULL);

			//create thread 
     		if(-1 == pthread_create(&_thread, NULL, LidarDriverSerialport::periodThread, this)){
				 _thread = -1;
         		return false;
			}

			return true;

		#endif 
	}

	//close thread 
	void LidarDriverSerialport::closeThread()
	{
		#if	defined(_WIN32)
			CloseHandle(_thread);
			CloseHandle(_event_analysis);
			CloseHandle(_event_circle);
		#else 
			pthread_cancel(_thread);
			pthread_cond_destroy(&_cond_analysis);
			pthread_mutex_destroy(&_mutex_analysis);
			pthread_cond_destroy(&_cond_point);
			pthread_mutex_destroy(&_mutex_point);
		#endif 
	}

	//wait for response 
	bool LidarDriverSerialport::waitNormalResponse(uint32_t timeout)
	{
		#if	defined(_WIN32)
			DWORD state;
			ResetEvent(_event_analysis);		// 重置事件，让其他线程继续等待（相当于获取锁）
			state = WaitForSingleObject(_event_analysis, timeout);
			if(state == WAIT_OBJECT_0)
			{
				return true;
			}
		#else 
			struct timeval now;
    		struct timespec outtime;
			int state = -1;

			pthread_mutex_lock(&_mutex_analysis);
 
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + timeout / 1000;
			outtime.tv_nsec = now.tv_usec * 1000 + timeout%1000*1000;
		
			state = pthread_cond_timedwait(&_cond_analysis, &_mutex_analysis, &outtime);
			pthread_mutex_unlock(&_mutex_analysis);

			if(0 == state){
				return true;
			}

		#endif

		return false;
	}

	//wait for event unlock 
	void LidarDriverSerialport::setNormalResponseUnlock()
	{
		#if	defined(_WIN32)
			SetEvent(_event_analysis);			// wait the thread
		#else 
			pthread_mutex_lock(&_mutex_analysis);
    		pthread_cond_signal(&_cond_analysis);
    		pthread_mutex_unlock(&_mutex_analysis);
		#endif 
	}

	//wait for circle unlock 
	bool LidarDriverSerialport::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	{
		//wait for unlock 
		#if	defined(_WIN32)
			DWORD state;
			ResetEvent(_event_circle);		// wait the thread
			state = WaitForSingleObject(_event_circle, timeout);
			if (state == WAIT_OBJECT_0)
			{
				//点集格式转换 
				LidarSamplingData(circleDataInfo, scan);

				return true;
			}	
		#else 
			struct timeval now;
    		struct timespec outtime;
			int state = -1;

			pthread_mutex_lock(&_mutex_point);
 
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + timeout / 1000;
			outtime.tv_nsec = now.tv_usec * 1000 + timeout%1000*1000;
		
			state = pthread_cond_timedwait(&_cond_point, &_mutex_point, &outtime);
			pthread_mutex_unlock(&_mutex_point);

			if(0 == state){
				//data change 
				LidarSamplingData(circleDataInfo, scan);

				return true;
			}
		#endif

		return false;
	}
	
	//data analysis   
	void LidarDriverSerialport::LidarSamplingData(CircleDataInfoTypeDef info, LidarScan &outscan){
		uint32_t all_nodes_counts = 0;		//all point 
		uint64_t scan_time = 0;				//time spice  
		uint64_t point_stamp = 0;			//point gap time 

		//scan time  
		scan_time = info.stopStamp - info.startStamp;

		//origin data count 
		uint32_t lidar_ori_count = info.lidarCircleNodePoints.size();

		//fixed rate  
		if (lidar_cfg.resolution_fixed){
			all_nodes_counts = (uint32_t)(lidar_cfg.sampling_rate * 1000 / lidar_cfg.aim_speed);
		}
		else{ 	//true points 
			all_nodes_counts = lidar_ori_count;
		}

		//min and max angle 
		if (lidar_cfg.angle_max < lidar_cfg.angle_min){
			float temp = lidar_cfg.angle_min;
			lidar_cfg.angle_min = lidar_cfg.angle_max;
			lidar_cfg.angle_max = temp;
		}

		//output data 
		outscan.stamp_start = info.startStamp;
		outscan.stamp_stop = info.stopStamp;
		if((all_nodes_counts > 1) && (info.startStamp > 0)){
			info.differStamp = (info.stopStamp - info.startStamp)/(all_nodes_counts - 1);   //get the differ value 
		}else{
			info.differStamp = 0;
		}
		outscan.stamp_differ = info.differStamp;

		outscan.config.max_angle = lidar_cfg.angle_max*M_PI / 180.0;			//max angle 				
		outscan.config.min_angle = lidar_cfg.angle_min*M_PI / 180.0;			//min angle   
		outscan.config.angle_increment = 2.0 * M_PI/(double)(all_nodes_counts - 1);	//2 points increment 
		if(time_unit > 0){
			outscan.config.scan_time = static_cast<float>(1.0 * scan_time / time_unit);  	//scan time  
		}else{
			outscan.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);
		}
		outscan.config.time_increment = outscan.config.scan_time / (double)(all_nodes_counts - 1); 	//2 point time increment
		outscan.config.min_range = lidar_cfg.range_min;
		outscan.config.max_range = lidar_cfg.range_max;

		//init var   
		float dist = 0.0;
		float angle = 0.0;
		float intensity = 0.0;
		unsigned int i = 0;
		outscan.points.clear();		//clear vector 

		//get the data from raw data 
		for (; i < lidar_ori_count; i++) {
			dist = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_distance / 1000.f);
			intensity = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_quality);
			angle = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_angle);
			angle = angle * M_PI / 180.0;
			point_stamp = static_cast<uint64_t>(info.startStamp + (i*info.differStamp));

			//Rotate 180 degrees or not
			if (lidar_cfg.reversion){
				angle = angle + M_PI;
			}
			//Is it counter clockwise
			if (!lidar_cfg.inverted){
				angle = 2 * M_PI - angle;
			}

			//ignore points 
			if (lidar_cfg.ignore_array.size() != 0){
				for (uint16_t j = 0; j < lidar_cfg.ignore_array.size(); j = j + 2){
					double angle_start = lidar_cfg.ignore_array[j] * M_PI / 180.0;
					double angle_end = lidar_cfg.ignore_array[j + 1] * M_PI / 180.0;

					if ((angle_start <= angle) && (angle <= angle_end))
					{
						dist = 0.0;
						intensity = 0.0; 

						break;
					}
				}
			}

			//-pi ~ pi
			angle = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
			if (angle > M_PI){
				angle -= 2.0 * M_PI;
			}

			//distance valid? 
			if (dist > lidar_cfg.range_max || dist < lidar_cfg.range_min){
				dist = 0.0;
				intensity = 0.0;
			}

			//angle valid? 
			if ((angle >= outscan.config.min_angle) &&
				(angle <= outscan.config.max_angle)){
				NviLidarPoint point;
				point.angle = angle;
				point.range = dist;
				point.intensity = intensity;
				point.stamp = point_stamp;

				outscan.points.push_back(point);
			}
		}
		//fill 
		if (lidar_cfg.resolution_fixed){
			int output_count = all_nodes_counts * ((outscan.config.max_angle - outscan.config.min_angle) / M_PI / 2);
			outscan.points.resize(output_count);
		}
		//error code 
		outscan.error_code = info.error_code;
		if(outscan.error_code != VP100_ERROR_CODE_NORMAL){
			outscan.points.clear();
		}
	}

	//wait for a circle data  
	void LidarDriverSerialport::setCircleResponseUnlock() {
		#if	defined(_WIN32)
			SetEvent(_event_circle);			// get lock 
		#else 
			pthread_mutex_lock(&_mutex_point);
    		pthread_cond_signal(&_cond_point);
    		pthread_mutex_unlock(&_mutex_point);
		#endif 
	}

	//thread (linux & windows )
	#if	defined(_WIN32)
		DWORD WINAPI  LidarDriverSerialport::periodThread(LPVOID lpParameter)
		{
			size_t recv_len = 0;

			//在线程中要做的事情
			LidarDriverSerialport *pObj = (LidarDriverSerialport *)lpParameter;   //传入的参数转化为类对象指针

			while (pObj->lidar_state.m_CommOpen){	
				//解包处理 ==== 点云解包 
				//读串口接收数据长度 
				recv_len = pObj->serialport.serialReadData(pObj->recv_data, 8192);
				if ((recv_len > 0) && (recv_len <= 8192)){
					pObj->PointDataUnpack(pObj->recv_data, recv_len);
				}

				vp100_lidar::TimeStamp::sleepMS(1);		//必须要加sleep 不然会超高占用cpu	
			}

			return 0;
		}
	#else 
		/* 定义线程pthread */
	   	void * LidarDriverSerialport::periodThread(void *lpParameter)       
		{
			size_t recv_len = 0;

			//在线程中要做的事情
			LidarDriverSerialport *pObj = (LidarDriverSerialport *)lpParameter;   //传入的参数转化为类对象指针

			while (pObj->lidar_state.m_CommOpen){	
				//解包处理 ==== 点云解包 
				//读串口接收数据长度 
				recv_len = pObj->serialport.serialReadData(pObj->recv_data, 8192);
				if((recv_len > 0) && (recv_len <= 8192)){
					pObj->PointDataUnpack(pObj->recv_data, recv_len);
				}

				vp100_lidar::TimeStamp::sleepMS(1);		//必须要加sleep 不然会超高占用cpu	
			}

			return 0;
		}
	#endif 

	
}





