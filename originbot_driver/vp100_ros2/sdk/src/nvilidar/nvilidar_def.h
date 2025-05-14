#ifndef _NVILIDAR_DEF_H_
#define _NVILIDAR_DEF_H_

#include <stdint.h>
#include "nvilidar_protocol.h"
#include <string>


//======================================basic parameter============================================ 

//SDK version 
#define NVILIDAR_SDKVerision     "1.0.3"

//PI def
#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif 

//other 
#define NVILIDAR_DEFAULT_TIMEOUT     	2000    //default timeout 
#define NVILIDAR_POINT_TIMEOUT		 	2000	 //one circle time  for example, the lidar speed is 10hz ,the timeout must smaller the 100ms
 
#define NVILIDAR_COMMUNICATE_TWO_WAY	0

//lidar model  list 
typedef enum
{
	NVILIDAR_Unknow = 0,		//unknow lidar 
   	NVILIDAR_VP100,				//lidar VP100
   	NVILIDAR_Tail,
}LidarModelListEnumTypeDef;

//lidar error flag 
typedef enum{
	VP100_ERROR_CODE_RESET = 0,			//lidar reset...
	VP100_ERROR_CODE_MOTOR_LOCK  = 1,	//motor not run 
	VP100_ERROR_CODE_UP_NO_POINT = 2,	//upboard no data 
	VP100_ERROR_CODE_NORMAL = 0xFF,		//normal 
}VP100Lidar_ErrorFlagEnumTypeDef;

//======================================other parameters============================================ 

//lidar current state 
typedef struct
{
	bool m_CommOpen;              	//serialport open flag 
	//bool m_Scanning;                //lidar is scanning data 
	uint8_t last_device_byte;       //last byte 
}Nvilidar_PackageStateTypeDef;

//lidar configure para
typedef struct
{
	std::string frame_id;				//ID
	std::string serialport_name;		//serialport name 
	int    		serialport_baud;		//serialport baudrate 
	bool		auto_reconnect;			//auto reconnect 
    bool		reversion;				//add 180.0 
	bool		inverted;				//turn backwards(if it is true)
	double		angle_max;				//angle max value for lidar 
	double		angle_min;				//angle min value for lidar  
	double		range_max;				//measure distance max value for lidar  
	double		range_min;				//measure distance min value for lidar  
	double 		aim_speed;				//lidar aim speed   
	int			sampling_rate;			//sampling rate  
	bool 		angle_offset_change_flag;  //is enable to change angle offset 
	double 		angle_offset;			//angle offset 

	std::string ignore_array_string;	//filter angle ,string,like ,
	std::vector<float> ignore_array;	//filter angle to array list 

	bool 		resolution_fixed;		//is good resolution  

	bool 		log_enable_flag;		//is use the log?
}Nvilidar_UserConfigTypeDef;

//lidar point 
typedef struct{
	uint16_t   distance;
	double     angle;
	uint16_t   quality;	
	double     speed;	
}Nvilidar_PackagePoint;

//circle data  
typedef struct
{
	uint64_t  startStamp;			//One Lap Start Timestamp 
	uint64_t  stopStamp;			//One Lap Stop Timestamp 
	uint64_t  differStamp;	  		//differ Timestamp
	std::vector<Nvilidar_Node_Info>  lidarCircleNodePoints;	//lidar point data
	VP100Lidar_ErrorFlagEnumTypeDef  error_code;			//error code 
}CircleDataInfoTypeDef;



//======================================Output data information============================================ 
/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
	/// lidar angle. unit(rad)
	float angle;
	/// lidar range. unit(m)
	float range;
	/// lidar intensity
	float intensity;
	/// stamp 
	uint64_t stamp;
} NviLidarPoint;

/**
 * @brief A struct for returning configuration from the NVILIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
	/// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float min_angle;
	/// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float max_angle;
	/// angle resoltuion [rad]
	float angle_increment;
	/// Scan resoltuion [s]
	float time_increment;
	/// Time between scans
	float scan_time;
	/// Minimum range [m]
	float min_range;
	/// Maximum range [m]
	float max_range;
} NviLidarConfig;


typedef struct {
	// System time when first range was measured in nanoseconds
	uint64_t stamp_start;
	uint64_t stamp_stop;
	uint64_t stamp_differ;
	// Array of lidar points
	std::vector<NviLidarPoint> points;
	// Configuration of scan
	NviLidarConfig config;	
	// error code 
	VP100Lidar_ErrorFlagEnumTypeDef  error_code;
} LidarScan;


#endif
