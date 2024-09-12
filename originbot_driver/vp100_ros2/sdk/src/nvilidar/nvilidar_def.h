#ifndef _NVILIDAR_DEF_H_
#define _NVILIDAR_DEF_H_

#include <stdint.h>
#include "nvilidar_protocol.h"
#include <string>


//======================================basic parameter============================================ 

//SDK version 
#define NVILIDAR_SDKVerision     "1.0.0"

//PI def
#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif 

//other 
#define NVILIDAR_DEFAULT_TIMEOUT     2000    //default timeout 
#define NVILIDAR_POINT_TIMEOUT		 2000	 //one circle time  for example, the lidar speed is 10hz ,the timeout must smaller the 100ms

#define NVILIDAR_COMMUNICATE_TWO_WAY	0
#define NVILIDAR_SINGLE_PACK_POINTS		8		//one pack has N points 

//lidar model  list 
typedef enum
{
	NVILIDAR_Unknow = 0,		//unknow lidar 
   	NVILIDAR_VP100,				//lidar VP100
   	NVILIDAR_Tail,
}LidarModelListEnumTypeDef;


//======================================other parameters============================================ 

//filter para 
//lidar filter para --- lidar tail para 
typedef struct{
	bool   enable;
	int    level;
	bool   distance_limit_flag;
	int    distance_limit_value;
	int    neighbors;
}TailFilterPara;
//lidar filter para --- lidar sliding filter para 
typedef struct{
	bool   enable;          
	int    jump_threshold;  
	int    max_range;       
	bool   max_range_flag;  
	int    window;
}SlidingFilterPara;
//lidar filter para 
typedef struct{
	TailFilterPara  tail_filter;
	SlidingFilterPara  sliding_filter;
}FilterPara;

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
	LidarModelListEnumTypeDef  lidar_model_name;	//lidar model name 

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

	FilterPara	filter_para;			//lidar pointcloud filter para info 
}Nvilidar_UserConfigTypeDef;

//lidar receive info typedef 
typedef union {
	uint8_t buf[256];
	Nvilidar_Node_Package_Quality        pack_qua;
	Nvilidar_Node_Package_No_Quality     pack_no_qua;
}Nvilidar_PackageBufTypeDef;

//lidar point 
typedef struct{
	uint16_t   distance;
	double     angle;
	uint8_t    quality;	
	double     speed;	
}Nvilidar_PackagePoint;

//package info 
typedef struct 
{
	bool     packageHasQuality;		//is has quality,it is defined by protocol 
	Nvilidar_PackagePoint  packagePoints[NVILIDAR_SINGLE_PACK_POINTS];    //points info 
	bool     packageErrFlag;       //error flag 
	uint16_t packageCheckSumGet;   //crc get from protocol 
	uint16_t packageCheckSumCalc;  //crc calculate 
	uint16_t packageSpeed;         //lidar speed 
	bool     packageHas0CAngle;    //is 0 
	uint16_t package0CIndex;   		//is 0 index 
	uint64_t packageStamp;		   //time stamp  
}Nvilidar_PointViewerPackageInfoTypeDef;

//one circle data info  
typedef struct
{
	uint64_t  startStamp;			//One Lap Start Timestamp 
	uint64_t  stopStamp;			//One Lap Stop Timestamp 
	std::vector<Nvilidar_Node_Info>  lidarCircleNodePoints;	//lidar point data
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
	/// System time when first range was measured in nanoseconds
	uint64_t stamp;
	/// Array of lidar points
	std::vector<NviLidarPoint> points;
	/// Configuration of scan
	NviLidarConfig config;
} LidarScan;



#endif
