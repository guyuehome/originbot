#ifndef _NVILIDAR_PROTOCOL_H_
#define _NVILIDAR_PROTOCOL_H_

#include <stdint.h>
#include <vector>
#include <string>

//一包最多的点数信息
#define NVILIDAR_POINT_HEADER                  0xAA55        //包头信息
#define NVILIDAR_T10_POINT_HEADER              0x2C54        //包头信息

//one pack points 
#define NORMAL_NO_QUALITY_PACK_MAX_POINTS      8            //normal no quality 
#define NORMAL_HAS_QUALITY_PACK_MAX_POINTS     8            //normal has quality 
#define YW_HAS_QUALITY_PACK_MAX_POINTS         12           //yw has quality 
#define FS_HAS_QUALITY_PACK_MAX_POINTS         17           //fs has quality 
#define T10_HAS_QUALITY_PACK_MAX_POINTS        12           //t10 has quality 

const uint8_t T10_CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

//pack 
#pragma pack(push)
#pragma pack(1)

//command enum 
typedef enum{
    PROTOCOL_VP100_NORMAL_NO_QUALITY = 0x0208,              //no quality 
    PROTOCOL_VP100_NORMAL_QUALITY = 0x0308,                 //has quality 
    PROTOCOL_VP100_YW_QUALITY = 0x070C,                     //yw quality has quality 
    PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY = 0x0711,          //fs quality has quality 
    PROTOCOL_T10_MODEL_QUAILIY = 0x2C54,                    //T10 quality
    PROTOCOL_VP100_ERROR_FAULT = 0x8008                     //error code 
}VP100_LidarModel_Enum;

//single point info 
typedef struct{
    uint8_t    lidar_angle_zero_flag;       //is check angle 0 
    uint16_t   lidar_quality;               //quality info 
    float      lidar_angle;                 //check angle info 
    uint16_t   lidar_distance;              //current distance 
    uint64_t   lidar_stamp;                 //timestamp 
    float      lidar_speed;                 //scan speed 
    uint32_t   lidar_point_time;            //time 
    uint8_t    lidar_index;                 //current index   
    uint8_t    lidar_error_package;         //error pack info 
}Nvilidar_Node_Info;

//lidar version info 
typedef struct{
    std::string downBoard_Model;            //lidar down model 
    std::string downBoard_HardVersion;      //lidar down hard version 
    std::string downBoard_SoftVersion;      //lidar down software version 
    std::string downBoard_ID;               //lidar down id
    std::string downBoard_Date;             //lidar down date
    std::string downBoard_UID;              //lidar down uid 
    std::string downBoard_InputVoltage;     //lidar down voltage 
    std::string downBoard_BuildTime;        //lidar down buildtime

    std::string upBoard_Model;              //lidar up model 
    std::string upBoard_HardVersion;        //lidar up hard version 
    std::string upBoard_SoftVersion;        //lidar up soft version 
    std::string upBoard_ID;                 //lidar up id 
    std::string upBoard_Date;               //lidar up date 
    std::string upBoard_UID;                //lidar up uid
    std::string upBoard_BuildTime;          //lidar up build time
    uint8_t upBoard_VBD;                    //lidar up vbd
    uint8_t upBoard_TDC;                    //lidar up tdc
    int8_t  upBoard_Temperature;            //lidar up temperature 
}VP100_Head_LidarInfo_TypeDef;

//雷达信息 接收数据
typedef struct
{
    uint8_t   package_head;                 //包头
    uint8_t   package_cmd;                  //命令字
    uint8_t   package_length;               //长度
    uint8_t   package_data[255];            //有效数据 + 最后一个字节校验
}VP100_LidarInfo_DownBoard_Node_Package;

typedef struct
{
    uint16_t  package_head;                 //包头
    uint8_t   package_cmd;                  //命令字
    uint8_t   package_length;               //长度
    uint8_t   package_data[255];            //有效数据 + 最后一个字节校验
}VP100_LidarInfo_UpBoard_Node_Package;

//======================== normal no quality 
//normal no quality 
typedef struct {
    uint16_t PakageSampleDistance;
}VP100_Normal_Protocol_PackageNode_NoQuality;

//normal no quality 
typedef struct{
    uint16_t  package_head;                 //head 
    uint8_t   package_information;          //info 
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed 
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_Normal_Protocol_PackageNode_NoQuality  package_Sample[NORMAL_NO_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc 
}VP100_Normal_Node_Package_No_Quality;

//======================== normal has quality 
//normal has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint8_t  PakageSampleQuality;
}VP100_Normal_Protocol_PackageNode_Quality;

//normal has quality 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_Normal_Protocol_PackageNode_Quality  package_Sample[NORMAL_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_Normal_Node_Package_Quality;

//===================yw protocol has quality 
//yw has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint16_t  PakageSampleQuality;
}VP100_YW_Protocol_PackageNode_Quality;
//yw has quality 
typedef struct
{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_YW_Protocol_PackageNode_Quality  package_Sample[YW_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_YW_Node_Package_Quality;

//===================fs protocol has quality 
//fs has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint16_t PakageSampleQuality;
}VP100_FS_Test_Protocol_PackageNode_Quality;
//fs has quality 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_FS_Test_Protocol_PackageNode_Quality  package_Sample[FS_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_FS_Test_Node_Package_Quality;

//====================T10 protocol has quality
//T10 has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint8_t  PakageSampleQuality;
}VP100_T10_Protocol_PackageNode_Quality;
//T10 has quality 
typedef struct{
    uint16_t  package_head;                 //head
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_T10_Protocol_PackageNode_Quality  package_Sample[T10_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_timeStamp;            //time stamp 
    uint8_t   package_checkSum;             //crc
}VP100_T10_Node_Package_Quality;

//====================lidar error code 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint8_t   package_constByteFirst;       //0x45 const byte
    uint8_t   package_errorCode;            //error code 
    uint8_t   package_constByteSecond;      //0x00 const byte
    uint8_t   package_checkAddSum;          //check sum 
}VP100_Error_Fault_TypeDef;


//pack info with union 
typedef union{
    VP100_Normal_Node_Package_Quality               vp100_normal_quality;       
    VP100_Normal_Node_Package_No_Quality            vp100_normal_no_quality;    
    VP100_YW_Node_Package_Quality                   vp100_yw_quality;        
    VP100_FS_Test_Node_Package_Quality              vp100_fs_test_quality;  
    VP100_T10_Node_Package_Quality                  vp100_t10_quality;
    VP100_Error_Fault_TypeDef                       vp100_error_fault_info;
    VP100_LidarInfo_UpBoard_Node_Package            vp100_lidar_info_upboard;   //upboard info 
    VP100_LidarInfo_DownBoard_Node_Package          vp100_lidar_info_downboard; //downboard info 
    uint8_t  buf[1024];
}VP100_Node_Package_Union;

#pragma pack(pop)

//get the protocol type size of the lidar model 
#define GET_LIDAR_DATA_SIZE(model) \
    (   \
      (model == PROTOCOL_VP100_NORMAL_NO_QUALITY) ? sizeof(VP100_Normal_Node_Package_No_Quality) : \
      (model == PROTOCOL_VP100_NORMAL_QUALITY) ? sizeof(VP100_Normal_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_YW_QUALITY) ? sizeof(VP100_YW_Node_Package_Quality) : \
      (model == PROTOCOL_T10_MODEL_QUAILIY) ? sizeof(VP100_T10_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY) ? sizeof(VP100_FS_Test_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_ERROR_FAULT) ? sizeof(VP100_Error_Fault_TypeDef) : \
      0 \
    )


#endif
