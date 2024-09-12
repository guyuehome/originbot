#ifndef _NVILIDAR_PROTOCOL_H_
#define _NVILIDAR_PROTOCOL_H_

#include <stdint.h>
#include <vector>

//长短命令头字节 尾字节
// #define NVILIDAR_START_BYTE_LONG_CMD           0x40         //长命令  头字节
// #define NVILIDAR_START_BYTE_SHORT_CMD          0xFE         //短命令  头字节
// #define NVILIDAR_END_CMD                       0xFF         //包尾字节

//一包最多的点数信息
#define NVILIDAR_PACK_MAX_POINTS               8            //一包对应的点数目  

#define NVILIDAR_POINT_HEADER                  0xAA55        //包头信息

#define NVILIDAR_RESP_MEASUREMENT_CHECKBIT     (0x1)        //角度标记位是否有效
#define NVILIDAR_ANGULDAR_RESOLUTION           64           //角分辨率 即多少数表示为1度角

#define NVILIDAR_SINGLE_PACK_MAX               1200         //单包点最大字节数

#define NVILIDAR_PACK_BYTES_SUM                36           //一包最多多少个字节信息 

//包对齐
#pragma pack(push)
#pragma pack(1)

//单点结构体信息
struct Nvilidar_Node_Info 
{
    uint8_t    lidar_angle_zero_flag;     //检测到0位角标记
    uint16_t   lidar_quality;             //信号质量
    float      lidar_angle;               //测距点角度
    uint16_t   lidar_distance;            //当前测距点距离
    uint64_t   lidar_stamp;               //时间戳
    float      lidar_speed;               //扫描频率
    uint32_t   lidar_point_time;          //2点时间间隔
    uint8_t    lidar_index;               //当前索引  
    uint8_t    lidar_error_package;       //错包信息 
};

//包信息(带信号质量)
struct Nvilidar_Protocol_PackageNode_Quality
{
    uint16_t PakageSampleDistance;
    uint8_t  PakageSampleQuality;
};

//包信息（不带信号质量）
struct Nvilidar_Protocol_PackageNode_NoQuality
{
    uint16_t PakageSampleDistance;
};

//包数目 加信号质量
struct Nvilidar_Node_Package_Quality 
{
    uint16_t  package_head;                 //包头
    uint8_t   package_information;          //信息 
    uint8_t   package_distanceDataNumber;   //点云数量 
    uint16_t  package_speed;                //速度 
    uint16_t  package_firstSampleAngle;     //起始角 
    Nvilidar_Protocol_PackageNode_Quality  package_Sample[NVILIDAR_PACK_MAX_POINTS];
    uint16_t  package_lastSampleAngle;      //结束角
    uint16_t  package_checkSum;             //校验
};

//包信息 不加信号质量
struct Nvilidar_Node_Package_No_Quality 
{
    uint16_t  package_head;                 //包头
    uint8_t   package_information;          //信息 
    uint8_t   package_distanceDataNumber;   //点云数量 
    uint16_t  package_speed;                //速度 
    uint16_t  package_firstSampleAngle;     //起始角 
    Nvilidar_Protocol_PackageNode_NoQuality  package_Sample[NVILIDAR_PACK_MAX_POINTS];
    uint16_t  package_lastSampleAngle;      //结束角
    uint16_t  package_checkSum;             //校验
};

//包信息 共用体 
union Nvilidar_Node_Package_Union{
    uint8_t  buf[NVILIDAR_PACK_BYTES_SUM];          //一包多少个字节信息 
    Nvilidar_Node_Package_Quality    quality;       //带信号质量 
    Nvilidar_Node_Package_No_Quality no_quality;    //不带信号质量 
};

#pragma pack(pop)


#endif
