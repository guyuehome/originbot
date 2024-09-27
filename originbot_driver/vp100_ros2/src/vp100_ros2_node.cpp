#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <pthread.h>

#include <sys/time.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

#include "nvilidar_process.h"
#include "mytimer.hpp"

//version 
#define ROS2Verision "1.0.3"

//define
#define READ_PARAM(TYPE, NAME, VAR, VALUE) VAR = VALUE; \
       	node->declare_parameter<TYPE>(NAME, VAR); \
       	node->get_parameter(NAME, VAR);

//get stamp 
uint64_t  get_stamp_callback(){
    uint64_t current_time = 0;
    current_time = rclcpp::Clock().now().nanoseconds();
    return current_time;
}

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);    //init
    printf(" _   ___      _______ _      _____ _____          _____ \n");
    printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
    printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
    printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
    printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
    printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
    printf("\n");
    fflush(stdout);

    auto node = rclcpp::Node::make_shared("vp100_ros2_node");

    RCLCPP_INFO(node->get_logger(), "[NVILIDAR INFO] Current ROS2 Driver Version: %s\n", ((std::string)ROS2Verision).c_str());  //version 

    Nvilidar_UserConfigTypeDef cfg;

    //sync para form rviz 
    READ_PARAM(std::string, "serialport_name", (cfg.serialport_name), "/dev/nvilidar");
    READ_PARAM(int, "serialport_baud", (cfg.serialport_baud), 230400);
    READ_PARAM(std::string, "frame_id", (cfg.frame_id), "laser_frame");
    READ_PARAM(bool, "resolution_fixed", (cfg.resolution_fixed), true);
    READ_PARAM(bool, "auto_reconnect", (cfg.auto_reconnect), false);
    READ_PARAM(bool, "reversion", (cfg.reversion), false);
    READ_PARAM(bool, "inverted", (cfg.inverted), false);
    READ_PARAM(double, "angle_max", (cfg.angle_max), 180.0);
    READ_PARAM(double, "angle_min", (cfg.angle_min), -180.0);
    READ_PARAM(double, "range_max", (cfg.range_max), 64.0);
    READ_PARAM(double, "range_min", (cfg.range_min), 0.001);
    READ_PARAM(double, "aim_speed", (cfg.aim_speed), 6.0);
    READ_PARAM(int, "sampling_rate", (cfg.sampling_rate), 3);
	READ_PARAM(bool, "angle_offset_change_flag", (cfg.angle_offset_change_flag), false);
    READ_PARAM(double, "angle_offset", (cfg.angle_offset), 0.0);
    READ_PARAM(std::string, "ignore_array_string", (cfg.ignore_array_string), "");
    READ_PARAM(bool, "log_enable_flag", (cfg.log_enable_flag), true);
    
    //choice use serialport
    vp100_lidar::LidarProcess laser(cfg.serialport_name,cfg.serialport_baud,get_stamp_callback,1e9);

    //reload lidar parameter 
    laser.LidarReloadPara(cfg); 

    //lidar init
    bool ret = laser.LidarInitialialize();
    if (ret) {
        //turn on the lidar 
        ret = laser.LidarTurnOn();
        if (!ret) {
            RCLCPP_ERROR(node->get_logger(),"Failed to start Scan!!!");
        }
    } 
    else {
        RCLCPP_ERROR(node->get_logger(),"Error initializing NVILIDAR Comms and Status!!!");
    }

    auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

    rclcpp::WallRate loop_rate(50);

    //read lidar data 
    while (ret && rclcpp::ok())
    {
        LidarScan scan;

        //get lidar data
        try
        {
            /* code */
            if(laser.LidarSamplingProcess(scan))
            {
            	if(scan.points.size() > 0)
            	{
					auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
                    size_t avaliable_count = 0;

					scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp_start);
					scan_msg->header.stamp.nanosec =  scan.stamp_start - RCL_S_TO_NS(scan_msg->header.stamp.sec);
					scan_msg->header.frame_id = cfg.frame_id;
					scan_msg->angle_min = scan.config.min_angle;
					scan_msg->angle_max = scan.config.max_angle;
					scan_msg->angle_increment = scan.config.angle_increment;
					scan_msg->scan_time = scan.config.scan_time;
					scan_msg->time_increment = scan.config.time_increment;
					scan_msg->range_min = scan.config.min_range;
					scan_msg->range_max = scan.config.max_range;

					size_t size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
					scan_msg->ranges.clear();
					scan_msg->ranges.resize(size);
					scan_msg->intensities.clear();
					scan_msg->intensities.resize(size);
                	avaliable_count = 0;
					for(size_t i=0; i < scan.points.size(); i++) {
						size_t index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
						if(index < size) 
						{
                        	avaliable_count++;

							scan_msg->ranges[index] = scan.points[i].range;
							scan_msg->intensities[index] = scan.points[i].intensity;
						}
					}
	                if(cfg.resolution_fixed){   //fix counts  
	                    if(size > avaliable_count){
	                        for(size_t j = avaliable_count; j<size; j++){
	                            scan_msg->ranges[j] = 0;
	                            scan_msg->intensities[j] = 0;
	                        }
	                    }
	                }
					laser_pub->publish(*scan_msg);
                }
                else 
                {
                	RCLCPP_WARN(node->get_logger(), "Lidar Data Invalid!");
                }
            }
            else 
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to get Lidar Data!");
                break;
            }

            rclcpp::spin_some(node);
        }
        catch(const rclcpp::exceptions::RCLError &e)
        {
            //RCLCPP_ERROR(node->get_logger(),"unexpectedly failed with %s",e.what());
        }
        
        loop_rate.sleep();
    }

    laser.LidarTurnOff();
    RCLCPP_INFO(node->get_logger(), "[NVILIDAR INFO] Now NVILIDAR is stopping .......");
    laser.LidarCloseHandle();
    rclcpp::shutdown();

    return 0;
}
