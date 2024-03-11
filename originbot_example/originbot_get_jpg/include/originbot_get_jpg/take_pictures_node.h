/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>

#ifndef ORIGINBOTEYS_DEMO_TAKE_PICTURES_NODE_H
#define ORIGINBOTEYS_DEMO_TAKE_PICTURES_NODE_H
#define ISDIGIT(x)  ((x) >= '0' && (x) <= '9')

using rclcpp::NodeOptions;

namespace take_pictures 
{
class TakePicturesNode : public rclcpp::Node
{
public:
    explicit TakePicturesNode(const rclcpp::NodeOptions & node_options = NodeOptions(),
        std::string node_name = "take_pictures", std::string topic_name = "");
     ~TakePicturesNode();
    void saveHbmImage(hbm_img_msgs::msg::HbmMsg1080P::SharedPtr image_msg);
    sensor_msgs::msg::Image Hbmem2Sensor(
        hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
    sensor_msgs::msg::Image NV122RGB(
        const sensor_msgs::msg::Image &image_nv12,const cv::Mat&mat_dst);
    std::string topic_name_ = "/image_raw";
    int mkdirPath(const char* pathName);
    int openPath(const char* pathName);
    int displayMenu2GetFrame();
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
        subscription_ = nullptr;
    rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
        hbmem_subscription_;
    int take_nums_ = 0;
    std::string save_dir_ = "/userdata/dev_ws/imagedata/";
    void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void hbmem_topic_callback(hbm_img_msgs::msg::HbmMsg1080P::SharedPtr image_msg);
    
};// TakePictures
}// takepictures

#endif //#define ORIGINBOTEYS_DEMO_TAKE_PICTURES_NODE_H