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

#include "originbot_get_jpg/take_pictures_node.h"
#include <fstream>
namespace take_pictures {
    TakePicturesNode::TakePicturesNode(const rclcpp::NodeOptions& node_options,
        std::string node_name, std::string topic_name)
        : Node(node_name, node_options) {
        this->declare_parameter("sub_img_topic", topic_name_);
        if (this->get_parameter("sub_img_topic", topic_name_)) {
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),
            "Update sub_img_topic with topic_name: %s", topic_name_.c_str());
        }

        RCLCPP_INFO(rclcpp::get_logger("take_pictures"),
            "save_dir: [%s]", save_dir_.c_str());

        this->declare_parameter("take_nums", take_nums_);
        if (this->get_parameter("take_nums", take_nums_)) {
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),
            "Update take_nums with nums: %d", take_nums_);
        }
        
        if(openPath(save_dir_.data()) != 0) {
            int ret = mkdirPath(save_dir_.data());
            if(ret != 0) {
                return ;
            }
        }

        if (!topic_name.empty()) {
            topic_name_ = topic_name;
        }

        if (topic_name_.find("hbmem_img") == std::string::npos) {
            topic_name_  = "/image_raw";
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),
            "Create subscription with topic_name: %s", topic_name_.c_str());
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                topic_name_, 10,
                std::bind(&TakePicturesNode::topic_callback, this,
                            std::placeholders::_1));
        } else {
            hbmem_subscription_ = this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
                    topic_name_, 10,
                    std::bind(&TakePicturesNode::hbmem_topic_callback, this,
                            std::placeholders::_1));
                RCLCPP_WARN(rclcpp::get_logger("take_pictures"),
                    "Create hbmem_subscription with topic_name: %s, sub = %p", topic_name_.c_str(), hbmem_subscription_);

        }
    }

    TakePicturesNode::~TakePicturesNode() {
        RCLCPP_INFO(rclcpp::get_logger("take_pictures"),
            "Finish taking pictures");
    }

    int TakePicturesNode::openPath(const char* pathName) {
        int ret = 0;
        DIR * saveDir  = opendir(pathName);
        if(saveDir == nullptr) {
            ret = -1;
        }
        return ret;
    }

    int TakePicturesNode::mkdirPath(const char* pathName) {
        int ret = 0;
        ret = mkdir(pathName,0755);
        if(ret != 0) {
            RCLCPP_INFO(rclcpp::get_logger("take_pictures"),"mkdir filed\n");
        }
        return ret;
    }
    sensor_msgs::msg::Image TakePicturesNode::Hbmem2Sensor(
            hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
        sensor_msgs::msg::Image image;
        std_msgs::msg::Header header;

        header.stamp = msg->time_stamp;
        header.frame_id = " ";
        image.encoding = "nv12";
        image.header.stamp = msg->time_stamp;
        image.height = msg->height;
        image.width = msg->width;
        image.step = msg->step;
        std::vector<uint8_t> data;
        data.resize(msg->data_size);
        memcpy(&data[0], msg->data.data(), msg->data_size);
        image.data = data;

        return image;
    }

    sensor_msgs::msg::Image TakePicturesNode::NV122RGB(
            const sensor_msgs::msg::Image &image_nv12,const cv::Mat&mat_dst) {
        sensor_msgs::msg::Image image_rgb;
        image_rgb.header = image_nv12.header;
        image_rgb.height = mat_dst.rows;
        image_rgb.width = mat_dst.cols;
        image_rgb.encoding = "rgb8";
        image_rgb.step = mat_dst.cols * mat_dst.elemSize();
        size_t size = mat_dst.rows * mat_dst.cols * mat_dst.elemSize();
        image_rgb.data.resize(size);
        memcpy(image_rgb.data.data(), mat_dst.data, size);

        return image_rgb;
    }


    void TakePicturesNode::saveHbmImage(
        hbm_img_msgs::msg::HbmMsg1080P::SharedPtr image_msg) {
        RCLCPP_INFO(rclcpp::get_logger("take_pictures"),"saveHbmImage start");

        auto image_nv12 = Hbmem2Sensor(image_msg);

        char *buf_src = new char[image_msg->data_size];
        cv::Mat mat_src = cv::Mat(image_nv12.height * 1.5, image_nv12.width, CV_8UC1, buf_src);
        cv::Mat mat_dst = cv::Mat(image_nv12.height, image_nv12.width, CV_8UC3);
        cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_NV12);

        auto image_rgb = NV122RGB(image_nv12,mat_dst);

        cv_bridge::CvImagePtr cv_ptr = nullptr;
        cv_ptr = cv_bridge::toCvCopy(image_rgb, sensor_msgs::image_encodings::RGB8);
        // cv::Mat frame_gray;
        // cvtColor( cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
        cv::Mat frame_rgb = cv_ptr->image;
        std::string fName = save_dir_ + "/raw_img_" +
            std::to_string(image_rgb.header.stamp.sec) +
            std::to_string(image_rgb.header.stamp.nanosec) + ".jpg";
        cv::imwrite(fName, frame_rgb);
        }

    void TakePicturesNode::hbmem_topic_callback(
        hbm_img_msgs::msg::HbmMsg1080P::SharedPtr image_msg) {
        if(displayMenu2GetFrame() > 0)
            saveHbmImage(image_msg);
    }

    void TakePicturesNode::topic_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
        if(displayMenu2GetFrame() > 0){
            cv_bridge::CvImagePtr cv_ptr = nullptr;
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame_rgb = cv_ptr->image;
            std::string fName = save_dir_ + "/raw_img_" +
                std::to_string(image_msg->header.stamp.sec) +
                std::to_string(image_msg->header.stamp.nanosec) + ".jpg";
            cv::imwrite(fName, frame_rgb);
        }
    }

    int TakePicturesNode::displayMenu2GetFrame() {
        char choose[32] = {'\0'};
        if(take_nums_ > 0) {
            take_nums_--;
        }
        if(take_nums_ == 0) {
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),"Please enter a number of images you want to get \n");
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),"if enter q ,the program close\n");
            RCLCPP_WARN(rclcpp::get_logger("take_pictures"),"if enter c ,continue get images\n");

            scanf("%s", choose);
            if (ISDIGIT(choose[0])) {
                take_nums_ = atoi(choose);
	        }
        }
        if(choose[0] == 'q') {
            rclcpp::shutdown();
        } 
        if (choose[0] == 'c') {
            take_nums_ = 10000;
        }
        return take_nums_;
    }

}