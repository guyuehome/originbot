// Copyright (c) 2022, www.guyuehome.com

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

using hbm_img_msgs::msg::HbmMsg1080P;
using sensor_msgs::msg::CompressedImage;
using std::placeholders::_1;

class Nv122BGR : public rclcpp::Node
{
  public:
    Nv122BGR(std::string node_name = "Nv122BGR") : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), node_name);

        // 创建消息订阅者，从摄像头节点订阅图像消息
        sublisher_ = this->create_subscription_hbmem<HbmMsg1080P>(
            "/hbmem_img", 10, std::bind(&Nv122BGR::image_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_out", 10);
    }

  private:
    void image_callback(const HbmMsg1080P::ConstSharedPtr img_msg)
    {
        // 对订阅到的图片消息进行验证，本示例只支持处理NV12格式图片数据
        if (!img_msg)
            return;
        if ("nv12" != std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
        {
            RCLCPP_ERROR(rclcpp::get_logger("Nv122BGR"), "Only support nv12 img encoding!");
            return;
        }
        cv::Mat nv12Image(img_msg->height * 3 / 2, img_msg->width, CV_8UC1,
                          const_cast<uint8_t *>(img_msg->data.data()));

        cv::Mat bgrImage;
        cv::cvtColor(nv12Image, bgrImage, cv::COLOR_YUV2BGR_NV12);

        std_msgs::msg::Header hdr;
        auto rosMsg = cv_bridge::CvImage(hdr, "bgr8", bgrImage).toCompressedImageMsg();
        image_pub_->publish(std::move(*rosMsg));
    }
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
    rclcpp::SubscriptionHbmem<HbmMsg1080P>::ConstSharedPtr sublisher_ = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nv122BGR>("Nv122BGR");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
