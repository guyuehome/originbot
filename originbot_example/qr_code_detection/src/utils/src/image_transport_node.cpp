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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

using hbm_img_msgs::msg::HbmMsg1080P;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class Nv122BGR : public rclcpp::Node
{
public:
    Nv122BGR(std::string node_name = "Nv122BGR") : Node(std::move(node_name)) {
        RCLCPP_INFO(this->get_logger(), node_name);
        
        this->sublisher_ = this->create_subscription_hbmem<HbmMsg1080P>(
            "/hbmem_img", 10, std::bind(&Nv122BGR::image_callback, this, _1));
        this->image_pub_ = image_transport::create_publisher(this, "/image_out");
    }

private:
    void image_callback(HbmMsg1080P::SharedPtr msg)
    {
      cv::Mat nv12Image(msg->height * 3 / 2, msg->width, CV_8UC1, 
        const_cast<uint8_t *>(msg->data.data()));

      cv::Mat bgrImage;
      cv::cvtColor(nv12Image, bgrImage, cv::COLOR_YUV2BGR_NV12);

      std_msgs::msg::Header hdr;
      sensor_msgs::msg::Image::SharedPtr rosMsg;
      rosMsg = cv_bridge::CvImage(hdr, "bgr8", bgrImage).toImageMsg();
      this->image_pub_.publish(std::move(rosMsg));
    }

    rclcpp::SubscriptionHbmem<HbmMsg1080P>::SharedPtr sublisher_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nv122BGR>("Nv122BGR");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
