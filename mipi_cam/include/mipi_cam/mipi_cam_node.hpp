// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MIPI_CAM__MIPI_CAM_NODE_HPP_
#define MIPI_CAM__MIPI_CAM_NODE_HPP_
#include "mipi_cam/mipi_cam.hpp"

#include <rclcpp/rclcpp.hpp>

// #include <vector>
#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/string.hpp>

#ifdef USING_HBMEM
#include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

namespace mipi_cam
{
class MipiCamNode : public rclcpp::Node
{
public:
  MipiCamNode(const rclcpp::NodeOptions & node_options);
  ~MipiCamNode();

  void init();
  void get_params();
  void update();
  void hbmem_update();
  bool take_and_send_image();
  bool send_calibration(const builtin_interfaces::msg::Time &stamp);

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  MipiCam mipiCam_;

  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_ = nullptr;
#ifdef USING_HBMEM
  int32_t mSendIdx = 0;
  rclcpp::TimerBase::SharedPtr timer_hbmem_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;
#endif
/*
  sensor_msgs::msg::CompressedImage::SharedPtr ros_img_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr video_compressed_publisher_;
*/
  // parameters
  std::string video_device_name_;
  std::string frame_id_;

  std::string io_method_name_;  // hbmem zero mem copy
  size_t count_;
  // these parameters all have to be a combination supported by the device
  // Use
  // v4l2-ctl --device=0 --list-formats-ext
  // to discover them,
  // or guvcview
  std::string pixel_format_name_;
  std::string out_format_name_;
  std::string gdc_file_path_;
  int rotate_degree_;
  int image_width_;
  int image_height_;
  int framerate_;
  int m_bIsInit;

  std::string camera_name_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;

  rclcpp::TimerBase::SharedPtr timer_;
  // 滑动窗口测方差 ， 20 s ，标准帧率
  /*std::vector<int> m_vecFps;
  int m_nMinFps;
  int m_nMaxFps;
  int m_nVarianceFps;*/
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_capture_;
};
}  // namespace mipi_cam
#endif  // MIPI_CAM__MIPI_CAM_NODE_HPP_
