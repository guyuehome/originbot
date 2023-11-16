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

#include "mipi_cam/mipi_cam_node.hpp"

#include <sstream>

#include "sensor_f37_config.h"
#include "sensor_gc4663_config.h"
// #include <std_srvs/srv/Empty.h>

#include <stdarg.h>

#include <memory>
#include <string>
#include <vector>
#include <fstream>

extern "C" int ROS_printf(char* fmt, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, fmt);
  vsprintf(buf, fmt, args);
  RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "%s", buf);
  va_end(args);
}
#define PUB_BUF_NUM 5
namespace mipi_cam {

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions& node_options)
    : m_bIsInit(0),
      Node("mipi_cam", node_options),
      img_(new sensor_msgs::msg::Image()),
      camera_calibration_info_(new sensor_msgs::msg::CameraInfo()) {
  std::string cfg_info;
  std::ifstream sif_info("/sys/devices/platform/soc/a4001000.sif/cfg_info");
  if (sif_info.is_open()) {
    sif_info >> cfg_info;
  }
  // 若mipi camera已打开，sif_info返回“pipeid”
  if (!cfg_info.compare("pipeid")) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_node"),
                 "mipi camera already in use.\n");
    exit(0);  //还未创建node，直接退出即可，或者抛出异常
    // throw std::runtime_error("mipi camera already in use.");
  }

  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", PUB_BUF_NUM);
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);  // 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 1080);  // 480);
  this->declare_parameter("image_width", 1920);   // 640);
  this->declare_parameter("io_method", "ros");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("out_format", "bgr8");   // nv12
  this->declare_parameter("video_device", "sc132gs");  // "IMX415");//"F37");
  this->declare_parameter("camera_calibration_file_path",
                          "/userdata/dev_ws/src/origineye/mipi_cam/config/SC132GS_calibration.yaml");
  this->declare_parameter("gdc_file_path", "/userdata/dev_ws/src/mipi_cam/config/sc132gs_gdc.bin");
  this->declare_parameter("rotate_degree", 0);
  get_params();
  init();  //外部可能会调用了
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
              "[%s]->mipinode init sucess.\n",
              __func__);
}

MipiCamNode::~MipiCamNode() {
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  mipiCam_.shutdown();
}

void MipiCamNode::get_params() {
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto& parameter :
       parameters_client->get_parameters({"camera_name",
                                          "camera_info_url",
                                          "out_format",
                                          "frame_id",
                                          "framerate",
                                          "image_height",
                                          "image_width",
                                          "io_method",
                                          "pixel_format",
                                          "video_device",
                                          "camera_calibration_file_path",
                                          "gdc_file_path",
                                          "rotate_degree"})) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "camera_name value: %s",
                  parameter.value_to_string().c_str());
      camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_format") {
      out_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "framerate: %f",
                  parameter.as_double());
      framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_height_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_width_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      pixel_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "video_device value: %s",
                  parameter.value_to_string().c_str());
      video_device_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_calibration_file_path") {
      camera_calibration_file_path_ = parameter.value_to_string();
    } else if (parameter.get_name() == "gdc_file_path") {
      gdc_file_path_ = parameter.value_to_string();
    } else if (parameter.get_name() == "rotate_degree") {
      rotate_degree_ = parameter.as_int();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "Invalid parameter name: %s",
                  parameter.get_name().c_str());
    }
  }
}

void MipiCamNode::service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  (void)request_header;
  if (request->data) {
    mipiCam_.start_capturing();
    response->message = "Start Capturing";
  } else {
    mipiCam_.stop_capturing();
    response->message = "Stop Capturing";
  }
}

void MipiCamNode::init() {
  if (m_bIsInit) return;

  // 使能sensor mclk
  std::vector<std::string> sys_cmds{
      "echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en",
      "echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en",
      "echo 1 > /sys/class/vps/mipi_host2/param/snrclk_en",
      "echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq",
      "echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq",
      "echo 24000000 > /sys/class/vps/mipi_host2/param/snrclk_freq",
      "echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart",
      "echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart",
      "echo 1 > /sys/class/vps/mipi_host2/param/stop_check_instart"
  };
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }

  // 自适应获取video_device_name
  std::string video_device_name_temp = x3_get_video_device(); // 获取目前连接的video_device
  if(video_device_name_temp.empty()) { // 未检测到有video_device连接
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_cap"),
                       "No camera detected! Please check if camera is connected!");
    rclcpp::shutdown();
    return;
  } else if(strcasecmp(video_device_name_temp.c_str(), video_device_name_.c_str())) { 
    // 与用户传入的video_device不一致，比较不区分大小写
    // 当检测到的sensor与用户传入的不一致时，打开检测到的sensor，并输出log提示用户
    RCLCPP_WARN_ONCE(rclcpp::get_logger("mipi_node"),
                   "No %s video_device was detected, but mipi_cam detected %s, mipi_cam will open %s device!"
                   " You can change the video_device parameter to %s in the "
                   "'/opt/tros/share/mipi_cam/launch/mipi_cam.launch.py'",
                   video_device_name_.c_str(), 
                   video_device_name_temp.c_str(),
                   video_device_name_temp.c_str(),
                   video_device_name_temp.c_str());
    video_device_name_ = video_device_name_temp;
  }

  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mipi_node"),
        "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  img_->header.frame_id = frame_id_;
  camera_calibration_info_->header.frame_id = frame_id_;
  RCLCPP_INFO(
      rclcpp::get_logger("mipi_node"),
      "[MipiCamNode::%s]->Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS "
      "gdc: %s rotate_degree: %d",
      __func__,
      camera_name_.c_str(),
      video_device_name_.c_str(),
      image_width_,
      image_height_,
      io_method_name_.c_str(),
      pixel_format_name_.c_str(),
      framerate_,
      gdc_file_path_.c_str(),
      rotate_degree_);
  // set the IO method
  MipiCam::io_method io_method =
      MipiCam::io_method_from_string(io_method_name_);
  if (io_method_name_.compare("ros") == 0) {
    image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_raw", PUB_BUF_NUM);
  }
#ifdef USING_HBMEM
  if (io_method_name_.compare("shared_mem") == 0) {
    // 创建hbmempub
    publisher_hbmem_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            "hbmem_img", PUB_BUF_NUM);
  }
#endif

  // set the pixel format
  MipiCam::pixel_format pixel_format =
      MipiCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == MipiCam::PIXEL_FORMAT_UNKNOWN) {
    RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("mipi_node"),
        "Unknown pixel format '%s'! yuyv、uyvy、mjpeg、yuvmono10、rgb24 and "
        "grey are supported! Please modify the parameter pixel_format!",
        pixel_format_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  ROS_printf("===>[%s]->start cam w:h=%d:%d.\n",
             __func__,
             image_width_,
             image_height_);
  // start the camera
  if (!mipiCam_.start(video_device_name_,
                      out_format_name_,
                      io_method,
                      pixel_format,
                      image_width_,
                      image_height_,
                      framerate_,
                      gdc_file_path_,
                      rotate_degree_)) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
                      "video dev '%s' start failed!",
                      video_device_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  mipiCam_.get_formats();
  const int period_ms = 1000.0 / framerate_;
  if (!mipiCam_.get_cam_calibration(*camera_calibration_info_,
                                    camera_calibration_file_path_)) {
    camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
  }
  if (io_method_name_ != "shared_mem") {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::update, this));
  } else {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::hbmem_update, this));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  m_bIsInit = 1;
}

bool MipiCamNode::take_and_send_image() {
  // grab the image
  if (!mipiCam_.get_image(img_->header.stamp,
                          img_->encoding,
                          img_->height,
                          img_->width,
                          img_->step,
                          img_->data)) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "grab failed");
    return false;
  }
  image_pub_->publish(*img_);
  return true;
}

bool MipiCamNode::send_calibration(const builtin_interfaces::msg::Time& stamp) {
  if (camera_calibration_info_ != nullptr) {
    camera_calibration_info_->header.stamp = stamp;
    info_pub_->publish(*camera_calibration_info_);
    return true;
  } else {
    return false;
  }
}

void MipiCamNode::update() {
  if (mipiCam_.is_capturing()) {
    if (!take_and_send_image()) {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "mipi camera did not respond in time.");
    }
    if (send_calibration(img_->header.stamp)) {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "publish camera info.\n");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "Unable to publish camera info.\n");
    }
  }
}
void MipiCamNode::hbmem_update() {
#ifdef USING_HBMEM
  if (mipiCam_.is_capturing()) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!mipiCam_.get_image_mem(msg.time_stamp,
                                  msg.encoding,
                                  msg.height,
                                  msg.width,
                                  msg.step,
                                  msg.data,
                                  msg.data_size)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_node"),
                     "hbmem_update grab img failed");
        return;
      }
      msg.index = mSendIdx++;
      publisher_hbmem_->publish(std::move(loanedMsg));

      if (send_calibration(msg.time_stamp)) {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "publish camera info.\n");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                    "Unable to publish camera info.\n");
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "borrow_loaned_message failed");
    }
  }
#endif
}
}  // namespace mipi_cam
