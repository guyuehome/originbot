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

#ifndef MIPI_CAM__MIPI_CAM_HPP_
#define MIPI_CAM__MIPI_CAM_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "sensor_msgs/msg/camera_info.hpp"

#include <string>
#include <vector>
#include <sstream>

#include "mipi_cap.h"

namespace mipi_cam
{
class MipiCam
{
public:
  typedef enum
  {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
    IO_METHOD_UNKNOWN,
  } io_method;

  typedef enum
  {
    PIXEL_FORMAT_YUYV,
    PIXEL_FORMAT_UYVY,
    PIXEL_FORMAT_MJPEG,
    PIXEL_FORMAT_YUVMONO10,
    PIXEL_FORMAT_NV12,
    PIXEL_FORMAT_RGB24,
    PIXEL_FORMAT_GREY,
    PIXEL_FORMAT_UNKNOWN
  } pixel_format;

  MipiCam();
  ~MipiCam();

  // start camera
  bool start(
    const std::string & dev, const std::string &outFormat, io_method io, pixel_format pf,
    int image_width, int image_height, int framerate, const std::string& gdc_file_path,
    int rotate_degree);
  // shutdown camera
  bool shutdown(void);

  // grabs a new image from the camera
  // bool get_image(sensor_msgs::msg::Image:::SharedPtr image);
  bool get_image(
    builtin_interfaces::msg::Time & stamp, 
    std::string & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step, std::vector<uint8_t> & data);
  bool get_image_mem(
    // uint64_t & stamp,
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step, std::array<uint8_t, 6220800> & data, uint32_t & data_size);
  bool get_cam_calibration(sensor_msgs::msg::CameraInfo& cam_info, const std::string &file_path);

  void get_formats();  // std::vector<usb_cam::msg::Format>& formats);

  static io_method io_method_from_string(const std::string & str);
  static pixel_format pixel_format_from_string(const std::string & str);

  bool stop_capturing(void);
  bool start_capturing(void);
  bool is_capturing();

private:
  // TODO(oal) just store an Image shared_ptr here
  typedef struct
  {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    builtin_interfaces::msg::Time stamp;
    char * image;
    int is_new;
  } camera_image_t;

  struct buffer
  {
    void * start;
    size_t length;
  };

  bool mjpeg2rgb(char * MJPEG, int len, char * RGB, int NumPixels);
  bool process_image(const void * src, int len, camera_image_t * dest);
  bool uninit_device(void);
  bool init_device(int image_width, int image_height, int framerate,
                   const std::string &gdc_file_path,
                   int rotate_degree);
  bool close_device(void);
  bool open_device(void);

  rclcpp::Clock::SharedPtr clock_;
  std::string camera_dev_;
  unsigned int pixelformat_;
  std::string out_format_;
  bool monochrome_;
  io_method io_;
  int fd_;
  buffer * buffers_;
  unsigned int n_buffers_;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  camera_image_t * image_pub_;
  camera_image_t * image_;
  bool is_capturing_;
  MipiDevice *m_pMipiDev = NULL;
  TCamInfo m_oCamInfo;
  static void GetCaptureHdl(struct TCapFrame *frame, void *user_args);
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
