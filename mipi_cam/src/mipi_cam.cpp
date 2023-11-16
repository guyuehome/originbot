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

#define __STDC_CONSTANT_MACROS
#include "mipi_cam/mipi_cam.hpp"

#include <errno.h>
#include <malloc.h>
#include <unistd.h>

#include "mipi_cam/video_utils.hpp"
// #include <rclcpp/rclcpp.hpp>
#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

#include "sensor_msgs/distortion_models.hpp"
// #include <mipi_cam/msg/formats.hpp>
// #include <sensor_msgs/fill_image.h>
#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace mipi_cam {

MipiCam::MipiCam()
    : io_(IO_METHOD_MMAP),
      fd_(-1),
      buffers_(NULL),
      n_buffers_(0),
      avframe_camera_size_(0),
      avframe_rgb_size_(0),
      image_(NULL),
      image_pub_(NULL),
      is_capturing_(false) {
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}
MipiCam::~MipiCam() { shutdown(); }

bool MipiCam::process_image(const void *src, int len, camera_image_t *dest) {
  // TODO(oal) return bool from all these
  if (pixelformat_ == PIXEL_FORMAT_YUYV) {
    if (monochrome_) {
      // actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy
      // if you don't use the advertised type (yuyv)
      mono102mono8(const_cast<char *>(reinterpret_cast<const char *>(src)),
                   dest->image,
                   dest->width * dest->height);
    } else {
      /*yuyv2rgb(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width *
        dest->height);
          */
      NV12_TO_BGR24((unsigned char *)src,
                    (unsigned char *)dest->image,
                    dest->width,
                    dest->height);
      dest->image_size = 3 * dest->width * dest->height;
    }
  } else if (pixelformat_ == PIXEL_FORMAT_UYVY) {
    uyvy2rgb(const_cast<char *>(reinterpret_cast<const char *>(src)),
             dest->image,
             dest->width * dest->height);
  } else if (pixelformat_ == PIXEL_FORMAT_RGB24) {
    rgb242rgb(const_cast<char *>(reinterpret_cast<const char *>(src)),
              dest->image,
              dest->width * dest->height);
  } else if (pixelformat_ == PIXEL_FORMAT_GREY) {
    memcpy(dest->image,
           const_cast<char *>(reinterpret_cast<const char *>(src)),
           dest->width * dest->height);
  }

  return true;
}

bool MipiCam::is_capturing() { return is_capturing_; }

bool MipiCam::stop_capturing(void) {
  if (!is_capturing_) {
    return false;
  }

  is_capturing_ = false;
  m_pMipiDev->StopStream();
  return true;
}

void MipiCam::GetCaptureHdl(struct TCapFrame *frame, void *user_args) {
  MipiCam *pThis = (MipiCam *)user_args;
}
bool MipiCam::start_capturing(void) {
  if (is_capturing_) {
    return false;
  }
  int nRet = m_pMipiDev->StartStream(GetCaptureHdl, this);
  is_capturing_ = nRet == 0 ? true : false;
  return true;
}

bool MipiCam::uninit_device(void) {
  unsigned int i;
  if (buffers_) {
    switch (io_) {
      case IO_METHOD_READ:
        free(buffers_[0].start);
        break;

      case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers_; ++i) {
          if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
            std::cerr << "error, quitting, TODO throw " << errno << std::endl;
            return false;  // ("munmap");
          }
        }
        break;

      case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers_; ++i) {
          free(buffers_[i].start);
        }
        break;
    }

    free(buffers_);
    buffers_ = NULL;
  }
  return true;
}

bool MipiCam::init_device(int image_width, int image_height, int framerate,
                          const std::string &gdc_file_path,
                          int rotate_degree) {
  unsigned int min;

  snprintf(m_oCamInfo.devName,
           sizeof(m_oCamInfo.devName),
           "%s",
           camera_dev_.c_str());
  m_oCamInfo.fps = framerate;
  m_oCamInfo.height = image_height;
  m_oCamInfo.width = image_width;
 // m_oCamInfo.need_gdc = !(gdc_file_path == "empty");
  m_oCamInfo.need_gdc = 0;
  m_oCamInfo.rotate_degree = rotate_degree;
  if (m_oCamInfo.need_gdc) {
    memcpy(m_oCamInfo.gdc_file_path,
           gdc_file_path.c_str(), gdc_file_path.size() + 1);
  }
  int nRet = m_pMipiDev->OpenCamera(&m_oCamInfo);
  ROS_printf("[%s]->cam %s ret=%d.\r\n", __func__, m_oCamInfo.devName, nRet);
  if (-3 == nRet) {  //发布分辨率不支持，返回失败
    return false;
  }
  if (-1 == nRet || -2 == nRet) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_cam"),
                        "Cannot open '" << camera_dev_ << "': " << errno << ", "
                                        << strerror(errno));
    return false;  // (EXIT_FAILURE);
  }

  return true;
}

bool MipiCam::close_device(void) {
  if (m_pMipiDev != NULL) {
    ROS_printf("[%s]->cam %p stop.\r\n", __func__, m_pMipiDev);
    m_pMipiDev->StopStream();
    delete m_pMipiDev;
    m_pMipiDev = NULL;
  }
  return true;
}

bool MipiCam::open_device(void) {
  m_pMipiDev = new MipiDevice();
  if (NULL == m_pMipiDev) {
    return false;  // (EXIT_FAILURE);
  }
  ROS_printf("[%s]->cam %p new.\r\n", __func__, m_pMipiDev);

  return true;
}
/*
支持格式：
"yuyv"= PIXEL_FORMAT_YUYV
  "uyvy" = PIXEL_FORMAT_UYVY
  "mjpeg" = PIXEL_FORMAT_MJPEG
  "yuvmono10" = PIXEL_FORMAT_YUVMONO10
  "rgb24" = PIXEL_FORMAT_RGB24
  "grey" = PIXEL_FORMAT_GREY
*/
bool MipiCam::start(const std::string &dev,
                    const std::string &outFormat,
                    io_method io_method,
                    pixel_format pixel_format,
                    int image_width,
                    int image_height,
                    int framerate,
                    const std::string &gdc_file_path,
                    int rotate_degree) {
  camera_dev_ = dev;
  out_format_ = outFormat;

  io_ = io_method;
  monochrome_ = false;
  if (dev.find("/video") != dev.npos) {
    // 说明找到了，当前不支持 usb  wuwlNG
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
                 "Start failed! Does not support USB camera. Please make sure "
                 "the video_device parameter is correct!");
    return false;
  }
  pixelformat_ = pixel_format;
  if (pixel_format == PIXEL_FORMAT_YUYV) {
  } else if (pixel_format == PIXEL_FORMAT_UYVY) {
  } else if (pixel_format == PIXEL_FORMAT_MJPEG) {
  } else if (pixel_format == PIXEL_FORMAT_YUVMONO10) {
    monochrome_ = true;
  } else if (pixel_format == PIXEL_FORMAT_RGB24) {
  } else if (pixel_format == PIXEL_FORMAT_GREY) {
    monochrome_ = true;
  }
  // TODO(oal) throw exceptions instead of return value checking
  if (!open_device()) {
    return false;
  }
  if (!init_device(image_width, image_height, framerate,
          gdc_file_path, rotate_degree)) {
    return false;
  }
  if (!start_capturing()) {
    return false;
  }
  image_ =
      reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));
  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;  // corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_pub_ =
      reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));
  memcpy((void *)image_pub_, (void *)image_, sizeof(camera_image_t));

  image_->image =
      reinterpret_cast<char *>(calloc(image_->image_size, sizeof(char *)));
  memset(image_->image, 0, image_->image_size * sizeof(char *));
  image_pub_->image =
      reinterpret_cast<char *>(calloc(image_pub_->image_size, sizeof(char *)));
  memset(image_pub_->image, 0, image_pub_->image_size * sizeof(char *));
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
              "[%s]->w:h=%d:%d.\n",
              __func__,
              image_width,
              image_height);
  return true;
}

bool MipiCam::shutdown(void) {
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "[%s]->start.\n", __func__);
  stop_capturing();
  uninit_device();
  close_device();

  if (image_) {
    free(image_);
  }
  image_ = NULL;
  if (image_pub_) {
    free(image_pub_);
  }
  image_pub_ = NULL;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "[%s]->end.\n", __func__);
  return true;
}

void TestSave(char *pFilePath, char *imgData, int nDlen) {
  FILE *yuvFd = fopen(pFilePath, "w+");
  if (yuvFd) {
    fwrite(imgData, 1, nDlen, yuvFd);
    fclose(yuvFd);
  }
}

static int s_nIdx = 0;
bool MipiCam::get_image(builtin_interfaces::msg::Time &stamp,
                        std::string &encoding,
                        uint32_t &height,
                        uint32_t &width,
                        uint32_t &step,
                        std::vector<uint8_t> &data) {
  uint64_t timestamp;
  if ((image_->width == 0) || (image_->height == 0)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("mipi_cam"),
        "Invalid publish width:%d height: %d! Please check the image_width "
        "and image_height parameters!",
        image_->width,
        image_->height);
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  if (m_pMipiDev->GetVpsFrame(
          2,
          &image_pub_->width,
          &image_pub_->height,
          reinterpret_cast<void **>(&image_->image),
          reinterpret_cast<unsigned int *>(&image_->image_size),
          timestamp))
    return false;
  stamp.sec = timestamp / 1e9;
  stamp.nanosec = timestamp - stamp.sec * 1e9;
  height = image_pub_->height;
  width = image_pub_->width;
  //这里出来都是 yuv 的
  step = width;
  if (0 == out_format_.compare("nv12")) {
    encoding = "nv12";
    data.resize(image_->image_size);  // step * height);
    memcpy(&data[0], image_->image, data.size());
  } else {
    if (monochrome_) {
      if (0 != out_format_.compare("mono8")) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("mipi_cam"),
                         "Invalid out_format: %s! Use mono8 instead!",
                         out_format_.c_str());
      }
      encoding = "mono8";
    } else {
      // TODO(oal) aren't there other encoding types?
      encoding = "bgr8";
      if (0 != out_format_.compare("bgr8")) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("mipi_cam"),
                         "Invalid out_format: %s! Use bgr8 instead!",
                         out_format_.c_str());
      }
      step = width * 3;
    }
    // jpeg，png---opencv 转 bgr8
    process_image(image_->image, image_->image_size, image_pub_);
    // eliminate this copy
    data.resize(image_pub_->image_size);  // step * height);
    memcpy(&data[0], image_pub_->image, data.size());
  }
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
              "[%s]->enc=%s,step=%d, w:h=%d:%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.c_str(),
              step,
              width,
              height,
              image_->image_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCam::get_image_mem(
    // uint64_t & stamp,
    builtin_interfaces::msg::Time &stamp,
    std::array<uint8_t, 12> &encoding,
    uint32_t &height,
    uint32_t &width,
    uint32_t &step,
    std::array<uint8_t, 6220800> &data,
    uint32_t &data_size) {
  uint64_t timestamp;
  if ((image_->width == 0) || (image_->height == 0)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("mipi_cam"),
        "Invalid publish width:%d height: %d! Please check the image_width "
        "and image_height parameters!",
        image_->width,
        image_->height);
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  uint64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  if (m_pMipiDev->GetVpsFrame(
          2,
          &image_pub_->width,
          &image_pub_->height,
          reinterpret_cast<void **>(&image_->image),
          reinterpret_cast<unsigned int *>(&image_->image_size), timestamp))
    return false;
  stamp.sec = timestamp / 1e9;
  stamp.nanosec = timestamp - stamp.sec * 1e9;

  height = image_pub_->height;
  width = image_pub_->width;
  //这里出来都是 yuv 的
  step = width;
  if (0 == out_format_.compare("nv12")) {
    memcpy(encoding.data(), "nv12", strlen("nv12"));
    data_size = image_->image_size;  // step * height);
    memcpy(data.data(), image_->image, data_size);
  } else {
    if (monochrome_) {
      memcpy(encoding.data(), "mono8", strlen("mono8"));
      RCLCPP_WARN_ONCE(rclcpp::get_logger("mipi_cam"),
                       "Invalid out_format: %s! Use mono8 instead!",
                       out_format_.c_str());
    } else {
      // TODO(oal) aren't there other encoding types?
      memcpy(encoding.data(), "bgr8", strlen("bgr8"));
      // RCLCPP_WARN_ONCE(rclcpp::get_logger("mipi_cam"),
      //                  "Invalid out_format: %s! Use bgr8 instead!",
      //                  out_format_.c_str());
      step = width * 3;
    }
    // jpeg，png---opencv 转 bgr8
    process_image(image_->image, image_->image_size, image_pub_);
    // eliminate this copy
    data_size = image_pub_->image_size;  // step * height);
    memcpy(data.data(), image_pub_->image, data_size);
  }
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
              "[%s]->hbmem enc=%s,step=%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.data(),
              step,
              image_->image_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCam::get_cam_calibration(sensor_msgs::msg::CameraInfo &cam_info,
                                  const std::string &file_path) {
  try {
    std::string camera_name;
    std::ifstream fin(file_path.c_str());
    if (!fin) {
      RCLCPP_ERROR(
          rclcpp::get_logger("mipi_cam"),
          "Camera calibration file: %s not exist! Please make sure the "
          "calibration file path is correct and the calibration file exists!",
          file_path.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    if (calibration_doc["camera_name"]) {
      camera_name = calibration_doc["camera_name"].as<std::string>();
    } else {
      camera_name = "unknown";
    }
    cam_info.width = calibration_doc["image_width"].as<int>();
    cam_info.height = calibration_doc["image_height"].as<int>();

    const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
    const YAML::Node &camera_matrix_data = camera_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.k[i] = camera_matrix_data[i].as<double>();
    }
    const YAML::Node &rectification_matrix =
        calibration_doc["rectification_matrix"];
    const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.r[i] = rectification_matrix_data[i].as<double>();
    }
    const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
    const YAML::Node &projection_matrix_data = projection_matrix["data"];
    for (int i = 0; i < 12; i++) {
      cam_info.p[i] = projection_matrix_data[i].as<double>();
    }

    if (calibration_doc["distortion_model"]) {
      cam_info.distortion_model =
          calibration_doc["distortion_model"].as<std::string>();
    } else {
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
                  "Camera calibration file did not specify distortion model, "
                  "assuming plumb bob");
    }
    const YAML::Node &distortion_coefficients =
        calibration_doc["distortion_coefficients"];
    int d_rows, d_cols;
    d_rows = distortion_coefficients["rows"].as<int>();
    d_cols = distortion_coefficients["cols"].as<int>();
    const YAML::Node &distortion_coefficients_data =
        distortion_coefficients["data"];
    cam_info.d.resize(d_rows * d_cols);
    for (int i = 0; i < d_rows * d_cols; ++i) {
      cam_info.d[i] = distortion_coefficients_data[i].as<double>();
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
                "[get_cam_calibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("mipi_cam"),
                "Unable to parse camera calibration file normally:%s",
                e.what());
    return false;
  }
}

void MipiCam::get_formats()  // std::vector<mipi_cam::msg::Format>& formats)
{}

MipiCam::io_method MipiCam::io_method_from_string(const std::string &str) {
  // all did't support
  return IO_METHOD_UNKNOWN;
}

MipiCam::pixel_format MipiCam::pixel_format_from_string(
    const std::string &str) {
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}
}  // namespace mipi_cam
