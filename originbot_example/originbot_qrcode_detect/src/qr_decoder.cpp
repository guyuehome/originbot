#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "zbar.h"
#include <std_msgs/msg/string.hpp>

class QrCodeDetection : public rclcpp::Node
{
public:
  QrCodeDetection() : Node("qr_code_detection")
  {
    RCLCPP_INFO(this->get_logger(), "qr_code_detection Start");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&QrCodeDetection::imageCallback, this, std::placeholders::_1));

    qr_code_pub_ = this->create_publisher<std_msgs::msg::String>("qr_code_result", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("qr_code_image", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat frame = cv_ptr->image;
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      zbar::ImageScanner scanner;
      scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
      zbar::Image zbar_image(frame.cols, frame.rows, "Y800", (uchar *)gray.data, frame.cols * frame.rows);
      scanner.scan(zbar_image);

      for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
           symbol != zbar_image.symbol_end(); ++symbol) {
        std::string qr_code_data = symbol->get_data();
        RCLCPP_INFO(this->get_logger(), "Scanned QR Code: %s", qr_code_data.c_str());

        // 发布二维码数据
        auto qr_code_msg = std_msgs::msg::String();
        qr_code_msg.data = qr_code_data;
        qr_code_pub_->publish(qr_code_msg);

        // 在图像上绘制二维码边界和信息
        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); i++) {
          points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }
        cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);

        cv::Point text_origin = points[0];
        text_origin.y -= 10;  // 将文本位置稍微上移
        cv::putText(frame, qr_code_data, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      }

      // 发布带有二维码标记的图像
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      image_pub_->publish(*out_img);
    }
    catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_code_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrCodeDetection>());
  rclcpp::shutdown();
  return 0;
}