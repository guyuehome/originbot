#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>


#define X_CENTER_MAX (380)
#define X_CENTER_MIN (270)

#define Z_SIZE_MAX  (3500)
#define Z_SIZE_MIN  (20000)

#define X_CENTER_AVERAGE  (X_CENTER_MAX + X_CENTER_MIN) / 2
#define Z_SIZE_AVERAGE  (Z_SIZE_MAX + Z_SIZE_MIN) / 2


class QrCodeControl : public rclcpp::Node
{
public:
    QrCodeControl() : Node("qrcode_control")
    {
        RCLCPP_INFO(this->get_logger(), "qrcode_control Start");
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_qrcode_info_ = this->create_subscription<std_msgs::msg::String>(
            "qr_code_result", 10, std::bind(&QrCodeControl::handleQrcodeInfo, this, std::placeholders::_1));
        sub_qrcode_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/qrcode_detected/pose_result", 1, std::bind(&QrCodeControl::handleQrcodePose, this, std::placeholders::_1));

        this->declare_parameter<bool>("control_with_qrcode_info", true);
    }

private:
    void handleQrcodeInfo(const std_msgs::msg::String::SharedPtr qrcode_info)
    {
        bool control_with_qrcode_info = this->get_parameter("control_with_qrcode_info").as_bool();
        if (control_with_qrcode_info) {
            setTwistWithQrInfo(qrcode_info->data);
        }
    }

    void handleQrcodePose(const geometry_msgs::msg::Pose::SharedPtr qrcode_pose)
    {
        bool control_with_qrcode_info = this->get_parameter("control_with_qrcode_info").as_bool();
        if (!control_with_qrcode_info) {
            setTwistWithQrPose(*qrcode_pose);
        }
    }

    void pubControlCommand()
    {
        pub_cmd_vel_->publish(twist_);
        RCLCPP_INFO(this->get_logger(), "Publish velocity command [%f m/s, %f rad/s]", twist_.linear.x, twist_.angular.z);
        rclcpp::sleep_for(std::chrono::milliseconds(150));
        setTwist(0.0, 0.0);
        pub_cmd_vel_->publish(twist_);    
    }

    void setTwist(double linear_x, double angular_z)
    {
        twist_.linear.x = std::clamp(linear_x, -1.0, 1.0);
        twist_.angular.z = std::clamp(angular_z, -1.0, 1.0);
    }

    void setTwistWithQrInfo(const std::string &info)
    {
        if (info.empty()) return; // Corrected from isempty to empty

        if (info.find("Front") != std::string::npos) {
            setTwist(0.5, 0.0);
        } else if (info.find("Back") != std::string::npos) {
            setTwist(-0.5, 0.0);
        } else if (info.find("Left") != std::string::npos) {
            setTwist(0.0, 0.5);
        } else if (info.find("Right") != std::string::npos) {
            setTwist(0.0, -0.5);
        } else {
            setTwist(0.0, 0.0);
        }
        pubControlCommand();
    }

    void setTwistWithQrPose(const geometry_msgs::msg::Pose &pose)
    {
        double qrcode_x = pose.position.x;
        double qrcode_size = pose.position.z;
        setTwistWithX(qrcode_x);
        setTwistWithZ(qrcode_size);
        pubControlCommand();
    }

    void setTwistWithX(double qrcode_x)
    {
        if (qrcode_x > X_CENTER_MIN && qrcode_x < X_CENTER_MAX) {
            setTwist(twist_.linear.x, 0.0);
        } else if (qrcode_x < X_CENTER_MIN || qrcode_x > X_CENTER_MAX) {
            setTwist(twist_.linear.x, 1.0 - qrcode_x / X_CENTER_AVERAGE);
        } else {
            RCLCPP_INFO(this->get_logger(), "No X, cannot control!");
        }
    }

    void setTwistWithZ(double qrcode_size)
    {
        if (qrcode_size >= Z_SIZE_MIN && qrcode_size <= Z_SIZE_MAX) {
            setTwist(0.0, twist_.angular.z);
        } else if (qrcode_size < Z_SIZE_MIN || qrcode_size > Z_SIZE_MAX) {
            setTwist((1.0 - qrcode_size / Z_SIZE_AVERAGE) * 0.8, twist_.angular.z);
        } else {
            RCLCPP_INFO(this->get_logger(), "No Z, cannot control!");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_qrcode_info_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_qrcode_pose_;
    geometry_msgs::msg::Twist twist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Corrected from rclpy to rclcpp
    auto node = std::make_shared<QrCodeControl>();
    rclcpp::spin(node); // Corrected from rclpy to rclcpp
    rclcpp::shutdown(); // Corrected from rclpy to rclcpp
    return 0;
}