#include "originbot_base.h"

OriginbotBase::OriginbotBase(std::string nodeName) : Node(nodeName)
{
    // 加载参数
    std::string port_name="ttyUSB0";    
    this->declare_parameter("port_name");   //声明参数
    this->get_parameter_or<std::string>("port_name", port_name, "ttyUSB0");//获取参数
    printf("Loading parameters: \n - port name: %s\n", port_name.c_str()); 

    // 创建里程计、IMU的发布者、速度指令的订阅者和TF广播器
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OriginbotBase::cmd_vel_callback, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 控制器与扩展驱动板卡的串口配置与通信
    try
    {
        serial_.setPort("/dev/" + port_name);                            //选择要开启的串口号
        serial_.setBaudrate(115200);                                     //设置波特率
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000);  //超时等待
        serial_.setTimeout(timeOut);                                     
        serial_.open();                                                  //开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "originbot can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    // 如果串口打开，则驱动读取数据的线程
    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "originbot serial port opened"); //串口开启成功提示

        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&OriginbotBase::readRawData, this)));
    }

    RCLCPP_INFO(this->get_logger(), "originbot Start ...");
}

OriginbotBase::~OriginbotBase()
{
    serial_.close();
}

void OriginbotBase::readRawData()
{
    uint8_t rx_data = 0;
    DataFrame frame;

    while (rclcpp::ok()) 
    {
        auto len = serial_.read(&rx_data, 1);
        if (len < 1)
            continue;
        
        // 发现帧头
        if(rx_data == 0x55)
        {
            // 读取完整的数据帧
            serial_.read(&frame.id, 10);

            // 判断帧尾是否正确
            if(frame.tail != 0xbb)
            {
                RCLCPP_WARN(this->get_logger(), "Data frame tail error!"); 
                printf("Frame raw data[Error]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                        frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                        frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);
                continue;
            }

            frame.header = 0x55;
            
            //帧校验
            if(checkDataFrame(frame))
            {
                // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                //         frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                //         frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

                //处理帧数据
                processDataFrame(frame);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Data frame check failed!"); 
            }
        }
    }
}

bool OriginbotBase::checkDataFrame(DataFrame &frame)
{    
    if(((frame.data[0] + frame.data[1] + frame.data[2] + 
        frame.data[3] + frame.data[4] + frame.data[5]) & 0xff) == frame.check)
        return true;
    else
        return false;
}

void OriginbotBase::processDataFrame(DataFrame &frame)
{
    switch(frame.id)
    {
    case FRAME_ID_VELOCITY:
        processVelocityData(frame);
        break;
    case FRAME_ID_ACCELERATION:
        processAccelerationData(frame);
        break;
    case FRAME_ID_ANGULAR:
        processAngularData(frame);
        break;
    case FRAME_ID_EULER:
        processEulerData(frame);
        break;
    case FRAME_ID_SENSOR:
        processSensorData(frame);
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[%d]", frame.id);
        break;
    }
}

void OriginbotBase::processVelocityData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process velocity data");

    float left_speed = 0.0, right_speed = 0.0;
    float vx = 0.0, vth = 0.0;
    float delta_th = 0.0, delta_x = 0.0, delta_y = 0.0;

    static rclcpp::Time last_time_ = this->now();
    current_time_ = this->now();

    float dt = (current_time_.seconds() - last_time_.seconds());
    last_time_ = current_time_;    

    // 获取机器人两侧轮子的速度，单位mm/s --> m/s
    uint16_t dataTemp = frame.data[2];
    float speedTemp = (float)((dataTemp << 8) | frame.data[1]);
    if (frame.data[0] == 0)
        left_speed = -1.0 * speedTemp / 1000.0;
    else
        left_speed = speedTemp / 1000.0;

    dataTemp = frame.data[5];
    speedTemp = (float)((dataTemp << 8) | frame.data[4]);
    if (frame.data[3] == 0)
        right_speed = -1.0 * speedTemp / 1000.0;
    else
        right_speed = speedTemp / 1000.0;

    // 通过两侧轮子的速度，计算机器人整体的线速度和角速度
    vx  = (left_speed  + right_speed) / 2;                    //m/s
    vth = (right_speed - left_speed) / ORIGINBOT_WHEEL_TRACK; //rad/s

    //RCLCPP_INFO(this->get_logger(), "dt=%f left_speed=%f right_speed=%f vx=%f vth=%f", dt, left_speed, right_speed, vx, vth);

    // 里程计的微分量计算
    delta_x = vx * cos(odom_th_) * dt;
    delta_y = vx * sin(odom_th_) * dt;
    delta_th = vth * dt;

    // 里程积分计算
    odom_x_  += delta_x;
    odom_y_  += delta_y;
    odom_th_ += delta_th;
    
    // 校正姿态角度，让机器人处于-180~180度之间
    if(odom_th_ > M_PI) 
        odom_th_ -= M_PI*2;
    else if(odom_th_ < (-M_PI)) 
        odom_th_ += M_PI*2;

    //RCLCPP_INFO(this->get_logger(), "x=%f y=%f th=%f delta_x=%f delta_y=%f,delta_th=%f", odom_x_, odom_y_, odom_th_, delta_x, delta_y, delta_th);

    odom_publisher(vx, vth);    
}

double OriginbotBase::degToRad(double deg) 
{
    return deg / 180.0 * M_PI;
}

void OriginbotBase::processAccelerationData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process acceleration data");

    imuData_.acceleration_x = imu_conversion(frame.data[1], frame.data[0]) / 32768 * 16 * 9.8;
    imuData_.acceleration_y = imu_conversion(frame.data[3], frame.data[2]) / 32768 * 16 * 9.8;
    imuData_.acceleration_z = imu_conversion(frame.data[5], frame.data[4]) / 32768 * 16 * 9.8;
}

void OriginbotBase::processAngularData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process angular data");

    imuData_.angular_x = imu_conversion(frame.data[1], frame.data[0]) / 32768 * degToRad(2000);
    imuData_.angular_y = imu_conversion(frame.data[3], frame.data[2]) / 32768 * degToRad(2000);
    imuData_.angular_z = imu_conversion(frame.data[5], frame.data[4]) / 32768 * degToRad(2000);
}

void OriginbotBase::processEulerData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process euler data");

    imuData_.roll  = imu_conversion(frame.data[1], frame.data[0]) / 32768 * degToRad(180);
    imuData_.pitch = imu_conversion(frame.data[3], frame.data[2]) / 32768 * degToRad(180);
    imuData_.yaw   = imu_conversion(frame.data[5], frame.data[4]) / 32768 * degToRad(180);

    imu_publisher();
}

void OriginbotBase::processSensorData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process sensor data");

    frame = frame;
}

void OriginbotBase::odom_publisher(float vx, float vth)
{
    auto odom_msg = nav_msgs::msg::Odometry();

    //里程数据计算
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_th_);
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.orientation.x = q[0];
    odom_msg.pose.pose.orientation.y = q[1];
    odom_msg.pose.pose.orientation.z = q[2];
    odom_msg.pose.pose.orientation.w = q[3];

    const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,

                                             0, 1e-3, 0, 0, 0, 0,

                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,

                                             0, 0, 0, 0, 1e6, 0,

                                             0, 0, 0, 0, 0, 1e-9};
    const double odom_pose_covariance2[36]= {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.00;
    odom_msg.twist.twist.linear.z = 0.00;

    odom_msg.twist.twist.angular.x = 0.00;
    odom_msg.twist.twist.angular.y = 0.00;
    odom_msg.twist.twist.angular.z = vth;

    const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                              0, 1e-3, 1e-9, 0, 0, 0,

                                              0, 0, 1e6, 0, 0, 0,

                                              0, 0, 0, 1e6, 0, 0,

                                              0, 0, 0, 0, 1e6, 0,

                                              0, 0, 0, 0, 0, 1e-9};
    const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                        0, 1e-3, 1e-9,   0,   0,    0,
                                        0,    0,  1e6,   0,   0,    0,
                                        0,    0,    0, 1e6,   0,    0,
                                        0,    0,    0,   0, 1e6,    0,
                                        0,    0,    0,   0,   0, 1e-9};

    // RCLCPP_INFO(this->get_logger(), "%f %f %f",x,y,th);

    if (vx == 0 && vth == 0)
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q[0];
    t.transform.rotation.y = q[1];
    t.transform.rotation.z = q[2];
    t.transform.rotation.w = q[3];

    tf_broadcaster_->sendTransform(t);
}

void OriginbotBase::imu_publisher()
{
    //RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");

    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();

    imu_msg.linear_acceleration.x = imuData_.acceleration_x;
    imu_msg.linear_acceleration.y = imuData_.acceleration_y;
    imu_msg.linear_acceleration.z = imuData_.acceleration_z;

    imu_msg.angular_velocity.x = imuData_.angular_x;
    imu_msg.angular_velocity.y = imuData_.angular_y;
    imu_msg.angular_velocity.z = imuData_.angular_z;

    tf2::Quaternion q;
    q.setRPY(imuData_.roll, imuData_.pitch, imuData_.yaw);

    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];
    imu_msg.orientation.w = q[3];

    imu_msg.linear_acceleration_covariance = {0.04, 0.00, 0.00, 0.00, 0.04, 0.00, 0.00, 0.00, 0.04};

    imu_msg.angular_velocity_covariance = {0.02, 0.00, 0.00, 0.00, 0.02, 0.00, 0.00, 0.00, 0.02};

    imu_msg.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025, 0.0000, 0.0000, 0.0000, 0.0025};

    imu_publisher_->publish(imu_msg);
}

void OriginbotBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    DataFrame cmdFrame;
    float leftSpeed = 0.0, rightSpeed = 0.0;

    float x_linear = msg->linear.x;
    float z_angular = msg->angular.z;

    //差分轮运动学模型求解，三种情况
    if (x_linear == 0)
    {
        rightSpeed = z_angular * ORIGINBOT_WHEEL_TRACK / 2.0;
        leftSpeed = (-1) * rightSpeed;
    }
    else if (z_angular == 0)
    {
        leftSpeed = x_linear;
        rightSpeed = x_linear;
    }
    else
    {
        leftSpeed = x_linear - z_angular * ORIGINBOT_WHEEL_TRACK / 2.0;
        rightSpeed = x_linear + z_angular * ORIGINBOT_WHEEL_TRACK / 2.0;
    }

    RCLCPP_INFO(this->get_logger(), "leftSpeed = '%f' rightSpeed = '%f'", leftSpeed * 100, rightSpeed * 100);

    if (leftSpeed < 0)
        cmdFrame.data[0] = 0x00;
    else
        cmdFrame.data[0] = 0xff;
    cmdFrame.data[1] = int(abs(leftSpeed) * 1000) & 0xff;         //速度值从m/s变为mm/s
    cmdFrame.data[2] = (int(abs(leftSpeed) * 1000) >> 8) & 0xff;

    if (rightSpeed < 0)
        cmdFrame.data[3] = 0x00;
    else
        cmdFrame.data[3] = 0xff;
    cmdFrame.data[4] = int(abs(rightSpeed) * 1000) & 0xff;        //速度值从m/s变为mm/s
    cmdFrame.data[5] = (int(abs(rightSpeed) * 1000) >> 8) & 0xff;

    cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] + 
                      cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) & 0xff;

    //一组数据
    cmdFrame.header = 0x55;
    cmdFrame.id     = 0x01;
    cmdFrame.length = 0x06;
    cmdFrame.tail   = 0xbb;
    try
    {
        serial_.write(&cmdFrame.header, sizeof(cmdFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }
}

double OriginbotBase::imu_conversion(uint8_t data_high, uint8_t data_low)
{
    short transition_16;
    
    transition_16 = 0;
    transition_16 |= data_high << 8;
    transition_16 |= data_low;

    return transition_16;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OriginbotBase>("originbot_base"));
    
    rclcpp::shutdown();

    return 0;
}