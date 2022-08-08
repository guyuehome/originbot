#include "originbot_driver.h"

originbot_driver::originbot_driver() : Node("originbot_driver")
{
    std::string port_name="ttyUSB0";    
    this->declare_parameter("port_name");   //声明参数
    this->get_parameter_or<std::string>("port_name", port_name, "ttyUSB0");//获取参数
    std::cout << "Loading parameters: " << std::endl;
    std::cout << "- port name: " << port_name << std::endl;

    // 创建里程计、IMU的发布者、速度指令的订阅者和TF广播器
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&originbot_driver::cmd_vel_callback, this, _1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 初始化机器人的初始位置
    x_ = 0;
    y_ = 0;
    th_= 0;

    // 控制器与扩展驱动板卡的串口配置与通信
    try
    {
        ///尝试初始化与开启串口
        serial_.setPort("/dev/" + port_name);                     //选择要开启的串口号
        serial_.setBaudrate(115200);                              //设置波特率
        serial_.setTimeout(serial::Timeout::simpleTimeout(2000)); //超时等待
        serial_.open();                                           //开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "originbot can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "originbot serial port opened"); //串口开启成功提示
    }

    // 启动定时器，按照固定频率更新里程计和IMU的话题
    odom_timer_ = this->create_wall_timer(50ms, std::bind(&originbot_driver::odom_publisher, this));
    imu_timer_  = this->create_wall_timer(50ms, std::bind(&originbot_driver::imu_publisher, this));
}

originbot_driver::~originbot_driver()
{
    serial_.close();
}

void originbot_driver::driver_loop()
{
    float speed[2] = {0.0, 0.0};
    float vx = 0, vth = 0;
    float delta_th = 0, delta_x = 0, delta_y = 0, vth = 0;
    float linear_acceleration[3] = {0.0}, angular_velocity[3] = {0.0}, euler[3] = {0.0};

    last_time_ = this->now();
    while (rclcpp::ok())
    {
        float dt = (current_time_.seconds() - last_time_.seconds());
        current_time_ = this->now();

        if (get_sensor_data())
        {
            RCLCPP_INFO(this->get_logger(), "Update data successful");

            // 获取机器人两侧轮子的速度，单位m/s
            if (sensor_data_raw_[3] == 0xFF)
                speed[0] = 1.0 * sensor_data_raw_[4] / 100.0;
            else
                speed[0] = sensor_data_raw_[4] * -1.0 / 100.0;

            if (sensor_data_raw_[5] == 0xFF)
                speed[1] = 1.0 * sensor_data_raw_[6] / 100.0;
            else
                speed[1] = sensor_data_raw_[6] * -1.0 / 100.0;

            // 在原地旋转时减小误差
            if (abs(speed[0] + speed[1]) <= 0.03 && speed[0] * speed[1] < 0)
            {
                if (speed[0] > 0)
                    speed[1] = -1.0 * speed[0];
                else
                    speed[0] = -1.0 * speed[1];
            }

            // 通过两侧轮子的速度，计算机器人整体的线速度和角速度
            vx  = (speed[0] + speed[1]) / 2;
            vth = (speed[1] - speed[0]) / ORIGINBOT_WHEEL_TRACK;
            // if (vth < -1)
            //     vth = -1;
            // if (vth > 1)
            //     vth = 1;
            RCLCPP_INFO(this->get_logger(), "dt=%f 左轮速度=%f 右轮速度=%f 线速度=%f 角速度=%f", dt, speed[0], speed[1], vx, vth);

            // 里程计的微分量计算
            delta_x = vx * cos(th_) * dt;
            delta_y = vx * sin(th_) * dt;
            delta_th = vth * dt;

            // 里程积分计算
            x_ = x_ + delta_x;
            y_ = y_ + delta_y;
            th_ = th_ + delta_th;
            //  if(th_>3.1415926) th_-=3.1415926*2;
            //  if(th_<-3.1415926) th_+=3.1415926*2;

            RCLCPP_INFO(this->get_logger(), "x=%f y=%f th=%f delta_x=%f delta_y=%f,delta_th=%f", x_, y_, th_, delta_x, delta_y, delta_th);

            odom_publisher(vx, vth);

            // 获取机器人IMU的数据
            // linear_acceleration[0]=imu_conversion(sensor_data_raw_[13],sensor_data_raw_[12])/1671.84;
            // linear_acceleration[1]=imu_conversion(sensor_data_raw_[15],sensor_data_raw_[14])/1671.84;
            // linear_acceleration[2]=imu_conversion(sensor_data_raw_[17],sensor_data_raw_[16])/1671.84;
            // angular_velocity[0]=imu_conversion(sensor_data_raw_[24],sensor_data_raw_[23])*0.00026644;
            // angular_velocity[1]=imu_conversion(sensor_data_raw_[26],sensor_data_raw_[25])*0.00026644;
            // angular_velocity[2]=imu_conversion(sensor_data_raw_[28],sensor_data_raw_[27])*0.00026644;
            // euler[0]=imu_conversion(sensor_data_raw_[35],sensor_data_raw_[34]);
            // euler[1]=imu_conversion(sensor_data_raw_[37],sensor_data_raw_[36]);
            // euler[2]=imu_conversion(sensor_data_raw_[39],sensor_data_raw_[38]);
            linear_acceleration[0] = imu_conversion(sensor_data_raw_[12], sensor_data_raw_[13]) / 1671.84;
            linear_acceleration[1] = imu_conversion(sensor_data_raw_[14], sensor_data_raw_[15]) / 1671.84;
            linear_acceleration[2] = imu_conversion(sensor_data_raw_[16], sensor_data_raw_[17]) / 1671.84;
            angular_velocity[0] = imu_conversion(sensor_data_raw_[23], sensor_data_raw_[24]) * 0.00026644;
            angular_velocity[1] = imu_conversion(sensor_data_raw_[25], sensor_data_raw_[26]) * 0.00026644;
            angular_velocity[2] = imu_conversion(sensor_data_raw_[27], sensor_data_raw_[28]) * 0.00026644;
            euler[0] = imu_conversion(sensor_data_raw_[34], sensor_data_raw_[35]) * 0.00026644;
            euler[1] = imu_conversion(sensor_data_raw_[36], sensor_data_raw_[37]) * 0.00026644;
            euler[2] = imu_conversion(sensor_data_raw_[38], sensor_data_raw_[39]) * 0.00026644;
            // RCLCPP_INFO(this->get_logger(), "二进制下imu数据 %f %f %f %f %f %f %f %f %f ",linear_acceleration[0],linear_acceleration[1],linear_acceleration[2],angular_velocity[0],angular_velocity[1],angular_velocity[2],euler[0],euler[1],euler[2]);

            imu_publisher(linear_acceleration, angular_velocity, euler);

            last_time_ = current_time_;
        }

        rclcpp::spin_some(shared_from_this());
    }
}

void originbot_driver::odom_publisher(float vx, float vth)
{
    auto odom_msg = nav_msgs::msg::Odometry();

    //里程数据计算
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
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

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q[0];
    t.transform.rotation.y = q[1];
    t.transform.rotation.z = q[2];
    t.transform.rotation.w = q[3];

    tf_broadcaster_->sendTransform(t);
}

void originbot_driver::imu_publisher(float linear_acceleration[3], float angular_velocity[3], float euler[3])
{
    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();

    imu_msg.linear_acceleration.x = linear_acceleration[0];
    imu_msg.linear_acceleration.y = linear_acceleration[1];
    imu_msg.linear_acceleration.z = linear_acceleration[2];

    imu_msg.angular_velocity.x = angular_velocity[0];
    imu_msg.angular_velocity.y = angular_velocity[1];
    imu_msg.angular_velocity.z = angular_velocity[2];

    tf2::Quaternion q;
    q.setRPY(euler[0], euler[1], euler[2]);

    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];
    imu_msg.orientation.w = q[3];

    imu_msg.linear_acceleration_covariance = {0.04, 0.00, 0.00, 0.00, 0.04, 0.00, 0.00, 0.00, 0.04};

    imu_msg.angular_velocity_covariance = {0.02, 0.00, 0.00, 0.00, 0.02, 0.00, 0.00, 0.00, 0.02};

    imu_msg.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025, 0.0000, 0.0000, 0.0000, 0.0025};

    // RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");
    imu_publisher_->publish(imu_msg);
}

void originbot_driver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    int left_motor_speed = 0, right_motor_speed = 0;

    int leftFlag = 0xff, rightFlag = 0xff;
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

    RCLCPP_INFO(this->get_logger(), "leftSpeed = '%f'   rightSpeed = '%f'", leftSpeed * 100, rightSpeed * 100);

    if (leftSpeed < 0)
        leftFlag = 0x00;
    else
        leftFlag = 0xff;
    left_motor_speed = int(abs(leftSpeed) * 100); //速度值从m/s变为cm/s

    if (rightSpeed < 0)
        rightFlag = 0x00;
    else
        rightFlag = 0xff;
    right_motor_speed = int(abs(rightSpeed) * 100);

    int dataCheck = (leftFlag + left_motor_speed + rightFlag + right_motor_speed) & 0xff;

    //一组数据
    uint8_t data[9] = {0x55, 0x01, 0x04, leftFlag, left_motor_speed, rightFlag, right_motor_speed, dataCheck, 0xbb};
    try
    {
        serial_.write(data, sizeof(data)); //向串口发数据，发送时int转bytes
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }
}

int originbot_driver::find_data(uint8_t ar[], int n, int element) //查找元素并返回位置下标,find(数组，长度，元素)
{
    int i = 0;
    int index = -1; //原始下标，没找到元素返回-1

    for (i = 0; i < n; i++)
    {
        if (element == ar[i] && ar[i + 1] == 0x02)
        {
            index = i;    //记录元素下标
            return index; //返回下标
        }
    }
}

float originbot_driver::imu_conversion(uint8_t data_high, uint8_t data_low)
{
    short transition_16;
    
    transition_16 = 0;
    transition_16 |= data_high << 8;
    transition_16 |= data_low;

    return transition_16;
}

bool originbot_driver::get_sensor_data()
{
    uint8_t  Receive_Data_Pr[100] = {0};  //临时变量，保存>下位机数据

    serial_.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); //通过串口读取下位机发送过来的数据
    int index = find_data(Receive_Data_Pr, 100, 0x55);

    for (int i = index, j = 0; j < 53; i++, j++)
    {
        sensor_data_raw_[j] = Receive_Data_Pr[i];
    }

    if (sensor_data_raw_[0] == 0x55 && sensor_data_raw_[52] == 0xBB)
    {
        //   RCLCPP_INFO(this->get_logger(), "速度反馈 %X %X %X %X %X %X %X %X %X",Receive_Data_Pr[index],Receive_Data_Pr[index+1],Receive_Data_Pr[index+2],Receive_Data_Pr[index+3],Receive_Data_Pr[index+4],Receive_Data_Pr[index+5],Receive_Data_Pr[index+6],Receive_Data_Pr[index+7],Receive_Data_Pr[index+8]);
        //   index+=9;
        //   RCLCPP_INFO(this->get_logger(), "陀螺仪加速度 %X %X %X %X %X %X %X %X %X %X %X",Receive_Data_Pr[index],Receive_Data_Pr[index+1],Receive_Data_Pr[index+2],Receive_Data_Pr[index+3],Receive_Data_Pr[index+4],Receive_Data_Pr[index+5],Receive_Data_Pr[index+6],Receive_Data_Pr[index+7],Receive_Data_Pr[index+8],Receive_Data_Pr[index+9],Receive_Data_Pr[index+10]);
        //     index+=11;
        //   RCLCPP_INFO(this->get_logger(), "陀螺仪角速度 %X %X %X %X %X %X %X %X %X %X %X",Receive_Data_Pr[index],Receive_Data_Pr[index+1],Receive_Data_Pr[index+2],Receive_Data_Pr[index+3],Receive_Data_Pr[index+4],Receive_Data_Pr[index+5],Receive_Data_Pr[index+6],Receive_Data_Pr[index+7],Receive_Data_Pr[index+8],Receive_Data_Pr[index+9],Receive_Data_Pr[index+10]);
        //      index+=11;
        //  RCLCPP_INFO(this->get_logger(), "陀螺仪欧拉角 %X %X %X %X %X %X %X %X %X %X %X",Receive_Data_Pr[index],Receive_Data_Pr[index+1],Receive_Data_Pr[index+2],Receive_Data_Pr[index+3],Receive_Data_Pr[index+4],Receive_Data_Pr[index+5],Receive_Data_Pr[index+6],Receive_Data_Pr[index+7],Receive_Data_Pr[index+8],Receive_Data_Pr[index+9],Receive_Data_Pr[index+10]);
        //       index+=11;
        //   RCLCPP_INFO(this->get_logger(), "传感器数据 %X %X %X %X %X %X %X %X %X %X %X",Receive_Data_Pr[index],Receive_Data_Pr[index+1],Receive_Data_Pr[index+2],Receive_Data_Pr[index+3],Receive_Data_Pr[index+4],Receive_Data_Pr[index+5],Receive_Data_Pr[index+6],Receive_Data_Pr[index+7],Receive_Data_Pr[index+8],Receive_Data_Pr[index+9],Receive_Data_Pr[index+10]);

        return true;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<originbot_driver>());
    auto originbot_driver = std::make_shared<originbot_driver>();
    originbot_driver->driver_loop();
    rclcpp::shutdown();

    return 0;
}