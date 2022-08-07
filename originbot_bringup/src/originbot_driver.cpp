#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <serial/serial.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include<algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

float wheel_track = 0.1;
float x=0.0,y=0.0,th=0.0;
typedef struct _RECEIVE_DATA_
{
          uint8_t rx[9];//定义长度
          uint8_t Flag_Stop;//标识位
                unsigned char Frame_Header;//帧头
                float left_dirct;  //左电机速度方向
                float left_speed;  //左电机速度
                float right_dirct;  //右电机速度方向
                float right_speed;  //右电机速度
    unsigned char flag_test;//校验位
                unsigned char Frame_Tail;//帧尾
}RECEIVE_DATA;

class originbot_driver : public rclcpp::Node
{
  serial::Serial Stm32_Serial;
  public:

    originbot_driver():Node("originBot_driver")
    {
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&originbot_driver::cmd_vel_callback, this, _1));

      odom_timer_ = this->create_wall_timer(50ms, std::bind(&originbot_driver::odom_callback, this));
      imu_timer_ = this->create_wall_timer(50ms, std::bind(&originbot_driver::imu_callback, this));

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      std::string usart_port_name="/dev/ttyUSB0";
      int serial_baud_rate=115200;

      try
      {
        //Attempts to initialize and open the serial port //尝试初始化与开启串口
                Stm32_Serial.setPort("/dev/ttyUSB0"); //Select the serial port number to enable //选择要开启的串口号
        Stm32_Serial.setBaudrate(115200); //Set the baud rate //设置波特率
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open(); //Open the serial port //开启串口
      }
      catch (serial::IOException& e)
      {
        RCLCPP_ERROR(this->get_logger(),"originbot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
      }
            }

      if(Stm32_Serial.isOpen())
      {
        RCLCPP_INFO(this->get_logger(),"originbot serial port opened"); //Serial port opened successfully //串口开启成功提示
      }

    }

    //析构函数
    ~originbot_driver()
    {
      Stm32_Serial.close();
    }

    void Control()
    {
      _Last_Time = this->now();
      while(rclcpp::ok())
      {
        Sampling_Time = (_Now.seconds() - _Last_Time.seconds());
        _Now = this->now();

        if (Get_Sensor_Data_New())
        {
          RCLCPP_INFO(this->get_logger(),"successful");
          if(arry[3]==0xFF)
            Speed[0]=1.0*arry[4]/100.0;
          else
            Speed[0]=arry[4]*-1.0/100.0;

          if(arry[5]==0xFF)
            Speed[1]=1.0*arry[6]/100.0;
          else
            Speed[1]=arry[6]*-1.0/100.0;
//        Speed[0]+=0.1f;
//        Speed[1]+=0.1f;
         // if(Speed[0]>0.5) Speed[0]=0.5;
         // if(Speed[1]>0.5) Speed[1]=0.5;
         // if(Speed[0]<-0.5) Speed[0]=-0.5;
         // if(Speed[1]<0.5) Speed[1]=-0.5;

        if(abs(Speed[0]+Speed[1])<=0.03 && Speed[0]*Speed[1]<0){
                if(Speed[0]>0)
                Speed[1]=-1.0*Speed[0];
                else Speed[0]=-1.0*Speed[1];
        }
//      Speed[0]=Speed[0]+0.1;
//      Speed[1]=Speed[1]+0.1;
          x_linear_speed = (Speed[0] + Speed[1])/2;

          z_angle_speed  =(Speed[1] - Speed[0])/wheel_track;
         if(z_angle_speed<-1) z_angle_speed=-1;
        if(z_angle_speed>1) z_angle_speed=1;
          de_th=z_angle_speed*Sampling_Time;
         RCLCPP_INFO(this->get_logger(), "sampling_Time=%f 左轮速度=%f 右轮速度=%f 线速度=%f 角速度=%f",Sampling_Time,Speed[0],Speed[1],x_linear_speed,z_angle_speed);
//      if(abs(Speed[0]-Speed[1])<0.2) Speed[0]=Speed[1];
         // if((z_angle_speed<0.1)&&(z_angle_speed>-0.1)) vth=0;
         // else  vth=z_angle_speed;

          de_x=x_linear_speed*cos(th)*Sampling_Time;
          de_y=x_linear_speed*sin(th)*Sampling_Time;
        //  de_th=z_angle_speed*Sampling_Time;

          x=x+de_x;
          y=y+de_y;
          th=th+de_th;
       //  if(th>3.1415926) th-=3.1415926*2;
           RCLCPP_INFO(this->get_logger(), "x=%f y=%f th=%f de_x=%f de_y=%f,de_th=%f",x,y,th,de_x,de_y,de_th);

       //  if(th<-3.1415926) th+=3.1415926*2;   
          odom_callback();

          // linear_acceleration[0]=IMU_Trans(arry[13],arry[12])/1671.84;
          // linear_acceleration[1]=IMU_Trans(arry[15],arry[14])/1671.84;
          // linear_acceleration[2]=IMU_Trans(arry[17],arry[16])/1671.84;
          // angular_velocity[0]=IMU_Trans(arry[24],arry[23])*0.00026644;
          // angular_velocity[1]=IMU_Trans(arry[26],arry[25])*0.00026644;
          // angular_velocity[2]=IMU_Trans(arry[28],arry[27])*0.00026644;
          // euler[0]=IMU_Trans(arry[35],arry[34]);
          // euler[1]=IMU_Trans(arry[37],arry[36]);
          // euler[2]=IMU_Trans(arry[39],arry[38]);
          linear_acceleration[0]=IMU_Trans(arry[12],arry[13])/1671.84;
          linear_acceleration[1]=IMU_Trans(arry[14],arry[15])/1671.84;
          linear_acceleration[2]=IMU_Trans(arry[16],arry[17])/1671.84;
          angular_velocity[0]=IMU_Trans(arry[23],arry[24])*0.00026644;
          angular_velocity[1]=IMU_Trans(arry[25],arry[26])*0.00026644;
          angular_velocity[2]=IMU_Trans(arry[27],arry[28])*0.00026644;
          euler[0]=IMU_Trans(arry[34],arry[35])*0.00026644;
          euler[1]=IMU_Trans(arry[36],arry[37])*0.00026644;
          euler[2]=IMU_Trans(arry[38],arry[39])*0.00026644;
          //RCLCPP_INFO(this->get_logger(), "二进制下imu数据 %f %f %f %f %f %f %f %f %f ",linear_acceleration[0],linear_acceleration[1],linear_acceleration[2],angular_velocity[0],angular_velocity[1],angular_velocity[2],euler[0],euler[1],euler[2]);

          imu_callback();
          _Last_Time = _Now;
        }

        rclcpp::spin_some(shared_from_this());
      }
    }


  private:
    rclcpp::Time _Now, _Last_Time;
    float Sampling_Time;
    uint8_t arry[53]={0};
    float Speed[2] = {0.0,0.0};
    float x_linear_speed=0 ,z_angle_speed=0,de_th=0,de_x=0,de_y=0,vth=0;
    float linear_acceleration[3]={0.0},angular_velocity[3]={0.0},euler[3]={0.0};

    float IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
    {
      short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;
      transition_16 |=  Data_Low;
      return transition_16;
    }


    void odom_callback()
    {
      // tf2::Quaternion odom_quat;
      // odom_quat.setRPY(0, 0, th);
      // geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(th);
      auto odom_msg = nav_msgs::msg::Odometry();
      //里程数据计算
      odom_msg.header.frame_id = "odom";
      odom_msg.header.stamp = this->get_clock()->now();
      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = 0;
      // odom_msg.pose.pose.orientation = odom_quat;
      tf2::Quaternion q;
      q.setRPY(0, 0, th);
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose.orientation.x = q[0];
      odom_msg.pose.pose.orientation.y = q[1];
      odom_msg.pose.pose.orientation.z = q[2];
      odom_msg.pose.pose.orientation.w = q[3];

      const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,

      0, 1e-3,    0,   0,   0,    0,

      0,    0,  1e6,   0,   0,    0,
      0,    0,    0, 1e6,   0,    0,

      0,    0,    0,   0, 1e6,    0,

      0,    0,    0,   0,   0, 1e-9 };

      odom_msg.twist.twist.linear.x = x_linear_speed;
      odom_msg.twist.twist.linear.y = 0.00;
      odom_msg.twist.twist.linear.z = 0.00;

      odom_msg.twist.twist.angular.x = 0.00;
      odom_msg.twist.twist.angular.y = 0.00;
      odom_msg.twist.twist.angular.z = z_angle_speed;

      const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
      0, 1e-3, 1e-9,   0,   0,    0,

      0,    0,  1e6,   0,   0,    0,

      0,    0,    0, 1e6,   0,    0,

      0,    0,    0,   0, 1e6,    0,

      0,    0,    0,   0,   0, 1e-9} ;
     // RCLCPP_INFO(this->get_logger(), "%f %f %f",x,y,th);
      if(x_linear_speed==0&&z_angle_speed==0)
              memcpy(&odom_msg.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom_msg.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
      else
               memcpy(&odom_msg.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom_msg.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
      odom_publisher_->publish(odom_msg);

      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = 0.0;

      t.transform.rotation.x = q[0];
      t.transform.rotation.y = q[1];
      t.transform.rotation.z = q[2];
      t.transform.rotation.w = q[3];

      tf_broadcaster_->sendTransform(t);
    }

    void imu_callback()
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

      imu_msg.linear_acceleration_covariance = {0.04,0.00,0.00,0.00,0.04,0.00,0.00,0.00,0.04};

      imu_msg.angular_velocity_covariance = {0.02,0.00,0.00,0.00,0.02,0.00,0.00,0.00,0.02};

      imu_msg.orientation_covariance = {0.0025,0.0000,0.0000,0.0000,0.0025,0.0000,0.0000,0.0000,0.0025};

      //RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");
      imu_publisher_->publish(imu_msg);
    }
      int  leftSpeedValue=0,rightSpeedValue=0;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

      float x_linear = msg->linear.x;
      float z_angular = msg->angular.z;

      float leftSpeed=0.0;
      float rightSpeed=0.0;

      //差分轮运动学模型求解，三种情况
      if(x_linear == 0)
      {
        rightSpeed = z_angular * wheel_track /2.0;
        leftSpeed = (-1) * rightSpeed;
      }else if(z_angular == 0)
      {
        leftSpeed = x_linear;
        rightSpeed = x_linear;
      }else{
        leftSpeed = x_linear - z_angular * wheel_track /2.0;
        rightSpeed = x_linear + z_angular * wheel_track /2.0;
      }

      RCLCPP_INFO(this->get_logger(), "leftSpeed = '%f'   rightSpeed = '%f'", leftSpeed*100,rightSpeed*100);

      int leftFlag=0xff,rightFlag=0xff;
  //    int leftSpeedValue=0,rightSpeedValue=0;

      if(leftSpeed < 0)
          leftFlag = 0x00;
      else
          leftFlag = 0xff;
      leftSpeedValue = int(abs(leftSpeed)*100);    //速度值从m/s变为cm/s


      if(rightSpeed < 0)
          rightFlag = 0x00;
      else
          rightFlag = 0xff;
      rightSpeedValue = int(abs(rightSpeed)*100);

      int dataCheck = (leftFlag + leftSpeedValue + rightFlag + rightSpeedValue) & 0xff;

      //一组数据
      uint8_t data[9] = {0x55,0x01,0x04,leftFlag,leftSpeedValue,rightFlag,rightSpeedValue,dataCheck,0xbb};
      try
      {
        Stm32_Serial.write(data,sizeof (data));  //Send data to the serial port //向串口发数据 
      }

      catch (serial::IOException& e)
      {
          RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
      }
          //发送时int转bytes
    }

    int find(uint8_t ar[], int n, int element)//查找元素并返回位置下标,find(数组，长度，元素)
    {
      int i = 0;
      int index=-1;//原始下标，没找到元素返回-1
      for (i = 0; i <n; i++)
      {
        if (element ==ar[i]&&ar[i+1]==0x02)
        {
          index=i;//记录元素下标
          return index;//返回下标
        }
      }
    }

    bool Get_Sensor_Data_New()
    {
      short transition_16=0; //Intermediate variable //中间变量
      uint8_t i=0,check=0, error=1,Receive_Data_Pr[100]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存>下位机数据

      Stm32_Serial.read(Receive_Data_Pr,sizeof(Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通
过串口读取下位机发送过来的数据
      int index=find(Receive_Data_Pr,100,0x55);

      for(int i=index, j=0;j<53;i++,j++)
      {
        arry[j]=Receive_Data_Pr[i];
      }

      if(arry[0]==0x55 && arry[52]==0xBB)
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

  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<originbot_driver>());
  auto rplidar_scan_publisher = std::make_shared<originbot_driver>();
  rplidar_scan_publisher->Control();
  rclcpp::shutdown();

  return 0;
}

