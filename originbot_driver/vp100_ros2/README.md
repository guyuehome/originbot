# NVILIDAR ROS2 Driver

nvilidar_ros2_driver is a new ros package, which is designed to gradually become the standard driver package for nvilidar devices in the ros2 environment.

## How to install ROS2

[install ros2](https://index.ros.org/doc/ros2/Installation)

## How to Create a ROS2 workspace
[Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

## How to build NVILiDAR ROS Package

### 1.Get the ros2 code
    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/nvilidar/vp100_ros2.git  
             or
             git clone https://gitee.com/nvilidar/vp100_ros2.git
    	(2). git chectout master

    2) download the code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin

### 2.Copy the ros2 code
    1) copy the file to the "src" dir
### 3.Build the ros2 code
      --$ cd vp100_ros2_ws
      --$ colcon build --symlink-install

   Note: How to [install colcon](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)
### 4.Ros2 Package environment setup
      --$ source ./install/setup.bash

   Note: Add permanent workspace environment variables.
   It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:

      --$ $echo "source ~/vp100_ros2_ws/install/setup.bash" >> ~/.bashrc
      --$ $source ~/.bashrc

### 5.Confirmation Ros2
   To confirm that your package path has been set, printenv the `grep -i ROS` variable.

    --$ printenv | grep -i ROS


### 6.Serialport configuration
   1) if you want to use the unchanging device name,Create the name "/dev/nvilidar" to rename serialport

            --$ $chmod 0777 vp100_ros2/startup/*
            --$ $sudo sh vp100_ros2/startup/initenv.sh
 
   Note: After completing the previous operation, replug the lidar again.

   2) if you use the lidar device name,you must give the permissions to user.

            --$ whoami
            get the user name.link ubuntu.
            --$ sudo usermod -a -G dialout ubuntu
            ubuntu is the user name.
            --$ sudo reboot 
            

## ROS Parameter Configuration
### 1. Lidar Support
   
   current ros support 2 types of Lidar,VP100A and VP100L.

   the VP100A Lidar's baudrate is 115200bps,the VP100L Lidar's baudrate is 230400bps.

## Interface function definition
### 1. bool LidarProcess::LidarInitialialize()
    Initialize the lidar, including opening the serial interface and synchronizing the radar with the SDK parameter information
	if initial fail return false.
### 2. bool LidarProcess::LidarTurnOn()
	Turn on lidar scanning so that it can output point cloud data.
### 3. bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	Real-time radar data output interface.
	The LidarScan variables are described as follows:
 
|  value   | Element | define  |
|  :----:  | :----:  | :----:  |
|  stamp   | none    |  lidar stamps, unit ns|
|  config  | min_angle   | lidar angle min value, 0~2*PI,Unit Radian|
|          | max_angle   | lidar angle max value, 0~2*PI,Unit Radian|
|          | angle_increment   | angular interval between 2 points, 0~2PI|
|          | scan_time   | time interval of 2 turns of data|
|          | min_range   | Distance measurement min, unit m|
|          | max_range   | Distance measurement max, unit m|
|  points  | angle       | lidar angle,0~2PI|
|  | range       | lidar distance, unit m|
|  | intensity       | lidar intensity,it is aviliable when sensitive is true|
### 4. bool LidarProcess::LidarTurnOff()
	lidar turn off the scanning data 
### 5. void LidarProcess::LidarCloseHandle()
	lidar close serialport/socket 

## Run vp100_ros2

### Run vp100_ros2 using launch file

The command format is : 

     --$ ros2 launch vp100_ros2 [launch file].py

#### 1. Connect LiDAR uint(s).
   ```
   ros2 launch vp100_ros2 vp100_launch.py 
   ```
#### 2. RVIZ 
   ```
   ros2 launch vp100_ros2 vp100_launch_view.py 
   ```
#### 3. echo scan topic
   ```
   ros2 run vp100_ros2 vp100_ros2_client
   ```
   or
   ```
   ros2 topic echo /scan
   ```
	

## nvilidar_ros2 Parameter

### 1. Configure File [paramters](params/nvilidar.yaml)
```
vp100_ros2_node:
  ros__parameters:
    serialport_name: "/dev/nvilidar"
    serialport_baud: 115200
    frame_id: "laser_frame"
    resolution_fixed: false
    auto_reconnect: true
    reversion: false
    inverted: false
    angle_min: -180.0
    angle_max: 180.0
    range_min: 0.001
    range_max: 64.0
    aim_speed: 6.0
    sampling_rate: 3
    angle_offset_change_flag: false
    angle_offset: 0.0
    ignore_array_string: ""
    log_enable_flag: true

```
### Parameter define
|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| ip_addr  | if use udp socket,the lidar's ip addr,default:192.168.1.200 |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| auto_reconnect  | lidar auto connect,if it is disconnet in case |
| reversion  | lidar's point revert|
| inverted  | lidar's point invert|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:64.0 meters|
| range_min  | lidar's min measure distance,default:0.0 meters|
| aim_speed  | lidar's run speed,default:10.0 Hz|
| sampling_rate  | lidar's sampling rate,default:10.0 K points in 1 second|
| angle_offset_change_flag  | angle offset enable to set,default:false|
| angle_offset  | angle offset,default:0.0|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°~60° and 90°~120° points in the view|
| log_enable_flag |  if you want to write log to file,you must write it 'true' |







