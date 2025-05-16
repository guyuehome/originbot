# OriginBot Open Source Kit
![originbot](images/originbot.jpg)

OriginBot is an open-source robot kit and a community-driven project aimed at enabling every participant to enjoy the fun of robot development.

## Project Links

### Official Website

[https://www.originbot.org/en/](https://www.originbot.org/en/)

### Source Code Repositories

| Repository                                                     | Description                 |
| -------------------------------------------------------------- | --------------------------- |
[originbot ](https://github.com/guyuehome/originbot)             | OriginBot robot function package repository |
[originbot_desktop](https://github.com/guyuehome/originbot_desktop) | OriginBot desktop function package repository |
[originbot_controller](https://github.com/guyuehome/originbot_controller) | OriginBot controller source code repository |

### Community Forum

[https://www.guyuehome.com/interlocution](https://www.guyuehome.com/interlocution)


## Software Architecture

- originbot_base: Robot chassis driver
- originbot_driver: Robot device drivers
    - serial_ros2: Serial driver package
    - ydlidar_ros2_driver: Lidar driver package
    - vp100_ros2: Lidar driver package
    - qpOASES: qpOASES solver package
- originbot_msgs: OriginBot custom communication interface
- originbot_bringup: Scripts and files for robot startup
- originbot_demo: Programming examples for basic robot functions
- originbot_linefollower: Robot vision line-following function package
- originbot_navigation: Scripts and configuration files for robot mapping and navigation
- originbot_deeplearning: Robot deep learning functions
    - body_tracking: Robot body tracking function package
    - gesture_control: Robot gesture control function package
    - line_follower_perception: Robot AI vision line-following function package
    - parking_search: Robot parking space search function package
    - play_football: Robot football-playing function package
- originbot_example: Common robot application functions
    - originbot_autonomous: Robot trajectory tracking
    - originbot_qr_code: Robot QR code recognition
    - originbot_teleop: Robot key control
    - originbot_vlpr: Robot license plate recognition
    - send_goal: Robot waypoint navigation
    - take_pictures_node: Robot image capture


## Contributing

We sincerely invite developers to participate in the OriginBot project. There are many ways and forms to contribute:

### **Provide Feedback**

- If you encounter any issues or have suggestions while using the OriginBot kit, feel free to ask questions and discuss them in the [Guyuehome Community Forum](https://guyuehome.com/Bubble/circleDetail/id/95);

- If you find any bugs while using the OriginBot software, please submit an issue in the [repository](https://github.com/guyuehome/originbot);

### **Contribute Code**

- If you have optimizations, additions, or other modifications to the native code while using the OriginBot kit, feel free to submit a Pull Request in the [repository](https://github.com/guyuehome/originbot);

### **Spread Open Source**

- If you are interested in OriginBot, feel free to star the project's source code repository or share it with developers who might need it;

- If you develop more interesting functions or robots based on the OriginBot open-source project, feel free to share them in the [community forum](https://guyuehome.com/Bubble/circleDetail/id/95). Outstanding projects will also be promoted in the community.
