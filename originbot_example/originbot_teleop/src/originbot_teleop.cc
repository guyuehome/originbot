#include "originbot_teleop.hpp"

//init启动
OriginbotTeleop::OriginbotTeleop(std::string nodeName) : Node(nodeName) {
    RCLCPP_INFO(this->get_logger(),"Starting up OriginBot telop keyboard controller");

    _speed_linear_x = MAX_SPEED_LINEARE_X;
    _speed_angular_z = MAX_SPEED_ANGULAR_Z;
    
    tcgetattr(kfd,&initial_settings);
    new_settings = initial_settings;
    //使用标准输入模式 | 显示输入字符
    new_settings.c_lflag &= ~(ICANON | ECHO);
    //VEOL: 附加的end of life字符
    new_settings.c_cc[VEOL] = 1;
    //VEOF: end of life字符
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);

    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    showMenu();
    originbot_teleopkeyboardLoop();
}

void OriginbotTeleop::stopRobot() {
    cmdvel_.linear.x = 0.0;
    cmdvel_.angular.z = 0.0;
    pub_cmd->publish(cmdvel_);
}

void OriginbotTeleop::showMenu() {
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "|    Q   W   E   |   left-forward    forward     right-forward  |" << std::endl;
    std::cout << "|    A   S   D   |   left-turn       backward    right-turn     |" << std::endl;
    std::cout << "|    Z       C   |   left_backward               right-backward |" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << std::endl;
    std::cout << "press ctrl+c to quit" << std::endl;
}

void OriginbotTeleop::originbot_teleopkeyboardLoop() {
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    char key;
    int speed = 0;
    int turn = 0;				
    bool dirty = false;
    while (rclcpp::ok()) {
          boost::this_thread::interruption_point();
            int sparkbasebit = 0;
            int num;
            if ((num = poll(&ufd, 1, 500)) < 0) {
                tcsetattr(kfd, TCSANOW, &initial_settings);
                perror("poll():");
                return;
            } else if (num > 0) {
                new_settings.c_cc[VMIN] = 0;
                tcsetattr(0, TCSANOW, &new_settings);
                read(0,&key,1);
                new_settings.c_cc[VMIN] = 1;
                tcsetattr(0, TCSANOW, &new_settings);
            } else {
                if (dirty == true) {
                    stopRobot();
                    dirty = false;
                }
                continue;
            }
            switch (key) {
                case KEYCODE_W:	
                    speed = 1;
                    turn = 0;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout << "Move forward" << std::endl;
                    break;
                case KEYCODE_S:;
                    speed = -1;
                    turn = 0;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout << "Move backward" << std::endl;
                    break;
                case KEYCODE_A:
                    speed = 0;
                    turn = 1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout << "Left rotate" << std::endl;
                    break;
                case KEYCODE_D:
                    speed = 0;
                    turn = -1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout << "Right rotate" << std::endl; 
                    break;
                case KEYCODE_C:
                    speed = -1;
                    turn = 1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout <<"Left backward turn" << std::endl;
                    break;
                case KEYCODE_Z:
                    speed = -1;
                    turn = -1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout <<"Right backward turn"<< std::endl ;
                    break;
                case KEYCODE_Q:
                    speed = 1;
                    turn = 1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout <<"Right forward turn"<< std::endl ;
                    break;
                case KEYCODE_E:
                    speed = 1;
                    turn = -1;
                    dirty = true;
                    sparkbasebit = 1;
                    std::cout <<"Left forward turn"<< std::endl ;
                    break;
                default:
                    speed = 0;
                    turn = 0;
                    dirty = false;
                    sparkbasebit = 1;
            }
            if (sparkbasebit == 1) {
                cmdvel_.linear.x = speed * _speed_linear_x;	
                cmdvel_.angular.z = turn * _speed_angular_z;	
                pub_cmd->publish(cmdvel_);
            }
    }
}
int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<OriginbotTeleop>("originbot_teleop");
    boost::thread thread = boost::thread(boost::bind(&OriginbotTeleop::originbot_teleopkeyboardLoop, node));
    rclcpp::spin(node);
    thread.interrupt();
    rclcpp::shutdown();
    return 0;
}

