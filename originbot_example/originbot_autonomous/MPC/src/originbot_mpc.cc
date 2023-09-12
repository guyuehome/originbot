/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "originbot_mpc.h"

//构造函数
MpcController::MpcController(string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(),"Starting up MPC controller");

    //初始化各类参数
    initParm();
    //加载路径点信息
    loadPath();
    //设置订阅者和发布者
    pub_path = this->create_publisher<nav_msgs::msg::Path>("base_path",1);
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom",1,bind(&MpcController::odom_callback,this,_1));
}

//初始化各类参数
void MpcController::initParm()
{
    this->declare_parameter<std::float_t>("car_length",wheel_base);                             //车轴长度
    this->declare_parameter<std::float_t>("front_left_wheel_mass",mass_front_left_wheel);       //左前轮质量
    this->declare_parameter<std::float_t>("front_right_wheel_mass",mass_front_right_wheel);     //右前轮质量
    this->declare_parameter<std::float_t>("rear_left_wheel_mass",mass_rear_left_wheel);         //左后轮质量
    this->declare_parameter<std::float_t>("rear_right_wheel_mass",mass_rear_right_wheel);       //右后轮质量
    this->declare_parameter<std::float_t>("body_mass",mass_body);                               //车身质量
    this->declare_parameter<std::float_t>("front_wheel_stiffiness",cf_);                        //前轮侧偏刚度
    this->declare_parameter<std::float_t>("rear_wheel_stiffiness",cr_);                         //后轮侧偏刚度
    this->declare_parameter<std::float_t>("control_period",ts_);                                //控制时间
    this->declare_parameter<std::float_t>("max_steer_angle",max_steer_angle);                   //最大转向角
    this->declare_parameter<std::float_t>("reference_velocity",v_ref);                          //参考速度
    this->declare_parameter<std::float_t>("horizon",horizon_);                                  //预测步长
    this->declare_parameter<std::float_t>("number_of_iteration",numOfIter);                     //mpc迭代步长

    //从yaml中获取车辆信息
    this->get_parameter("car_length",wheel_base); 
    this->get_parameter("front_left_wheel_mass",mass_front_left_wheel);                             //左前轮质量
    this->get_parameter("front_right_wheel_mass",mass_front_right_wheel);                           //右前轮质量
    this->get_parameter("rear_left_wheel_mass",mass_rear_left_wheel);                               //左后轮质量
    this->get_parameter("rear_right_wheel_mass",mass_rear_right_wheel);                             //右后轮质量
    this->get_parameter("body_mass",mass_body);                                                     //车身质量
    this->get_parameter("front_wheel_stiffiness",cf_);                                              //前轮侧偏刚度
    this->get_parameter("rear_wheel_stiffiness",cr_);                                               //后轮侧偏刚度
    this->get_parameter("control_period",ts_);                                                      //控制时间
    this->get_parameter("max_steer_angle",max_steer_angle);                                         //最大转向角
    this->get_parameter("reference_velocity",v_ref);                                                //参考速度
    this->get_parameter("horizon",horizon_);                                                        //预测步长
    this->get_parameter("number_of_iteration",numOfIter);                                           //mpc迭代步长

    //矩阵初始化
    r_ = Matrix::Identity(basic_control_size,basic_control_size);                                   //控制权重矩阵
    q_ = Matrix::Identity(basic_state_size+basic_control_size,basic_state_size+basic_control_size); //状态权重矩阵
    q_ *= 20;
    state_mat_ = Matrix::Zero(basic_state_size,1);                                                  //误差状态矩阵
    control_mat_ = Matrix::Zero(basic_control_size,1);                                              //控制矩阵
    a_mat_ = Matrix::Zero(basic_state_size,basic_state_size);                                       //状态空间矩阵
    a_mat_d_ = Matrix::Zero(basic_state_size,basic_state_size);                                     //离散状态空间矩阵
    b_mat_ = Matrix::Zero(basic_state_size,1);                                                      //控制空间矩阵
    b_mat_d_ = Matrix::Zero(basic_state_size,1);                                                    //离散控制空间矩阵
    c_mat_ = Matrix::Zero(basic_state_size,1);                                                      //扰动空间矩阵
    c_mat_d_ = Matrix::Zero(basic_state_size,1);                                                    //离散扰动空间矩阵

    double mass_front = mass_front_left_wheel + mass_front_right_wheel + mass_body / 2.0;           //前车质量
    double mass_rear = mass_rear_left_wheel + mass_rear_right_wheel + mass_body / 2.0;              //后车质量
    mass_ = mass_front + mass_rear;
    lf_ = wheel_base * (1 - mass_front / mass_);                                                    //前轮轴距
    lr_ = wheel_base * (1 - mass_rear / mass_);                                                     //后轮轴距
    izz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
    max_steer_angle *= M_PI/180.0;
    string pkg_name = "originbot_autonomous";
    string waypoints_name = "path.csv";
    string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;
    RCLCPP_INFO(this->get_logger(),"waypoints file is loaded at directory: %s",waypoints_path.c_str());
}

void MpcController::loadPath()
{
    WaypointLoader wploader(waypoints_path);
    bool isFileLoaded = wploader.loadWayPoints();
    if (!isFileLoaded) {
        RCLCPP_ERROR(this->get_logger(),"File is not exist or file is empty!");
        exit(1);
    }
    vector<vector<double>> wp = wploader.getWayPoints();
    for(vector<vector<double>>::const_iterator it=wp.begin();it!=wp.end();it++) {
        xr.push_back((*it)[0]);
        yr.push_back((*it)[1]);
        yawr.push_back((*it)[2]);
        kappar.push_back((*it)[3]);
    }    
}

int MpcController::findCloestIndex(const double x,const double y)
{
    vector<double> dist; 
    for (int i = 0;i<(int)xr.size();i++) {
        dist.push_back(pow(xr[i]-x,2)+pow(yr[i]-y,2));
    }
    auto smallest = min_element(dist.begin(),dist.end());
    int index = distance(dist.begin(),smallest);

    return index;
}

//误差计算模块
void MpcController::computeStateError(const double x,
                                        const double y, 
                                        const double v, 
                                        const double heading, 
                                        const double heading_rate, 
                                        const int index)
{
    double dx = x - xr[index];
    double dy = y - yr[index];

    //横向误差
    double ed = dy * cos(yawr[index]) - dx * sin(yawr[index]);
    //航向误差
    double e_psi = normalizeAngle(heading - yawr[index]);
    //横向误差率
    double ed_dot = v * sin(e_psi);
    //纵向速度
    double s_dot = v * cos(e_psi) / (1 - kappar[index] * ed);
    //航向误差率
    double e_psi_dot =  heading_rate - kappar[index] * s_dot;

    //把误差放入矩阵中
    state_mat_(0,0) = ed;
    state_mat_(1,0) = ed_dot;
    state_mat_(2,0) = e_psi;
    state_mat_(3,0) = e_psi_dot;
}

//计算状态空间矩阵
void MpcController::computeStateMatrix(const double linear_velocity)
{
    Matrix i_mat = Matrix::Identity(basic_state_size,basic_state_size);

    //状态矩阵
        // [0,1,0,0;
        // 0,(-(c_f+c_r)/(m*v),(c_f+c_r)/m,-(c_f*l_f-c_r*l_r)/(m*v);
        // 0,0,0,1;
        // 0,(-(c_f*l_f-c_r*l_r))/(I_z*v),(c_f*l_f-c_r*l_r)/I_z,(-(c_f*l_f^2+c_r*l_r^2))/(I_z*v)];
    a_mat_(0,1) = 1.0;
    a_mat_(1,1) = -(cf_ + cr_) / (mass_ * linear_velocity);
    a_mat_(1,2) = (cf_ + cr_) / mass_;
    a_mat_(1,3) = -(cf_ * lf_ - cr_ * lr_) / (mass_ * linear_velocity);
    a_mat_(2,3) = 1.0;
    a_mat_(3,1) = -(cf_ * lf_ - cr_ * lr_) / (izz_ * linear_velocity);
    a_mat_(3,2) = (cf_ * lf_ - cr_ * lr_) / izz_;
    a_mat_(3,3) = -(cf_ * lf_ * lf_ + cr_ * lr_ * lr_) / (izz_ * linear_velocity);
    a_mat_d_ = (i_mat + 0.5 * ts_ * a_mat_) * (i_mat - 0.5 * ts_ * a_mat_).inverse();

    //控制矩阵
        // [0;
        // cf/m;
        // 0;
        // (cf*lf)/Iz]
    b_mat_(1,0) = cf_ / mass_;
    b_mat_(3,0) = (cf_ * lf_) / izz_;
    b_mat_d_ = b_mat_ * ts_;

    //扰动矩阵
        // [0;
        // (cf*lf-cr*lr)/(m*v)-v;
        // 0;
        // (lf*lf*cf+lr*lr*cr)/(izz*v)]
    c_mat_(1,0) = (cf_ * lf_ - cr_ * lr_) / (mass_ * linear_velocity) - linear_velocity;
    c_mat_(3,0) = (cf_* lf_ * lf_ + cr_ * lr_ * lr_) / izz_ / linear_velocity;
    c_mat_d_ = c_mat_ * heading_error_rate * ts_;
}

//mpc求解器
double MpcController::mpcSolver()
{
    //设置某一时时刻下的转角限制
    Matrix upper_bound = Matrix::Zero(basic_control_size,1);
    upper_bound << max_steer_angle;
    Matrix lower_bound = Matrix::Zero(basic_control_size,1);
    lower_bound << -max_steer_angle;
    //设置转角变化限制
    Matrix upper_bound_delta = Matrix::Zero(basic_control_size,1);
    upper_bound_delta << 0.64;
    Matrix lower_bound_delta = Matrix::Zero(basic_control_size,1);
    lower_bound_delta << -0.64;

    // Y(t) = PSI * kesi + PHI * delta_U
    //构建预测空间
    //预测误差空间矩阵
    Matrix kesi_mat = Matrix::Zero(basic_state_size+basic_control_size,1);
    kesi_mat.block(0,0,basic_state_size,1) = state_mat_;
    kesi_mat.block(4,0,basic_control_size,1) = control_mat_;

    //预测状态空间矩阵
        // A = [a b]
        //     [O i]
    Matrix A_matrix_ =  Matrix::Zero(basic_state_size+basic_control_size,basic_state_size+basic_control_size);
    A_matrix_.block(0,0,basic_state_size,basic_state_size) = a_mat_d_;
    A_matrix_.block(0,4,b_mat_d_.rows(),b_mat_d_.cols()) = b_mat_d_;
    A_matrix_.block(4,4,1,1) = Matrix::Identity(1,1);

    //预测控制空间矩阵
        // B = [b]
        //     [I]
    Matrix B_matrix_ = Matrix::Identity(b_mat_d_.rows()+basic_control_size,1);
    B_matrix_.block(0,0,b_mat_d_.rows(),1) = b_mat_d_;

    //预测扰动空间矩阵
        // C = [c]
        //     [0]
    Matrix C_matrix_ = Matrix::Zero(c_mat_d_.rows()+basic_control_size,1);
    C_matrix_.block(0,0,c_mat_d_.rows(),1) = c_mat_d_;

    //预测模型空间矩阵
        // PSI = [A
        //         A^2
        //         A^3
        //         ...
        //         A^(Np)]
    vector<Matrix> A_power_matrix(horizon_);
    A_power_matrix[0] = A_matrix_;
    for (int i = 1; i < horizon_; ++i)
        A_power_matrix[i] = A_matrix_ * A_power_matrix[i-1];
    // 150 x 5
    Matrix PSI_matrix = Matrix::Zero(A_matrix_.rows() * horizon_,A_matrix_.cols());
    for (int j = 0; j < horizon_; j++)
        PSI_matrix.block(j * A_matrix_.rows(),0,A_matrix_.rows(),A_matrix_.cols()) = A_power_matrix[j];

    //预测控制空间矩阵
        // PHI = [B 0 0 0 ... 0
        //         AB B 0 0 ... 0
        //         A^2*B AB B 0 ... 0]
        //         .  . . . ... .
        //         .  . . . ... .
        //         A^(Np)B ... ... B]
    Matrix PHI_matrix = Matrix::Zero(B_matrix_.rows() * horizon_ , B_matrix_.cols() * horizon_);
    PHI_matrix.block(0,0,B_matrix_.rows(),B_matrix_.cols()) = B_matrix_;
    for (int r = 1; r<horizon_; r++) {
        for (int c = 0; c<r; ++c)
             PHI_matrix.block(r * B_matrix_.rows(), c * B_matrix_.cols(), B_matrix_.rows(), B_matrix_.cols()) = A_power_matrix[r - c - 1] * B_matrix_; 
       
        PHI_matrix.block(r * B_matrix_.rows(), r * B_matrix_.cols(), B_matrix_.rows(), B_matrix_.cols()) = B_matrix_;
    }
    //预测扰动空间矩阵
        // GAMMA = [C
        //         AC+C
        //         A^2C+AC+C
        //         ...        
        //         SUM(A^iC)
        //         ]
    Matrix GAMMA_matrix = Matrix::Zero(C_matrix_.rows() * horizon_, 1);
    GAMMA_matrix.block(0,0,C_matrix_.rows(),1) = C_matrix_;
    for (int r = 1; r < horizon_; r++) {
        GAMMA_matrix.block(r*C_matrix_.rows(),0,C_matrix_.rows(),1) = A_power_matrix[r-1] * C_matrix_ 
                                                                    + GAMMA_matrix.block((r-1)*C_matrix_.rows(),0,C_matrix_.rows(),1);
    }

    //mpc预测问题转换成二次规划问题
        // min J = 1/2 * delta_U.transpose() * H * delta_U + g.transpose() * delta_U
        // H = PHI.transpose() * Qq * PHI + Rr
        // G = PHI.trranspose() * Qq * E
        // E = PSI * kesi
    //设置E矩阵
    Matrix E_matrix = PSI_matrix * kesi_mat;            // 150 x 1
    //设置Qq矩阵
    Matrix Qq_matrix = Matrix::Identity(PHI_matrix.rows(),PHI_matrix.rows());       // 150 x 150
    //设置Rr矩阵
    Matrix Rr_matrix = Matrix::Identity(PHI_matrix.cols(),PHI_matrix.cols());       // 30 x 30
    for (int i = 0;i < horizon_; i++) {
        Qq_matrix.block(i*q_.rows(),i*q_.rows(),q_.rows(),q_.rows()) = q_;
        Rr_matrix.block(i*r_.rows(),i*r_.rows(),r_.cols(),r_.cols()) = r_;
    }
    //设置H矩阵
    Matrix H_matrix = PHI_matrix.transpose() * Qq_matrix * PHI_matrix + Rr_matrix;  // 30 x 30
    //设置G矩阵
    Matrix G_matrix = PHI_matrix.transpose() * Qq_matrix * (E_matrix+GAMMA_matrix);                // 30 x 1

    //设置二次规划上下限
        // Umin - Ut <= Ai * delta_U <= Umax - Ut      -- 约束
        // delta_U_min <= delta_U <= delta_U_max       -- 界限
    
    //delta_U_min 和 delta_U_max 上下界
    Matrix ll_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);      // 30 x 1
    Matrix uu_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);      // 30 x 1
    //不等式上下界
    Matrix Ut_matrix = Matrix::Zero(horizon_* control_mat_.rows(),1);       // 30 x 1
    Matrix Umax_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);    // 30 x 1
    Matrix Umin_matrix = Matrix::Zero(horizon_ * control_mat_.rows(),1);    // 30 x 1
    for (int j = 0 ; j < horizon_ ; ++j) {
        ll_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = lower_bound_delta;
        uu_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = upper_bound_delta;
        Ut_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = control_mat_;
        Umax_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = upper_bound;
        Umin_matrix.block(j*control_mat_.rows(),0,control_mat_.rows(),control_mat_.cols()) = lower_bound;
    }

    Matrix inequality_lower_boundary_matrix = Matrix::Zero(Umax_matrix.rows(),Umax_matrix.cols());      // 30 x 1
    inequality_lower_boundary_matrix = Umin_matrix - Ut_matrix;
    Matrix inequality_upper_boundary_matrix = Matrix::Zero(Umin_matrix.rows(),Umin_matrix.cols());      // 30 x 1
    inequality_upper_boundary_matrix = Umax_matrix - Ut_matrix;

    Matrix inequality_constraint_matrix = Matrix::Identity(inequality_lower_boundary_matrix.rows(),inequality_lower_boundary_matrix.rows());    // 30 x 30
    for (int r = 1; r < inequality_lower_boundary_matrix.rows(); r++) {
        for (int c = 0; c < r; c++)
            inequality_constraint_matrix(r,c) = 1;
    }
    //设置qp解算器
    double delta_steer = solveQP(H_matrix, G_matrix, 
                            inequality_constraint_matrix, inequality_lower_boundary_matrix,
                            inequality_upper_boundary_matrix , ll_matrix, uu_matrix, numOfIter);
    
    control_mat_(0,0) = kesi_mat(4,0) + delta_steer;

    return control_mat_(0,0);


}

//qp解算器
double MpcController::solveQP(const Matrix& H_matrix,
                              const Matrix& G_matrix, 
                              const Matrix& inequality_matrix, 
                              const Matrix& inequality_lower_boundary_matrix,
                              const Matrix& inequality_upper_boundary_matrix,
                              const Matrix& lower_bound, 
                              const Matrix& upper_bound, 
                              const int numOfIter)
{
    const qpOASES::int_t num_param_ = H_matrix.rows();
    const qpOASES::int_t num_constraints_ = inequality_matrix.rows();
    //创建qp解算器对象
    qpOASES::QProblem mpc_qp(num_param_,num_constraints_);
    const int numOfHmatrixElements = H_matrix.rows() * H_matrix.cols();

    qpOASES::real_t h_matrix[numOfHmatrixElements];
    
    const int numOfGmatrixElements = G_matrix.rows();

    qpOASES::real_t g_matrix[numOfGmatrixElements];
    
    int index = 0;
    for (int r = 0; r < H_matrix.rows();r++) {
        g_matrix[r] = G_matrix(r,0);
        for (int c = 0; c < H_matrix.cols(); c++)
            h_matrix[index++] = H_matrix(r,c);
    }

    //上下限
    qpOASES::real_t lb[num_param_];
    qpOASES::real_t ub[num_param_];
    for (int i = 0; i < num_param_ ; i++) {
        lb[i] = lower_bound(i,0);
        ub[i] = upper_bound(i,0);
    }

    //约束条件
    qpOASES::real_t inequality_constraint[num_param_ * num_constraints_];
    qpOASES::real_t lbA[num_constraints_];
    qpOASES::real_t ubA[num_constraints_];
    index = 0;
    for (int r = 0; r < inequality_matrix.rows(); r++) {
        //约束条件上下界
        lbA[r] = inequality_lower_boundary_matrix(r,0);
        ubA[r] = inequality_upper_boundary_matrix(r,0);

        for (int c = 0; c < inequality_matrix.cols(); c++)
            inequality_constraint[index++] = inequality_matrix(r,c);
    }
    //qp解算
    qpOASES::int_t iterNum = 50;
    auto ret = mpc_qp.init(h_matrix,g_matrix,inequality_constraint,lb,ub,lbA,ubA,iterNum);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
        if (ret == qpOASES::RET_MAX_NWSR_REACHED)
            RCLCPP_INFO(this->get_logger(),"qpOASES solver failed due to reached max iteration");
        else
            RCLCPP_INFO(this->get_logger(),"qpOASES solver filaed due to some reasons...Please check...");
    }
    //返回结果是delta_U的整个预测空间
    double result[num_param_];
    mpc_qp.getPrimalSolution(result);

    return result[0];
}

//前馈补偿
double MpcController::feedForwardControl(const double v, const double kappa)
{
    double kv = lr_ * mass_/ cf_ / wheel_base - lf_ * mass_ / cr_ / wheel_base;
    double steering_angle_feedforward = wheel_base * kappa + kv * v * v * kappa ;

    return steering_angle_feedforward;
}

//计算最终转向角
double MpcController::computeSteering(const double x,
                                        const double y,
                                        const double v,
                                        const double heading, 
                                        const double heading_rate)
{
    int index = findCloestIndex(x,y);
    //计算误差
    computeStateError(x,y,v,heading,heading_rate,index);
    //计算状态空间和控制空间矩阵
    computeStateMatrix(v);
    //前馈补偿
    double steer_feedforawrd = feedForwardControl(v,kappar[index]);
    //mpc控制
    double steer = mpcSolver();
    return steer+steer_feedforawrd;
}

//归一化角度
double MpcController::normalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
        a += (2.0 * M_PI);
    return a - M_PI;   
}

void MpcController::pubPath()
{
    geometry_msgs::msg::PoseStamped pose;
    nav_msgs::msg::Path base_path;
    base_path.header.stamp = this->get_clock()->now();
    base_path.header.frame_id = "odom";
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "odom";
    for (int i=0;i<(int)xr.size();i++) {   
        pose.pose.position.x = xr[i];
        pose.pose.position.y = yr[i];
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0,0,yawr[i]);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        base_path.poses.push_back(pose);
    }

    pub_path->publish(base_path);
}

void MpcController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs)
{
    pubPath();

    double x = odom_msgs->pose.pose.position.x;
    double y = odom_msgs->pose.pose.position.y;
    tf2::Quaternion q (
        odom_msgs->pose.pose.orientation.x,
        odom_msgs->pose.pose.orientation.y,
        odom_msgs->pose.pose.orientation.z,
        odom_msgs->pose.pose.orientation.w
        );
    double roll,pitch,yaw;
    tf2::Matrix3x3 m_rpy(q);
    m_rpy.getRPY(roll,pitch,yaw);
    double vx = odom_msgs->twist.twist.linear.x;
    double vy = odom_msgs->twist.twist.linear.y;
    double linear_velocity = sqrt(vx * vx + vy * vy);
    double heading_rate = odom_msgs->twist.twist.angular.z;
    double steering = computeSteering(x,y,linear_velocity,yaw,heading_rate);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_ref;
    cmd_vel.angular.z = steering;
    pub_cmd->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(),"Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);
}

//主程序接口
int main(int argc, char *argv[]) 
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);

    // 创建mpccontroller节点，通过spin不断查询订阅话题
    rclcpp::spin(std::make_shared<MpcController>("mpccontroller"));
    
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();

    return 0;
}
