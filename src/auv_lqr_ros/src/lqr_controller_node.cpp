#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>

// ======================================================
// 【新加】随机噪声需要
// ======================================================
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace std::chrono_literals;


// ======================================================
// 限幅函数
// ======================================================
double limit(double value, double max_value, double min_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}


// ======================================================
// 【新加】均匀随机测量噪声
//
// noise_amplitude = 0.03 时：
// 噪声范围为 [-0.03, 0.03]
// ======================================================
double noise(double noise_amplitude)
{
    double randomnum=(double)rand()/RAND_MAX;
    double randomnum_f1_1=randomnum*2-1;
    double noise_value=randomnum_f1_1*noise_amplitude;
    return noise_value;
}


// ======================================================
// LQR控制器类
// ======================================================

class LQR
{
private:
    double mass=20.0;
    double damping=5;
    double lqrDt=0.1;
    Eigen::Matrix2d A;//连续矩阵
    Eigen::Vector2d B;
    Eigen::Matrix2d Ad;//离散时间矩阵
    Eigen::Vector2d Bd;
    Eigen::Matrix2d Q;
    double R=4;
    Eigen::Matrix2d P;
    Eigen::RowVector2d K;

    double previous_output = 0.0;
    // 输出变化率限制
    double output_rate = 3.0;
    // 输出限幅
    double output_max = 5.0;
    double output_min = -5.0;
public:LQR(){
            A << 0.0, 1.0,0.0, -damping / mass;
            B<<0.0,1/mass;
            Q << 4.0, 0.0,0.0, 1.0;
            R = 4.0;
        //离散化先求Ad,Bd
        Ad= Eigen::Matrix2d::Identity()+A*lqrDt;
        Bd=B*lqrDt;
        
        calculateGain();

        // 打印结果
        cout << endl;
        cout << "==============================" << endl;
        cout << "LQR calculation finished" << endl;

        cout << "mass = " << mass << endl;
        cout << "damping = " << damping << endl;
        cout << "dt = " << lqrDt << endl;

        cout << "A =" << endl;
        cout << A << endl;

        cout << "B =" << endl;
        cout << B << endl;
        
cout << "Ad =" << endl;
cout << Ad << endl;

cout << "Bd =" << endl;
cout << Bd << endl;

        cout << "Q =" << endl;
        cout << Q << endl;

        cout << "R = " << R << endl;

        cout << "K = " << K << endl;

        cout << "==============================" << endl;
        cout << endl;
    }
public:void calculateGain(){
P = Q;
const int maxIterations = 10000;
const double tolerance = 1e-10;
bool converged = false;
for (int i=0;i<maxIterations;i++){
    double denominator=R+(Bd.transpose()*P*Bd)(0,0);
    Eigen::Matrix2d nextP=Ad.transpose()*P*Ad-Ad.transpose()*P*Bd*(1/denominator)*Bd.transpose()*P*Ad+Q;
     double error =(nextP - P).norm();
    P = nextP;
    if (error < tolerance)
            {cout<< "DARE converged after "
                 << i + 1
                 << " iterations."
                << endl;
                converged = true;
                break;
            };};
        if (!converged)
        {
            cerr<< "Warning: DARE did not converge."
            << endl;}
    double denominator=R+(Bd.transpose()*P*Bd)(0,0);
    K=(1/denominator)*Bd.transpose()*P*Ad;
};

public:
double output(double target_depth,double current_depth,double velocity,double dt){
    Eigen::Vector2d e;
    e<<current_depth - target_depth,velocity;
    
     double control =-(K * e)(0, 0);
    control =limit(control,output_max,output_min);
        double maxChange =output_rate * dt;
        double change =control - previous_output;
        change =limit(
                change,
                maxChange,
                -maxChange);
       control =previous_output + change;
       previous_output =control;
        return control;
    }
};



// ======================================================
// 只负责控制的LQR节点
//
// 默认目标深度：10 m

//
// 订阅：
// /target_depth
// /auv/current_trueposition
//
// 发布：
// /auv/output
// /auv/error

//
// 不计算AUV位置
// 不包含auvmodel.update()
// ======================================================
class LQRControllerOnlyNode : public rclcpp::Node
{
private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 目标深度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_depth_sub_;

    // Gazebo当前深度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        current_depth_sub_;
    // Gazebo速度订阅器
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    // 控制输出发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        output_pub_;

    // 误差发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        error_pub_;

;

    // 默认目标深度10 m
    double target_depth = 15.0;

    // Gazebo反馈的当前深度
    double current_depth = 0.0;
    double velocity=0.0;


    // ======================================================
    // 【新加】测量噪声与低通滤波
    // ======================================================

    // 加噪后的深度
    double measured_depth = 0.0;

    // 滤波后的深度
    double filtered_depth = 0.0;

    // 加噪后的速度
    double measured_velocity = 0.0;

    // 滤波后的速度
    double filtered_velocity = 0.0;

    // 深度噪声幅值
    // 与之前PID一致：±0.03 m
    double depth_noise_amplitude = 0.03;

    // 速度噪声幅值
    double velocity_noise_amplitude = 0.01;

    // 一阶低通滤波系数
    double filter_coefficient = 0.9;

    // 判断滤波器是否第一次运行
    bool filter_initialized = false;


    // 控制周期0.05
    double dt = 0.1;

    // 是否收到Gazebo深度
    bool received_current_depth = false;
    bool received_velocity=false;
    LQR controller;


    // CSV记录时间步
    std::size_t step = 0;

    // CSV文件
    ofstream file;

    // 默认CSV文件名
    string csv_file_path = "auv_lqr_force.csv";

public:
    LQRControllerOnlyNode()
        : Node("LQR_controller_only_node")
    {


        // ======================================================
        // 【新加】初始化随机数
        // ======================================================
        srand(time(0));


        // ==================================================
        // 声明并读取ROS2参数
        // ==================================================
        target_depth =
            this->declare_parameter<double>(
                "target_depth",
                15.0);
                

        csv_file_path =
            this->declare_parameter<string>(
                "csv_file",
                "auv_lqr_force.csv");



        // ==================================================
        // 订阅目标深度
        //
        // 没有收到话题时使用默认值20 m
        // 收到消息后使用新的目标深度
        // ==================================================
        target_depth_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/target_depth",
                10,
                std::bind(
                    &LQRControllerOnlyNode::target_depth_callback,
                    this,
                    std::placeholders::_1));
        velocity_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
             "/auv/vertical_velocity",
            10,
                std::bind(
                &LQRControllerOnlyNode::velocity_callback,
            this,
            std::placeholders::_1));

        // ==================================================
        // 订阅Gazebo当前深度
        // ==================================================
        current_depth_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/auv/current_trueposition",
                10,
                std::bind(
                    &LQRControllerOnlyNode::current_depth_callback,
                    this,
                    std::placeholders::_1));

        // ==================================================
        // 发布控制输出
        // ==================================================
        output_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/output",
                10);

        // 发布误差
        error_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/error",
                10);



        // ==================================================
        // 创建CSV文件
        // std::ios::out表示每次启动覆盖旧文件
        // ==================================================
        file.open(
            csv_file_path,
            std::ios::out);

        if (!file.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "CSV文件打开失败: %s",
                csv_file_path.c_str());
        }
        else
        {
   file
<< "time,"
<< "target_depth,"
<< "current_depth,"
<< "velocity,"
<< "error,"
<< "output"

// ======================================================
// 【新加】CSV继续追加测量值和滤波值
// 原来的6列没有删除
// ======================================================
<< ",measured_depth"
<< ",filtered_depth"
<< ",measured_velocity"
<< ",filtered_velocity"

<< endl;

            RCLCPP_INFO(
                this->get_logger(),
                "CSV文件已创建: %s",
                csv_file_path.c_str());
        }

        // ==================================================
        // 创建5 Hz控制定时器
        // ==================================================
        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &LQRControllerOnlyNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "LQR控制器节点启动成功");

        RCLCPP_INFO(
            this->get_logger(),
            "目标深度: %.3f m",
            target_depth);

        RCLCPP_INFO(
            this->get_logger(),
            "订阅目标深度: /target_depth");

        RCLCPP_INFO(
            this->get_logger(),
            "订阅Gazebo深度: /auv/current_trueposition");

        RCLCPP_INFO(
            this->get_logger(),
            "发布控制输出: /auv/output");
        RCLCPP_INFO(this->get_logger(),
            "订阅Gazebo速度: /auv/vertical_velocity");
    }

    ~LQRControllerOnlyNode()
    {
        if (file.is_open())
        {
            file.flush();
            file.close();
        }
    }

private:
    // ==================================================
    // 接收目标深度
    // ==================================================
    void target_depth_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_depth = msg->data;

    }
void velocity_callback(
const std_msgs::msg::Float64::SharedPtr msg)
{
    velocity = msg->data;
    received_velocity = true;
}
    // ==================================================
    // 接收Gazebo当前深度
    // ==================================================
    void current_depth_callback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_depth = msg->data;
        received_current_depth = true;
    }

    // ==================================================
    // 发布Float64数据
    // ==================================================
    void publish_double(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub,
        double value)
    {
        std_msgs::msg::Float64 msg;
        msg.data = value;

        pub->publish(msg);
    }

    // ==================================================
    // LQR控制循环
    // ==================================================
    void timer_callback()
    {
      if (!received_current_depth || !received_velocity)
{
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "等待深度或速度反馈");

    return;
};

        double time =
            static_cast<double>(step) * dt;


        // ======================================================
        // 【新加1】保存Gazebo真实深度和真实速度
        //
        // 后面LQR临时使用滤波值
        // 控制计算结束后再恢复
        // ======================================================
        double true_depth = current_depth;
        double true_velocity = velocity;


        // ======================================================
        // 【新加2】加入测量噪声
        //
        // measured_depth：
        // 真实深度 + [-0.03,0.03] m随机噪声
        //
        // measured_velocity：
        // 真实速度 + [-0.01,0.01] m/s随机噪声
        // ======================================================
        measured_depth =
            current_depth
            + noise(depth_noise_amplitude);

        measured_velocity =
            velocity
            + noise(velocity_noise_amplitude);


        // ======================================================
        // 【新加3】低通滤波
        //
        // 第一次直接使用测量值进行初始化
        // ======================================================
        if (!filter_initialized)
        {
            filtered_depth =
                measured_depth;

            filtered_velocity =
                measured_velocity;

            filter_initialized = true;
        }
        else
        {
            // 深度低通滤波
            filtered_depth =
                filtered_depth
                * filter_coefficient
                +
                (1-filter_coefficient)
                * measured_depth;

            // 速度低通滤波
            filtered_velocity =
                filtered_velocity
                * filter_coefficient
                +
                (1-filter_coefficient)
                * measured_velocity;
        }


        // ======================================================
        // 【新加4】
        //
        // 为了完全保留你下面原来的controller.output这一行，
        // 这里只是临时让current_depth和velocity等于滤波值。
        //
        // 因此LQR实际使用：
        //
        // filtered_depth
        // filtered_velocity
        // ======================================================
        current_depth = filtered_depth;
        velocity = filtered_velocity;


        // ==================================================
        // 根据目标深度和Gazebo深度计算LQR输出
        // ==================================================
        double output =
        controller.output(target_depth,current_depth,velocity,dt);


        // ======================================================
        // 【新加5】
        //
        // LQR计算完成后立即恢复Gazebo真实值
        //
        // 所以后面的：
        // error
        // current_depth
        // velocity
        //
        // 仍然全部是真实状态
        // ======================================================
        current_depth = true_depth;
        velocity = true_velocity;


        // 深度误差
        double error =
            current_depth - target_depth;

  
        // ==================================================
        // 发布控制数据
        // ==================================================
        publish_double(
            output_pub_,
            output);

        publish_double(
            error_pub_,
            error);

 

        // 每1秒显示一次
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "target=%.3f, current=%.3f, error=%.3f, output=%.3f",
            target_depth,
            current_depth,
            error,
            output);

        // ==================================================
        // 写入CSV
        // ==================================================
        if (file.is_open())
        {
file
<< time << ","
<< target_depth << ","
<< current_depth << ","
<< velocity << ","
<< error << ","
<< output

// ======================================================
// 【新加】CSV后面追加噪声与滤波数据
// ======================================================
<< "," << measured_depth
<< "," << filtered_depth
<< "," << measured_velocity
<< "," << filtered_velocity

<< endl;
        // 实验过程中持续写入磁盘，防止异常退出后数据丢失
            file.flush();
        }

        step++;
    }
};


// ======================================================
// 主函数
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<LQRControllerOnlyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}