#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

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
// PID控制器类
// ======================================================
class pid
{
private:
    double kp;
    double ki;
    double kd;

    double p_term = 0.0;
    double i_term = 0.0;
    double d_term = 0.0;

    double integral = 0.0;
    double previous_error = 0.0;

    bool first = true;

    // 微分滤波后的结果
    double filtered_derivative = 0.0;

    // 微分低通滤波系数
    double derivative_filter_coefficient = 0.85;

    // 上一次控制输出
    double previous_output = 0.0;

    // 输出变化率限制
    double output_rate = 3.0;

    // 误差死区
    double deadband = 0.02;

    // 积分限幅
    double integral_max = 20.0;
    double integral_min = -20.0;

    // 输出限幅
    double output_max = 5.0;
    double output_min = -5.0;

public:
    pid(double p, double i, double d)
        : kp(p), ki(i), kd(d)
    {
    }

    void setIntegralLimit(double max_value, double min_value)
    {
        integral_max = max_value;
        integral_min = min_value;
    }

    void setOutputLimit(double max_value, double min_value)
    {
        output_max = max_value;
        output_min = min_value;
    }

    void setDerivativeFilter(double value)
    {
        derivative_filter_coefficient = value;
    }

    void setOutputRate(double value)
    {
        output_rate = value;
    }

    void setDeadband(double value)
    {
        deadband = value;
    }

    void reset()
    {
        integral = 0.0;
        previous_error = 0.0;

        first = true;

        filtered_derivative = 0.0;
        previous_output = 0.0;

        p_term = 0.0;
        i_term = 0.0;
        d_term = 0.0;
    }

    double output(double target, double current, double dt)
    {
        if (dt <= 0.0)
        {
            cerr << "PID dt error" << endl;
            return 0.0;
        }

        // 深度误差：深度向下为正
        double error = target - current;

        // 死区处理
        if (fabs(error) < deadband)
        {
            error = 0.0;
        }

        // 积分
        integral += error * dt;

        // 积分限幅
        integral = limit(
            integral,
            integral_max,
            integral_min);

        // 第一次运行时令微分项为0
        if (first)
        {
            previous_error = error;
        }

        // 原始误差微分
        double derivative =
            (error - previous_error) / dt;

        // 微分低通滤波
        filtered_derivative =
            derivative_filter_coefficient * filtered_derivative
            + (1.0 - derivative_filter_coefficient) * derivative;

        // PID三项
        p_term = kp * error;
        i_term = ki * integral;
        d_term = kd * filtered_derivative;

        // PID原始输出
        double controller_output =
            p_term + i_term + d_term;

        previous_error = error;
        first = false;

        // 输出限幅
        controller_output = limit(
            controller_output,
            output_max,
            output_min);

        // 输出变化率限制
        double maximum_output_change =
            output_rate * dt;

        double output_change =
            controller_output - previous_output;

        output_change = limit(
            output_change,
            maximum_output_change,
            -maximum_output_change);

        controller_output =
            previous_output + output_change;

        // 再次进行总限幅
        controller_output = limit(
            controller_output,
            output_max,
            output_min);

        previous_output = controller_output;

        return controller_output;
    }

    double getP() const
    {
        return p_term;
    }

    double getI() const
    {
        return i_term;
    }

    double getD() const
    {
        return d_term;
    }
};


// ======================================================
// 只负责控制的PID节点
//
// 默认目标深度：20 m
// 默认CSV文件：auv_pid_force.csv
//
// 订阅：
// /target_depth
// /auv/current_trueposition
//
// 发布：
// /auv/output
// /auv/error
// /auv/p_term
// /auv/i_term
// /auv/d_term
//
// 不计算AUV位置
// 不包含auvmodel.update()
// ======================================================
class PidControllerOnlyNode : public rclcpp::Node
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

    // 控制输出发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        output_pub_;

    // 误差发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        error_pub_;

    // PID三项发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        p_term_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        i_term_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        d_term_pub_;

    // 默认目标深度20 m
    double target_depth = 20.0;

    // Gazebo反馈的当前深度
    double current_depth = 0.0;

    // 控制周期0.1 s，即10 Hz
    double dt = 0.1;

    // 是否收到Gazebo深度
    bool received_current_depth = false;

    // PID控制器
    pid controller = pid(0.18, 0.002, 1.5);

    // CSV记录时间步
    std::size_t step = 0;

    // CSV文件
    ofstream file;

    // 默认CSV文件名
    string csv_file_path = "auv_pid_force.csv";

public:
    PidControllerOnlyNode()
        : Node("pid_controller_only_node")
    {
        // ==================================================
        // 声明并读取ROS2参数
        // ==================================================
        target_depth =
            this->declare_parameter<double>(
                "target_depth",
                20.0);

        csv_file_path =
            this->declare_parameter<string>(
                "csv_file",
                "auv_pid_force.csv");

        // ==================================================
        // 设置PID参数
        // ==================================================
        controller.setIntegralLimit(20.0, -20.0);
        controller.setOutputLimit(5.0, -5.0);
        controller.setDerivativeFilter(0.85);
        controller.setOutputRate(3.0);
        controller.setDeadband(0.02);

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
                    &PidControllerOnlyNode::target_depth_callback,
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
                    &PidControllerOnlyNode::current_depth_callback,
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

        // 发布PID三项
        p_term_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/p_term",
                10);

        i_term_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/i_term",
                10);

        d_term_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/d_term",
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
                << "error,"
                << "output,"
                << "p_term,"
                << "i_term,"
                << "d_term"
                << endl;

            RCLCPP_INFO(
                this->get_logger(),
                "CSV文件已创建: %s",
                csv_file_path.c_str());
        }

        // ==================================================
        // 创建10 Hz控制定时器
        // ==================================================
        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &PidControllerOnlyNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "PID控制器节点启动成功");

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
    }

    ~PidControllerOnlyNode()
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

        // 修改目标深度后重置PID，避免旧积分影响新目标
        controller.reset();

        RCLCPP_INFO(
            this->get_logger(),
            "目标深度修改为: %.3f m",
            target_depth);
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
    // PID控制循环
    // ==================================================
    void timer_callback()
    {
        // 必须先收到Gazebo当前深度
        if (!received_current_depth)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待Gazebo深度 /auv/current_trueposition");

            return;
        }

        double time =
            static_cast<double>(step) * dt;

        // ==================================================
        // 根据目标深度和Gazebo深度计算PID输出
        // ==================================================
        double output =
            controller.output(
                target_depth,
                current_depth,
                dt);

        // 深度误差
        double error =
            target_depth - current_depth;

        // PID三项
        double p_term = controller.getP();
        double i_term = controller.getI();
        double d_term = controller.getD();

        // ==================================================
        // 发布控制数据
        // ==================================================
        publish_double(
            output_pub_,
            output);

        publish_double(
            error_pub_,
            error);

        publish_double(
            p_term_pub_,
            p_term);

        publish_double(
            i_term_pub_,
            i_term);

        publish_double(
            d_term_pub_,
            d_term);

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
                << error << ","
                << output << ","
                << p_term << ","
                << i_term << ","
                << d_term
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
        std::make_shared<PidControllerOnlyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}