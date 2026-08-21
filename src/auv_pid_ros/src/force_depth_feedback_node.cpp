#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;


// ======================================================
// Gazebo深度与垂向速度反馈节点
//
// Gazebo坐标定义：
// z > 0：向上
// z < 0：向下
//
// 控制系统定义：
// depth > 0：向下
//
// 坐标转换：
// depth = -gazebo_z
// depth_velocity = -gazebo_velocity_z
//
// 发布话题：
// /auv/current_trueposition
// /auv/vertical_velocity
//
// 使用服务：
// /get_entity_state
// ======================================================
class ForceDepthFeedbackNode : public rclcpp::Node
{
private:
    // Gazebo实体状态服务客户端
    rclcpp::Client<
        gazebo_msgs::srv::GetEntityState>::SharedPtr
        get_entity_state_client_;

    // 深度发布器
    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        current_depth_pub_;

    // 垂向速度发布器
    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        vertical_velocity_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // Gazebo中的模型名称
    std::string entity_name_;

    // 参考坐标系
    std::string reference_frame_;

    // Gazebo服务名称
    std::string service_name_;

    // 防止上一次服务请求未完成时再次请求
    bool request_in_progress_ = false;

public:
    ForceDepthFeedbackNode()
        : Node("force_depth_feedback_node")
    {
        // ==================================================
        // 声明ROS2参数
        // ==================================================
        entity_name_ =
            this->declare_parameter<std::string>(
                "entity_name",
                "simple_auv");

        reference_frame_ =
            this->declare_parameter<std::string>(
                "reference_frame",
                "world");

        service_name_ =
            this->declare_parameter<std::string>(
                "service_name",
                "/get_entity_state");

        // ==================================================
        // 创建深度发布器
        // ==================================================
        current_depth_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/current_trueposition",
                10);

        // ==================================================
        // 创建垂向速度发布器
        // ==================================================
        vertical_velocity_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/auv/vertical_velocity",
                10);

        // ==================================================
        // 创建Gazebo实体状态服务客户端
        // ==================================================
        get_entity_state_client_ =
            this->create_client<
                gazebo_msgs::srv::GetEntityState>(
                service_name_);

        // ==================================================
        // 20 Hz读取Gazebo模型状态
        // ==================================================
        timer_ =
            this->create_wall_timer(
                50ms,
                std::bind(
                    &ForceDepthFeedbackNode::timer_callback,
                    this));

        // ==================================================
        // 启动信息
        // ==================================================
        RCLCPP_INFO(
            this->get_logger(),
            "深度与垂向速度反馈节点启动成功");

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo实体名称: %s",
            entity_name_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "参考坐标系: %s",
            reference_frame_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo服务: %s",
            service_name_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "发布深度: /auv/current_trueposition");

        RCLCPP_INFO(
            this->get_logger(),
            "发布垂向速度: /auv/vertical_velocity");
    }

private:
    // ==================================================
    // 发布Float64消息
    // ==================================================
    void publish_double(
        const rclcpp::Publisher<
            std_msgs::msg::Float64>::SharedPtr &publisher,
        double value)
    {
        std_msgs::msg::Float64 message;
        message.data = value;

        publisher->publish(message);
    }

    // ==================================================
    // 定时器回调
    // ==================================================
    void timer_callback()
    {
        // 上一次请求尚未完成
        if (request_in_progress_)
        {
            return;
        }

        // Gazebo服务尚未准备好
        if (!get_entity_state_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "等待Gazebo服务: %s",
                service_name_.c_str());

            return;
        }

        // ==================================================
        // 构造服务请求
        // ==================================================
        auto request =
            std::make_shared<
                gazebo_msgs::srv::GetEntityState::Request>();

        request->name =
            entity_name_;

        request->reference_frame =
            reference_frame_;

        request_in_progress_ = true;

        // ==================================================
        // 异步发送请求
        // ==================================================
        get_entity_state_client_->async_send_request(
            request,
            std::bind(
                &ForceDepthFeedbackNode::response_callback,
                this,
                std::placeholders::_1));
    }

    // ==================================================
    // Gazebo服务响应回调
    // ==================================================
    void response_callback(
        rclcpp::Client<
            gazebo_msgs::srv::GetEntityState>::
            SharedFuture future)
    {
        request_in_progress_ = false;

        gazebo_msgs::srv::GetEntityState::
            Response::SharedPtr response;

        try
        {
            response = future.get();
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "调用GetEntityState服务异常: %s",
                error.what());

            return;
        }

        // ==================================================
        // ROS2 Humble的GetEntityState响应没有status_message
        // 只能检查success
        // ==================================================
        if (!response->success)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "读取实体状态失败，请检查实体名称: %s",
                entity_name_.c_str());

            return;
        }

        // ==================================================
        // 获取Gazebo世界坐标中的位置
        // ==================================================
        const double gazebo_z =
            response->state.pose.position.z;

        // Gazebo世界坐标中的z方向速度
        const double gazebo_velocity_z =
            response->state.twist.linear.z;

        // ==================================================
        // 转换为深度坐标
        //
        // Gazebo：z向上为正
        // 控制系统：深度向下为正
        // ==================================================
        const double current_depth =
            -gazebo_z;

        const double depth_velocity =
            -gazebo_velocity_z;

        // ==================================================
        // 发布深度
        // ==================================================
        publish_double(
            current_depth_pub_,
            current_depth);

        // ==================================================
        // 发布垂向速度
        //
        // depth_velocity > 0：正在下潜
        // depth_velocity < 0：正在上浮
        // ==================================================
        publish_double(
            vertical_velocity_pub_,
            depth_velocity);

        // 每1秒打印一次
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "depth=%.3f m, depth_velocity=%.3f m/s",
            current_depth,
            depth_velocity);
    }
};


// ======================================================
// 主函数
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ForceDepthFeedbackNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}