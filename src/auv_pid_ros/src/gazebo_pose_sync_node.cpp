#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class GazeboPoseSyncNode : public rclcpp::Node
{
public:
    GazeboPoseSyncNode() : Node("gazebo_pose_sync_node")
    {
        this->declare_parameter<std::string>("model_name", "simple_auv");
        this->declare_parameter<double>("x_position", 0.0);
        this->declare_parameter<double>("y_position", 0.0);

        model_name_ = this->get_parameter("model_name").as_string();
        x_position_ = this->get_parameter("x_position").as_double();
        y_position_ = this->get_parameter("y_position").as_double();

        set_state_client_ =
            this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

        current_depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_depth",
            10,
            std::bind(&GazeboPoseSyncNode::depthCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "gazebo_pose_sync_node started.");
        RCLCPP_INFO(this->get_logger(), "model_name: %s", model_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Rule: gazebo_z = -current_depth");
    }

private:
    void depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double current_depth = msg->data;
        double gazebo_z = -current_depth;

        if (!set_state_client_->wait_for_service(500ms))
        {
            RCLCPP_WARN(this->get_logger(), "Gazebo /set_entity_state service not available.");
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

        request->state.name = model_name_;
        request->state.reference_frame = "world";

        request->state.pose.position.x = x_position_;
        request->state.pose.position.y = y_position_;
        request->state.pose.position.z = gazebo_z;

        request->state.pose.orientation.x = 0.0;
        request->state.pose.orientation.y = 0.0;
        request->state.pose.orientation.z = 0.0;
        request->state.pose.orientation.w = 1.0;

        request->state.twist.linear.x = 0.0;
        request->state.twist.linear.y = 0.0;
        request->state.twist.linear.z = 0.0;
        request->state.twist.angular.x = 0.0;
        request->state.twist.angular.y = 0.0;
        request->state.twist.angular.z = 0.0;

        set_state_client_->async_send_request(request);

        RCLCPP_INFO(
            this->get_logger(),
            "current_depth = %.2f m -> gazebo_z = %.2f m",
            current_depth,
            gazebo_z
        );
    }

    std::string model_name_;
    double x_position_;
    double y_position_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_depth_sub_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboPoseSyncNode>());
    rclcpp::shutdown();
    return 0;
}