#include <functional>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class GoalCoordinate : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    GoalCoordinate() : Node("GoalCoordinate")
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GoalCoordinate::send_goal, this));
    }
    
    float x = 1.0;
    void send_goal()
    {
        using namespace std::placeholders;
        this->timer_->cancel();
        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
            rclcpp::shutdown();
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        // goal_msg.pose.header.stamp = rclcpp::Time::now();

        goal_msg.pose.pose.position.x = 1.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&GoalCoordinate::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&GoalCoordinate::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&GoalCoordinate::result_callback, this, _1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        auto distance_feedback_msg = std_msgs::msg::String();
        distance_feedback_msg.data = "Remaining Distance from Destination: " + std::to_string(feedback->distance_remaining);
        RCLCPP_INFO(this->get_logger(), "%s", distance_feedback_msg.data.c_str());
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalCoordinate>());
    rclcpp::shutdown();
    return 0;
}
