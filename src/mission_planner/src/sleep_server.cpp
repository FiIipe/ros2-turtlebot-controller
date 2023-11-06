#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/sleep_during.hpp"

class SleepActionServer : public rclcpp::Node
{
public:
  using SleepDuring = custom_interfaces::action::SleepDuring;
  using GoalHandleSleepDuring = rclcpp_action::ServerGoalHandle<SleepDuring>;

  explicit SleepActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("sleep_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SleepDuring>(
      this,
      "sleep_action",
      std::bind(&SleepActionServer::handle_goal, this, _1, _2),
      std::bind(&SleepActionServer::handle_cancel, this, _1),
      std::bind(&SleepActionServer::handle_accepted, this, _1));

    // Create a publisher object, able to push messages
    m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<SleepDuring>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SleepDuring::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: sleep for %li seconds", goal->sleep_duration);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSleepDuring> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSleepDuring> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SleepActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSleepDuring> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    auto message = geometry_msgs::msg::Twist();

    message.linear.x = 0.0;
    message.angular.z = 0.0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SleepDuring::Feedback>();
    auto timeStart = this->now().seconds();

    auto result = std::make_shared<SleepDuring::Result>();

    while (rclcpp::ok() && this->now().seconds() < timeStart + goal->sleep_duration) {
      if (goal_handle->is_canceling()) {
        result->result = "false";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->time_elapsed = this->now().seconds() - timeStart;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback, time elapsed: %li ", feedback->time_elapsed);

      // And finally publish the message
      m_publisher->publish(message);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = "true";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SleepActionServer>();

  rclcpp::spin(node);

  return 0;
}
