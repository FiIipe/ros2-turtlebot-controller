#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/move_during.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class TurtlebotControl : public rclcpp::Node
{
public:
  explicit TurtlebotControl()
  : Node("turtlebot_control"), m_vel_linear(0.0), m_vel_angular(0.0), m_turtle_enabled(false) {

    RCLCPP_INFO_STREAM(this->get_logger(), "Turtlebot controller node started!");

    RCLCPP_INFO_STREAM(this->get_logger(), "Current controller state: " << m_turtle_enabled);

    this->action_server_ = rclcpp_action::create_server<MoveDuring>(
      this,
      "move_during",
      std::bind(&TurtlebotControl::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TurtlebotControl::handle_cancel, this, std::placeholders::_1),
      std::bind(&TurtlebotControl::handle_accepted, this, std::placeholders::_1));

    // Create a publisher object, able to push messages
    m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    m_vel_linear = this->declare_parameter("linear_velocity", 0.0);
    m_vel_angular = this->declare_parameter("angular_velocity", 0.0);
  }

private:

  using MoveDuring = custom_interfaces::action::MoveDuring;
  using GoalHandleMoveDuring = rclcpp_action::ServerGoalHandle<MoveDuring>;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveDuring::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: move turtle during %li seconds", goal->movement_duration);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
                           
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveDuring> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }                               

  void handle_accepted(const std::shared_ptr<GoalHandleMoveDuring> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TurtlebotControl::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveDuring> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);

    auto message = geometry_msgs::msg::Twist();

    message.linear.x = m_vel_linear;
    message.angular.z = m_vel_angular;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveDuring::Feedback>();
    auto timeStart = this->now().seconds();

    auto result = std::make_shared<MoveDuring::Result>();

    while (rclcpp::ok() && this->now().seconds() < timeStart + goal->movement_duration) {
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

    double m_vel_linear;
    double m_vel_angular;
    bool m_turtle_enabled;

    rclcpp_action::Server<MoveDuring>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotControl>());
  rclcpp::shutdown();

  return 0;
}