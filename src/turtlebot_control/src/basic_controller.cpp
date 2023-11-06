#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtlebotControl : public rclcpp::Node
{
  public:
    TurtlebotControl()
    : Node("turtlebot_control"), m_vel_linear(0.0), m_vel_angular(0.0), m_turtle_enabled(false) {

      RCLCPP_INFO_STREAM(this->get_logger(), "Turtlebot controller node started!");

      RCLCPP_INFO_STREAM(this->get_logger(), "Current controller state: " << m_turtle_enabled);

      // Create a publisher object, able to push messages
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      m_vel_linear = this->declare_parameter("linear_velocity", 0.0);
      m_vel_angular = this->declare_parameter("angular_velocity", 0.0);

      m_service_server = this->create_service<std_srvs::srv::SetBool>(
        "enable_turtle", std::bind(
          &TurtlebotControl::enable_turtlebot, 
          this, 
          std::placeholders::_1, 
          std::placeholders::_2));

      // Create a timer to publish our messages periodically
      m_timer = this->create_wall_timer(50ms, std::bind(&TurtlebotControl::run, this));
    }

  private:
    void run()
    {
      if (!m_turtle_enabled) {
        return;
      }

      auto message = geometry_msgs::msg::Twist();

      message.linear.x = m_vel_linear;
      message.angular.z = m_vel_angular;

      m_publisher->publish(message);
    }
    
    void enable_turtlebot(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      if (m_turtle_enabled == request->data)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Current control state matches request");
        response->success = false; 
        response->message = "Current control state matches request";
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching control state to " << request->data);
        m_turtle_enabled = true;
        response->success = true; 
        response->message = "Control state updated";
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "Current controller state: " << m_turtle_enabled);      
    }

    double m_vel_linear;
    double m_vel_angular;
    bool m_turtle_enabled;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_service_server; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotControl>());
  rclcpp::shutdown();

  return 0;
}