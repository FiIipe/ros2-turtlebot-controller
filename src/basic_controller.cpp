#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtlebotControl : public rclcpp::Node
{
  public:
    TurtlebotControl()
    : Node("turtlebot_control"), m_vel_linear(0.0), m_vel_angular(0.0) {

      RCLCPP_INFO(this->get_logger(), "Turtlebot controller node started.");

      // Create a publisher object, able to push messages
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      
      m_vel_linear = this->declare_parameter("linear_velocity", 0.0);
      m_vel_angular = this->declare_parameter("angular_velocity", 0.0);

      // Create a timer to publish our messages periodically
      m_timer = this->create_wall_timer(50ms, std::bind(&TurtlebotControl::run, this));
    }

  private:
    void run()
    {
      auto message = geometry_msgs::msg::Twist();

      message.linear.x = m_vel_linear;
      message.angular.z = m_vel_angular;

      m_publisher->publish(message);
    }
    
    double m_vel_linear;
    double m_vel_angular;
    
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