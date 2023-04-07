#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtlebotController : public rclcpp::Node
{
  public:
    TurtlebotController()
    : Node("turtlebot_controller") {

      RCLCPP_INFO(this->get_logger(), "Turtlebot controller node started.");

      // Create a publisher object, able to push messages
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      
      // Create a timer to publish our messages periodically
      timer_ = this->create_wall_timer(50ms, std::bind(&TurtlebotController::run, this));
    }

  private:
    void run()
    {
      auto message = geometry_msgs::msg::Twist();

      // TODO: Our Control code goes here 
      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotController>());
  rclcpp::shutdown();

  return 0;
}