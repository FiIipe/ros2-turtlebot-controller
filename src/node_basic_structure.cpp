#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtlebotController : public rclcpp::Node
{
  public:
    TurtlebotController()
    : Node("turtlebot_controller")
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

      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<TurtlebotController>());

  rclcpp::shutdown();
  return 0;
}