#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/qos.hpp"


class QoSDemo : public rclcpp::Node {
public:
  QoSDemo() : Node("integer_publisher"), counter_(0) {

    publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "qos_demo", rclcpp::QoS(rclcpp::KeepLast(25)).best_effort());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&QoSDemo::publishInteger, this));
  }

private:
  void publishInteger() {
    auto message = std_msgs::msg::Int32();
    message.data = counter_;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
    counter_++;
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QoSDemo>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}