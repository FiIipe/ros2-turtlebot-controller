#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtlebotControl : public rclcpp_lifecycle::LifecycleNode
{
  public:
    TurtlebotControl(const std::string & node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)), m_vel_linear(0.0), m_vel_angular(0.0){

      RCLCPP_INFO_STREAM(this->get_logger(), "Turtlebot controller node started!");
    }

    void run()
    {
      auto message = geometry_msgs::msg::Twist();

      message.linear.x = m_vel_linear;
      message.angular.z = m_vel_angular;

      m_publisher->publish(message);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) 
    {
      // Create a publisher object, able to push messages
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      m_vel_linear = this->declare_parameter("linear_velocity", 0.0);
      m_vel_angular = this->declare_parameter("angular_velocity", 0.0);

      // Create a timer to publish our messages periodically
      m_timer = this->create_wall_timer(50ms, std::bind(&TurtlebotControl::run, this));

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_active(const rclcpp_lifecycle::State &) 
    {
      m_publisher->on_activate();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
    {
      m_publisher->on_deactivate();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
      m_publisher.reset();
      m_timer.reset();

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
      // In our shutdown phase, we release the shared pointers to the
      // timer and publisher. These entities are no longer available
      // and our node is "clean".
      m_timer.reset();
      m_publisher.reset();

      RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str());

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

  private:

    double m_vel_linear;
    double m_vel_angular;

    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> m_publisher;
};

int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<TurtlebotControl>());
  // rclcpp::shutdown();

  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<TurtlebotControl> lc_node =
    std::make_shared<TurtlebotControl>("lc_turtlebotControl");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();


  return 0;
}