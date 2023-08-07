#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ServiceClient : public rclcpp::Node
{
  public:
    ServiceClient()
    : Node("service_client") {

      RCLCPP_INFO_STREAM(this->get_logger(), "Service client initialized!");

      m_service_client = this->create_client<std_srvs::srv::SetBool>("enable_turtle");

      // Create a timer to publish our messages periodically
      main();
    }

  private:

    void main()
    {
      while(rclcpp::ok()) {

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

        if (true == m_state) {
          request->data = 0;
          m_state = 0;
        } else {
          request->data = 1;
          m_state = 1;
        }


        while (!m_service_client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
          }
          RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto result = m_service_client->async_send_request(request);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "AHAHAHAH");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }

        rclcpp::sleep_for(5s);
      }

    }

    bool m_state;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_service_client; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceClient>());
  rclcpp::shutdown();

  return 0;
}