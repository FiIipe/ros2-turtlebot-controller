#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"

#include "behavior_trees_demo/control_action.hpp"
#include "behavior_trees_demo/sleep_action.hpp"


using namespace BT;

  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree>
        <Sequence>
            <ControlAction name="ControlMovementA" msec="5"/>
            <Fallback>
                <Timeout msec="5000">
                   <SleepAction name="SleepA" msec="4"/>
                </Timeout>
                <ControlAction name="ControlMovementB" msec="3"/>
            </Fallback>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("control_client");

  BehaviorTreeFactory factory;
  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "control_action";

  RosNodeParams params_;
  params_.nh = nh;
  params_.default_port_value = "sleep_action";


  factory.registerNodeType<ControlAction>("ControlAction", params);
  factory.registerNodeType<SleepAction>("SleepAction", params_);

  auto tree = factory.createTreeFromText(xml_text);

  for(int i=0; i<2; i++){
    tree.tickWhileRunning();
  }

  return 0;
}
