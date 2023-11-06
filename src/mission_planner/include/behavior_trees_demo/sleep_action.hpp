#include "behaviortree_ros2/bt_action_node.hpp"
#include "custom_interfaces/action/sleep_during.hpp"

using namespace BT;

class SleepAction: public RosActionNode<custom_interfaces::action::SleepDuring>
{
public:
  SleepAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<custom_interfaces::action::SleepDuring>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
