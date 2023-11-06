#include "behaviortree_ros2/bt_action_node.hpp"
#include "custom_interfaces/action/move_during.hpp"

using namespace BT;

class ControlAction: public RosActionNode<custom_interfaces::action::MoveDuring>
{
public:
  ControlAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<custom_interfaces::action::MoveDuring>(name, conf, params)
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
