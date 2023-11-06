#include "behavior_trees_demo/control_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool ControlAction::setGoal(RosActionNode::Goal &goal)
{
  auto movement_duration = getInput<unsigned>("msec");
  goal.movement_duration = movement_duration.value();
  return true;
}

NodeStatus ControlAction::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->result ? "true" : "false" );

  return wr.result->result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus ControlAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void ControlAction::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
}

// Plugin registration.
// The class ControlAction will self register with name  "Control".
CreateRosNodePlugin(ControlAction, "Control");
