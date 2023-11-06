#include "behavior_trees_demo/sleep_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(RosActionNode::Goal &goal)
{
  auto sleep_duration = getInput<unsigned>("msec");
  goal.sleep_duration = sleep_duration.value();
  return true;
}

NodeStatus SleepAction::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->result ? "true" : "false" );

  return wr.result->result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void SleepAction::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
}

// Plugin registration.
// The class SleepAction will self register with name  "Sleep".
CreateRosNodePlugin(SleepAction, "Sleep");
