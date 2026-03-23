#include "pb2025_sentry_behavior/plugins/condition/is_manual_start.hpp"

namespace pb2025_sentry_behavior
{

IsManualStartCondition::IsManualStartCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsManualStartCondition::checkManualStart, this), config)
{
}

BT::PortsList IsManualStartCondition::providedPorts()
{
  return {
    BT::InputPort<std_msgs::msg::Int32>("key_port", "{@manual_start}", "Manual start port on blackboard"),
    BT::InputPort<int>("start_value", 1, "Minimum value to trigger manual start"),
  };
}

BT::NodeStatus IsManualStartCondition::checkManualStart()
{
  int start_value = 1;
  auto msg = getInput<std_msgs::msg::Int32>("key_port");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  getInput("start_value", start_value);
  return msg->data >= start_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsManualStartCondition>("IsManualStart");
}
