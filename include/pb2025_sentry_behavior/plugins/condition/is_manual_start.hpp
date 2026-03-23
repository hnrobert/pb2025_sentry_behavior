#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MANUAL_START_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MANUAL_START_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace pb2025_sentry_behavior
{
class IsManualStartCondition : public BT::SimpleConditionNode
{
public:
  IsManualStartCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkManualStart();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsManualStartCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_MANUAL_START_HPP_
