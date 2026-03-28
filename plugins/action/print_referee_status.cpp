// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/plugins/action/print_referee_status.hpp"

namespace pb2025_sentry_behavior
{

PrintRefereeStatusAction::PrintRefereeStatusAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PrintRefereeStatusAction::providedPorts()
{
  return {
    BT::InputPort<std_msgs::msg::String>(
      "raw_performance", "{@referee_raw_performance}",
      "Raw JSON String from /referee/robot_performance"),
  };
}

BT::NodeStatus PrintRefereeStatusAction::tick()
{
  auto raw = getInput<std_msgs::msg::String>("raw_performance");
  if (!raw) {
    RCLCPP_WARN(
      logger_,
      "[WAITING] /referee/robot_performance not received yet. "
      "Ensure dji_referee_protocol is running.");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger_, "========== Referee Connection OK ==========");
  RCLCPP_INFO(logger_, "[/referee/robot_performance] %s", raw->data.c_str());
  RCLCPP_INFO(logger_, "============================================");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::PrintRefereeStatusAction>("PrintRefereeStatus");
}
