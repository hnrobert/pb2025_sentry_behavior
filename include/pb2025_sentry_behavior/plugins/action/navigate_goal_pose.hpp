#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_GOAL_POSE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_GOAL_POSE_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{
class NavigateGoalPoseAction
: public BT::RosTopicPubStatefulActionNode<geometry_msgs::msg::PoseStamped>
{
public:
  NavigateGoalPoseAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override;

private:
  geometry_msgs::msg::PoseStamped goal_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_GOAL_POSE_HPP_
