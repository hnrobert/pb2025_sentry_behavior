#include "pb2025_sentry_behavior/plugins/action/navigate_goal_pose.hpp"

#include <cmath>

#include "pb2025_sentry_behavior/custom_types.hpp"
#include "tf2/utils.h"

namespace pb2025_sentry_behavior
{

NavigateGoalPoseAction::NavigateGoalPoseAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList NavigateGoalPoseAction::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
      BT::InputPort<std::string>("map_frame", "map", "Map frame"),
      BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<double>("position_tolerance", 0.25, "Position tolerance in meters"),
      BT::InputPort<double>("yaw_tolerance", 0.35, "Yaw tolerance in radians"),
    });
}

BT::NodeStatus NavigateGoalPoseAction::onStart()
{
  auto status = BT::RosTopicPubStatefulActionNode<geometry_msgs::msg::PoseStamped>::onStart();
  // Always return RUNNING: this node uses its own arrival check in onRunning(),
  // not the base class duration mechanism.
  if(status == BT::NodeStatus::FAILURE)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

bool NavigateGoalPoseAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  std::string map_frame = "map";
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    return false;
  }

  getInput("map_frame", map_frame);
  goal_ = *goal;
  goal_.header.stamp = node_->now();
  goal_.header.frame_id = map_frame;
  msg = goal_;
  return true;
}

BT::NodeStatus NavigateGoalPoseAction::onRunning()
{
  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  double position_tolerance = 0.25;
  double yaw_tolerance = 0.35;

  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);
  getInput("position_tolerance", position_tolerance);
  getInput("yaw_tolerance", yaw_tolerance);

  try {
    const auto transform = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    const double dx = goal_.pose.position.x - transform.transform.translation.x;
    const double dy = goal_.pose.position.y - transform.transform.translation.y;
    const double distance = std::hypot(dx, dy);
    const double goal_yaw = tf2::getYaw(goal_.pose.orientation);
    const double robot_yaw = tf2::getYaw(transform.transform.rotation);
    const double yaw_error = std::atan2(std::sin(goal_yaw - robot_yaw), std::cos(goal_yaw - robot_yaw));

    if (distance <= position_tolerance && std::abs(yaw_error) <= yaw_tolerance) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "NavigateGoalPose waiting for transform %s -> %s: %s", map_frame.c_str(),
      base_frame.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::NavigateGoalPoseAction, "NavigateGoalPose");
