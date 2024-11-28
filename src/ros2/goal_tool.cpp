#include "goal_tool.h"

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace wolf_rviz_plugin
{

GoalTool::GoalTool() : wolf_rviz_plugin::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 't';

  topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "target",
        "The topic on which to publish target pose",
        getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
}

GoalTool::~GoalTool() = default;

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  qos_profile_property_->initialize([this](rclcpp::QoS profile)
  { this->qos_profile_ = profile; });
  setName("Target");
  updateTopic();
}

void GoalTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void GoalTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;

  goal.pose.orientation = orientationAroundZAxis(theta);

  logPose("Target", goal.pose.position, goal.pose.orientation, theta, fixed_frame);

  publisher_->publish(goal);
}
} // namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::GoalTool, rviz_common::Tool)

