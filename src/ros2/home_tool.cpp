#include "home_tool.h"

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace wolf_rviz_plugin
{

HomeTool::HomeTool() : wolf_rviz_plugin::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'h';

  topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "home",
        "The topic on which to publish home pose",
        getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);
}

void HomeTool::onInitialize()
{
  PoseTool::onInitialize();
  qos_profile_property_->initialize([this](rclcpp::QoS profile)
  { this->qos_profile_ = profile; });
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  setName("Home");
  updateTopic();
}

void HomeTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void HomeTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.orientation = tf2::toMsg(quat);
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = clock_->now();
  //ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
  //         "Angle: %.3f\n",
  //         fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
  //         goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
  //         goal.pose.orientation.w, theta);
  publisher_->publish(goal);
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::HomeTool, rviz_common::Tool)
