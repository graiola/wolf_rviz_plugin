#include "initialpose_tool.h"

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rviz_rendering/objects/arrow.hpp>

namespace wolf_rviz_plugin
{

InitialPoseTool::InitialPoseTool() : wolf_rviz_plugin::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'p';

  topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "initialpose",
        "The topic on which to publish initial pose estimates",
        getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
        topic_property_, qos_profile_);

  std_dev_x_ = new rviz_common::properties::FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty("Y std deviation", 0.5, "Z standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_theta_ =
      new rviz_common::properties::FloatProperty("Theta std deviation", M_PI / 12.0,
                        "Theta standard deviation for initial pose [rad]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
}

void InitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  qos_profile_property_->initialize([this](rclcpp::QoS profile)
  { this->qos_profile_ = profile; });
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  setName("3D Pose Estimate");
  updateTopic();
}

void InitialPoseTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->template create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void InitialPoseTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = clock_->now();
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = z;

  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.pose.orientation = tf2::toMsg(quat);
  pose.pose.covariance[6 * 0 + 0] = std::pow(std_dev_x_->getFloat(), 2);
  pose.pose.covariance[6 * 1 + 1] = std::pow(std_dev_y_->getFloat(), 2);
  pose.pose.covariance[6 * 2 + 2] = std::pow(std_dev_z_->getFloat(), 2);
  pose.pose.covariance[6 * 5 + 5] = std::pow(std_dev_theta_->getFloat(), 2);
  //ROS_INFO("Setting pose: %.3f %.3f %.3f %.3f [frame=%s]", x, y, z, theta, fixed_frame.c_str());
  publisher_->publish(pose);
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::InitialPoseTool, rviz_common::Tool)
