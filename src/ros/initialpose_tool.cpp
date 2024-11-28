#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>

#include "initialpose_tool.h"

namespace wolf_rviz_plugin
{

InitialPoseTool::InitialPoseTool()
{
  shortcut_key_ = 'p';

  topic_property_ = new rviz::StringProperty( "Topic", "initialpose",
                                              "The topic on which to publish initial pose estimates.",
                                              getPropertyContainer(), SLOT( updateTopic() ), this );

  std_dev_x_ = new rviz::FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_y_ = new rviz::FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_z_ = new rviz::FloatProperty("Y std deviation", 0.5, "Z standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_theta_ =
      new rviz::FloatProperty("Theta std deviation", M_PI / 12.0,
                        "Theta standard deviation for initial pose [rad]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
}

void InitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  setName("3D Pose Estimate");
  updateTopic();
}

void InitialPoseTool::updateTopic()
{
  try
  {
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>( topic_property_->getStdString(), 1 );
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("InitialPoseTool", e.what());
  }
}

void InitialPoseTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = z;

  geometry_msgs::Quaternion quat_msg;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.pose.orientation = tf2::toMsg(quat);
  pose.pose.covariance[6 * 0 + 0] = std::pow(std_dev_x_->getFloat(), 2);
  pose.pose.covariance[6 * 1 + 1] = std::pow(std_dev_y_->getFloat(), 2);
  pose.pose.covariance[6 * 2 + 2] = std::pow(std_dev_z_->getFloat(), 2);
  pose.pose.covariance[6 * 5 + 5] = std::pow(std_dev_theta_->getFloat(), 2);
  ROS_INFO("Setting pose: %.3f %.3f %.3f %.3f [frame=%s]", x, y, z, theta, fixed_frame.c_str());
  pub_.publish(pose);
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::InitialPoseTool, rviz::Tool)
