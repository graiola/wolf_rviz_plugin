#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>

#include "home_tool.h"

namespace wolf_rviz_plugin
{
HomeTool::HomeTool()
{
  shortcut_key_ = 'h';
}

void HomeTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  setName("Home");

  try
  {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/home", 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("HomeTool", e.what());
  }
}

void HomeTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::PoseStamped goal;
  goal.pose.orientation = tf2::toMsg(quat);
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = ros::Time::now();
  //ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
  //         "Angle: %.3f\n",
  //         fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
  //         goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
  //         goal.pose.orientation.w, theta);
  pub_.publish(goal);
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::HomeTool, rviz::Tool)
