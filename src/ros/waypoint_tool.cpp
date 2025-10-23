#include "waypoint_tool.h"

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>
#include <rviz/config.h>   // optional since we forward-declared, but harmless

namespace wolf_rviz_plugin
{

WaypointTool::WaypointTool()
{
  shortcut_key_ = 'g';
}

void WaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.0f, 0.0f, 1.0f, 1.0f);

  // Configurable topic; RViz will persist it and our generator can rewrite it per robot.
  topic_property_ = new rviz::StringProperty(
      "Topic", "waypoints",
      "The topic on which to publish the waypoint pose.",
      getPropertyContainer(), SLOT(updateTopic()), this);

  label_property_ = new rviz::StringProperty(
      "Label", "Waypoints",
      "Text shown on the RViz toolbar for this tool instance.",
      getPropertyContainer(), SLOT(updateLabel()), this);


  updateTopic();
  updateLabel();

  // IMPORTANT: do NOT call setName("Waypoint") here.
  // RViz will use the Name from the .rviz file (e.g., "Waypoint ras_10").
}

void WaypointTool::updateLabel() {
  if (label_property_)
    setName(label_property_->getString());
}

void WaypointTool::updateTopic()
{
  try
  {
    pub_.shutdown();
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("WaypointTool", e.what());
  }
}

void WaypointTool::onPoseSet(double x, double y, double z, double theta)
{
  const std::string fixed_frame = context_->getFixedFrame().toStdString();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = fixed_frame;
  msg.header.stamp    = ros::Time::now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation = tf2::toMsg(quat);

  pub_.publish(msg);
}

void WaypointTool::save(rviz::Config config) const
{
  PoseTool::save(config);
  if (topic_property_)
    config.mapSetValue("Topic", topic_property_->getString());
}

void WaypointTool::load(const rviz::Config& config)
{
  PoseTool::load(config);
  QString t;
  if (config.mapGetString("Topic", &t) && topic_property_)
    topic_property_->setString(t);
  updateTopic();
}

} // namespace wolf_rviz_plugin

PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::WaypointTool, rviz::Tool)
