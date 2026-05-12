#include "home_tool.h"

#include <pluginlib/class_list_macros.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
  #include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>
#include <rviz/config.h>

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

  // Configurable + persistent topic (default "home").
  topic_property_ = new rviz::StringProperty(
      "Topic", "home",
      "The topic on which to publish the Home pose.",
      getPropertyContainer(), SLOT(updateTopic()), this);

  label_property_ = new rviz::StringProperty(
      "Label", "Home",
      "Text shown on the RViz toolbar for this tool instance.",
      getPropertyContainer(), SLOT(updateLabel()), this);


  updateTopic();
  updateLabel();

  // Do NOT call setName("Home") here — RViz will use the name from the .rviz file
  // (e.g., "Home ras_10") which your generator sets per robot.
}

void HomeTool::updateLabel() {
  if (label_property_)
    setName(label_property_->getString());
}

void HomeTool::updateTopic()
{
  try
  {
    pub_.shutdown();
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("HomeTool", e.what());
  }
}

void HomeTool::onPoseSet(double x, double y, double z, double theta)
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

void HomeTool::save(rviz::Config config) const
{
  PoseTool::save(config);
  if (topic_property_)
    config.mapSetValue("Topic", topic_property_->getString());
}

void HomeTool::load(const rviz::Config& config)
{
  PoseTool::load(config);
  QString t;
  if (config.mapGetString("Topic", &t) && topic_property_)
    topic_property_->setString(t);
  updateTopic();
}

} // namespace wolf_rviz_plugin

PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::HomeTool, rviz::Tool)
