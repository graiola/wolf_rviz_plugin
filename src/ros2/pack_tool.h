#ifndef PACK_TOOL_H
#define PACK_TOOL_H

#include <QObject>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include "pose_tool.h"

namespace rviz_common
{
  class DisplayContext;
  namespace properties
  {
    class StringProperty;
    class QosProfileProperty;
  } // namespace properties
} // namespace rviz_common

namespace wolf_rviz_plugin
{

class PackTool : public PoseTool
{
  Q_OBJECT
public:
  PackTool();
  ~PackTool() override
  {
  }
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty *topic_property_;
  rviz_common::properties::QosProfileProperty *qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

} // namespace wolf_rviz_plugin

#endif
