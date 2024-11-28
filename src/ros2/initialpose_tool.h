#ifndef INITIALPOSE_TOOL_H
#define INITIALPOSE_TOOL_H

#include <QObject>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
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
    class DisplayContext;
    class FloatProperty;
  } // namespace properties
} // namespace rviz_common

namespace wolf_rviz_plugin
{

class InitialPoseTool : public PoseTool
{
  Q_OBJECT
public:
  InitialPoseTool();
  ~InitialPoseTool() override
  {
  }
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::FloatProperty* std_dev_x_;
  rviz_common::properties::FloatProperty* std_dev_y_;
  rviz_common::properties::FloatProperty* std_dev_z_;
  rviz_common::properties::FloatProperty* std_dev_theta_;

  rviz_common::properties::StringProperty *topic_property_;
  rviz_common::properties::QosProfileProperty *qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

} // namespace wolf_rviz_plugin

#endif
