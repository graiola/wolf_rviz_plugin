#ifndef INITIALPOSE_TOOL_H
#define INITIALPOSE_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include "pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
class FloatProperty;
}

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
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;

  rviz::FloatProperty* std_dev_x_;
  rviz::FloatProperty* std_dev_y_;
  rviz::FloatProperty* std_dev_z_;
  rviz::FloatProperty* std_dev_theta_;
};

} // namespace wolf_rviz_plugin

#endif
