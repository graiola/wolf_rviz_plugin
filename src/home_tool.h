#ifndef HOMNE_TOOL_H
#define HOMNE_TOOL_H

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
}

namespace wolf_rviz_plugin
{

class HomeTool : public PoseTool
{
  Q_OBJECT
public:
  HomeTool();
  ~HomeTool() override
  {
  }
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

} // namespace wolf_rviz_plugin

#endif
