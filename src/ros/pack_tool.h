#ifndef PACK_TOOL_H
#define PACK_TOOL_H

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

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

} // namespace wolf_rviz_plugin

#endif
