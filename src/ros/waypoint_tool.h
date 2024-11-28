#ifndef WAYPOINT_TOOL_H
#define WAYPOINT_TOOL_H

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

class WaypointTool : public PoseTool
{
  Q_OBJECT
public:
  WaypointTool();
  ~WaypointTool() override
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
