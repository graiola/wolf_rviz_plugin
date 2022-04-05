#ifndef GOAL_TOOL_H
#define GOAL_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
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

class GoalTool : public PoseTool
{
  Q_OBJECT
public:
  GoalTool();
  ~GoalTool() override
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
