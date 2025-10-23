#ifndef GOAL_TOOL_H
#define GOAL_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include "pose_tool.h"
#endif

namespace rviz {
class Arrow;
class DisplayContext;
class StringProperty;
class Config;   // for save/load
}

namespace wolf_rviz_plugin {

class GoalTool : public PoseTool
{
  Q_OBJECT
public:
  GoalTool();
  ~GoalTool() override {}

  void onInitialize() override;

  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();
  void updateLabel();

private:
  ros::NodeHandle nh_;
  ros::Publisher  pub_;
  rviz::StringProperty* topic_property_{nullptr};
  rviz::StringProperty* label_property_{nullptr};
};

} // namespace wolf_rviz_plugin

#endif // GOAL_TOOL_H
