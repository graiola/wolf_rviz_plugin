#ifndef POSE_TOOL_H
#define POSE_TOOL_H

#include <OgreVector3.h>
#include <QCursor>
#include <ros/ros.h>
#include <rviz/tool.h>

namespace rviz {
  class Arrow;
  class DisplayContext;
}

namespace wolf_rviz_plugin
{

class PoseTool : public rviz::Tool
{
public:
  PoseTool();
  ~PoseTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  rviz::Arrow* arrow_;

  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;
};

} // namespace

#endif
