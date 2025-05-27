#pragma once

#include <rviz/display.h>
#include <ros/ros.h>
#include <wolf_msgs/ImageOverlayArray.h>
#include "image_overlay.h"

namespace wolf_rviz_plugin
{
class ImageOverlayDisplay : public rviz::Display
{
Q_OBJECT
public:
    ImageOverlayDisplay();
    ~ImageOverlayDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

private:
  void processMessage(const wolf_msgs::ImageOverlayArray& msg);

  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  std::map<int, std::shared_ptr<ImageOverlay>> overlays_;
  Ogre::SceneNode* scene_root_;
};
}  // namespace wolf_rviz_plugin
