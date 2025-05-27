#ifndef WOLF_IMAGE_OVERLAY_DISPLAY_H
#define WOLF_IMAGE_OVERLAY_DISPLAY_H

#include <rviz/display.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <wolf_msgs/ImageOverlayArray.h>

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz/image/ros_image_texture.h>

#include <map>

namespace wolf_rviz_plugin
{

class ImageOverlayDisplay : public rviz::Display
{
Q_OBJECT
public:
  ImageOverlayDisplay();
  ~ImageOverlayDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateTopic();

private:
  void processOverlay(const wolf_msgs::ImageOverlay& overlay);
  void overlayCallback(const wolf_msgs::ImageOverlayArray::ConstPtr& msg);
  void clearDisplay();

  rviz::RosTopicProperty* topic_property_;
  rviz::TfFrameProperty* tf_frame_property_;
  rviz::FloatProperty* meters_per_pixel_property_;

  ros::Subscriber overlay_sub_;
  ros::NodeHandle nh_;

  struct OverlayData
  {
    geometry_msgs::Pose pose;
    sensor_msgs::Image image;
    boost::shared_ptr<rviz::ROSImageTexture> texture;
    Ogre::ManualObject* manual_object;
    Ogre::SceneNode* scene_node;
    Ogre::MaterialPtr material;
  };

  std::map<int, OverlayData> overlays_;

  boost::mutex overlays_mutex_;
};

}  // namespace rviz

#endif // WOLF_IMAGE_OVERLAY_DISPLAY_H
