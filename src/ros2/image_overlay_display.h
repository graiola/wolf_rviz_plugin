#ifndef WOLF_IMAGE_OVERLAY_DISPLAY_H
#define WOLF_IMAGE_OVERLAY_DISPLAY_H

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_default_plugins/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <wolf_msgs/msg/image_overlay_array.hpp>
#include <wolf_msgs/msg/image_overlay.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>

#include <cv_bridge/cv_bridge.h>
#include <rviz_default_plugins/image/ros_image_texture.hpp>

#include <map>
#include <mutex>

namespace wolf_rviz_plugin
{

class ImageOverlayDisplay : public rviz_common::Display
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
  void processOverlay(const wolf_msgs::msg::ImageOverlay& overlay);
  void overlayCallback(const wolf_msgs::msg::ImageOverlayArray::SharedPtr msg);
  void clearDisplay();

  rviz_common::properties::RosTopicProperty* topic_property_;
  rviz_common::properties::TfFrameProperty* tf_frame_property_;
  rviz_common::properties::FloatProperty* meters_per_pixel_property_;

  rclcpp::Subscription<wolf_msgs::msg::ImageOverlayArray>::SharedPtr overlay_sub_;
  rclcpp::Node::SharedPtr raw_node_;

  struct OverlayData
  {
    geometry_msgs::msg::Pose pose;
    sensor_msgs::msg::Image image;
    std::shared_ptr<rviz_default_plugins::ROSImageTexture> texture;
    Ogre::ManualObject* manual_object;
    Ogre::SceneNode* scene_node;
    Ogre::MaterialPtr material;
  };

  std::map<int, OverlayData> overlays_;

  std::mutex overlays_mutex_;
};

}  // namespace wolf_rviz_plugin

#endif // WOLF_IMAGE_OVERLAY_DISPLAY_H
