#pragma once

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageOverlay
{
public:
  ImageOverlay(Ogre::SceneManager* scene_manager,
               Ogre::SceneNode* parent_node,
               const sensor_msgs::Image& msg,
               const std::string& overlay_id);

  ~ImageOverlay();

  void setPose(const geometry_msgs::Pose& pose);
  void setScale(float width_m, float height_m);
  void setVisible(bool visible);

private:
  void loadImageTexture(const sensor_msgs::Image& msg);
  void updateQuad();

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* node_;
  Ogre::ManualObject* quad_;
  Ogre::MaterialPtr material_;
  std::string texture_name_;
  float width_;
  float height_;
};