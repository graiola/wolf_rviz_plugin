#include "image_overlay.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreSceneNode.h>
#include <iostream>
#include <cstdlib>  // rand()

ImageOverlay::ImageOverlay(Ogre::SceneManager* scene_manager,
                           Ogre::SceneNode* parent_node,
                           const sensor_msgs::Image& msg,
                           const std::string& overlay_id)
  : scene_manager_(scene_manager), width_(1.0), height_(1.0)
{
  node_ = parent_node->createChildSceneNode();
  quad_ = scene_manager_->createManualObject("ImageQuad_" + overlay_id);
  node_->attachObject(quad_);
  loadImageTexture(msg);
  updateQuad();
}

ImageOverlay::~ImageOverlay()
{
  if (quad_) scene_manager_->destroyManualObject(quad_);
  if (node_) scene_manager_->destroySceneNode(node_);
}

void ImageOverlay::loadImageTexture(const sensor_msgs::Image& msg)
{
  if (msg.encoding.empty() || msg.data.empty() || msg.width == 0 || msg.height == 0) {
    std::cerr << "ImageOverlay: Invalid image message.\n";
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "rgba8");  // Force RGBA8 conversion
  } catch (cv_bridge::Exception& e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    return;
  }

  cv::Mat img = cv_ptr->image;

  if (img.empty() || img.cols == 0 || img.rows == 0) {
    std::cerr << "ImageOverlay: Empty or zero-size image after cv_bridge.\n";
    return;
  }

  if (!img.isContinuous()) {
    img = img.clone();  // ensures tight layout
  }

  texture_name_ = "OverlayTexture_" + std::to_string(rand());

  try {
    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().createManual(
        texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, img.cols, img.rows, 0,
        Ogre::PF_BYTE_RGBA, Ogre::TU_STATIC_WRITE_ONLY);

    Ogre::PixelBox pixel_box(img.cols, img.rows, 1, Ogre::PF_BYTE_RGBA, img.data);
    pixel_box.rowPitch = img.step / 4;  // 4 bytes per pixel

    tex->getBuffer()->blitFromMemory(pixel_box);

    std::string material_name = "OverlayMaterial_" + std::to_string(rand());
    material_ = Ogre::MaterialManager::getSingleton().create(material_name,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
    pass->createTextureUnitState(texture_name_);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setCullingMode(Ogre::CULL_NONE);
  } catch (const Ogre::Exception& e) {
    std::cerr << "ImageOverlay: Ogre exception: " << e.getFullDescription() << std::endl;
  }
}


void ImageOverlay::updateQuad()
{
  if (material_.isNull()) return;

  quad_->clear();
  quad_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP);

  float w = width_ / 2.0f;
  float h = height_ / 2.0f;

  quad_->position(-w, 0.0f, -h); quad_->textureCoord(0, 0);
  quad_->position(w, 0.0f, -h);  quad_->textureCoord(1, 0);
  quad_->position(-w, 0.0f, h); quad_->textureCoord(0, 1);
  quad_->position(w, 0.0f, h);  quad_->textureCoord(1, 1);

  quad_->end();
}

void ImageOverlay::setPose(const geometry_msgs::Pose& pose)
{
  Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  Ogre::Quaternion orientation(pose.orientation.w,
                               pose.orientation.x,
                               pose.orientation.y,
                               pose.orientation.z);
  node_->setPosition(position);
  node_->setOrientation(orientation);
}

void ImageOverlay::setScale(float width_m, float height_m)
{
  width_ = width_m;
  height_ = height_m;
  updateQuad();
}

void ImageOverlay::setVisible(bool visible)
{
  if (node_)
    node_->setVisible(visible);
}
