#include "image_overlay_display.h"

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/status_property.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

namespace wolf_rviz_plugin
{

using namespace rviz;

ImageOverlayDisplay::ImageOverlayDisplay()
{
  topic_property_ = new RosTopicProperty("Image Overlay Topic", "image_overlays",
    ros::message_traits::datatype<wolf_msgs::ImageOverlayArray>(),
    "ImageOverlayArray topic to subscribe to.",
    this, SLOT(updateTopic()));

  tf_frame_property_ = new TfFrameProperty("Fixed Frame", "map",
    "Fixed frame to use for positioning overlays.",
    this, nullptr, true);

  meters_per_pixel_property_ = new FloatProperty("Meters per Pixel", 0.002,
    "Image scale in RViz meters per image pixel.",
    this);
}

ImageOverlayDisplay::~ImageOverlayDisplay()
{
  clearDisplay();
}

void ImageOverlayDisplay::onInitialize()
{
  tf_frame_property_->setFrameManager(context_->getFrameManager());
}

void ImageOverlayDisplay::onEnable()
{
  updateTopic();
}

void ImageOverlayDisplay::onDisable()
{
  overlay_sub_.shutdown();
  clearDisplay();
}

void ImageOverlayDisplay::reset()
{
  Display::reset();
  clearDisplay();
}

void ImageOverlayDisplay::updateTopic()
{
  overlay_sub_ = nh_.subscribe(topic_property_->getTopicStd(), 1,
                               &ImageOverlayDisplay::overlayCallback, this);
}

void ImageOverlayDisplay::overlayCallback(const wolf_msgs::ImageOverlayArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(overlays_mutex_);
  clearDisplay();

  for (const auto& overlay : msg->overlays)
  {
    processOverlay(overlay);
  }

  context_->queueRender();
}

void ImageOverlayDisplay::clearDisplay()
{
  for (auto& pair : overlays_)
  {
    if (pair.second.manual_object)
    {
      scene_manager_->destroyManualObject(pair.second.manual_object);
    }
    if (pair.second.scene_node)
    {
      scene_node_->removeChild(pair.second.scene_node);
      scene_manager_->destroySceneNode(pair.second.scene_node);
    }
  }
  overlays_.clear();
}

void ImageOverlayDisplay::processOverlay(const wolf_msgs::ImageOverlay& overlay)
{
  OverlayData data;
  data.pose = overlay.pose;
  data.image = overlay.image;
  data.texture.reset(new ROSImageTexture());

  if (overlay.image.width == 0 || overlay.image.height == 0)
  {
    ROS_WARN("Received image with 0 width or height (ID %d), skipping.", overlay.id);
    return;
  }

  try
  {
    auto cv_ptr = cv_bridge::toCvCopy(overlay.image, sensor_msgs::image_encodings::RGBA8);

    if (cv_ptr->image.empty() || cv_ptr->image.cols == 0 || cv_ptr->image.rows == 0)
    {
      ROS_WARN("Converted OpenCV image is empty or has 0 size (ID %d), skipping.", overlay.id);
      return;
    }

    cv::Scalar border_color(255, 255, 255, 0);
    cv::copyMakeBorder(cv_ptr->image, cv_ptr->image, 1, 1, 1, 1, cv::BORDER_CONSTANT, border_color);
    cv::flip(cv_ptr->image, cv_ptr->image, -1);
    data.texture->addMessage(cv_ptr->toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
    return;
  }

  // Create material
  std::stringstream mat_name;
  mat_name << "ImageOverlayMaterial_" << overlay.id;
  data.material = Ogre::MaterialManager::getSingleton().create(
    mat_name.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto* pass = data.material->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
  pass->setLightingEnabled(false);

  auto* tex_state = pass->createTextureUnitState(data.texture->getTexture()->getName());
  tex_state->setTextureFiltering(Ogre::TFO_NONE);
  tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  // Create quad
  const float meters_per_pixel = meters_per_pixel_property_->getFloat();
  float width = overlay.image.width * meters_per_pixel;
  float height = overlay.image.height * meters_per_pixel;

  std::stringstream obj_name;
  obj_name << "OverlayObject_" << overlay.id;

  data.manual_object = scene_manager_->createManualObject(obj_name.str());
  data.manual_object->begin(data.material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  Ogre::Vector3 corners[4] = {
    {-width / 2, height / 2, 0},  // top left
    {width / 2, height / 2, 0},   // top right
    {-width / 2, -height / 2, 0}, // bottom left
    {width / 2, -height / 2, 0},  // bottom right
  };

  Ogre::Vector2 uvs[4] = {
    {0, 0},
    {1, 0},
    {0, 1},
    {1, 1}
  };

  for (int i = 0; i < 4; ++i)
  {
    data.manual_object->position(corners[i]);
    data.manual_object->textureCoord(uvs[i]);
  }

  data.manual_object->triangle(0, 1, 2);
  data.manual_object->triangle(1, 3, 2);
  data.manual_object->end();

  // Create scene node
  data.scene_node = scene_node_->createChildSceneNode();

  // Apply pose
  //Ogre::Vector3 position(data.pose.position.x, data.pose.position.y, data.pose.position.z);
  //Ogre::Quaternion orientation(data.pose.orientation.w, data.pose.orientation.x,
  //                             data.pose.orientation.y, data.pose.orientation.z);


  // Flatten Z to project on ground
  Ogre::Vector3 position(data.pose.position.x, data.pose.position.y, 0.0f);

// Orient image to lie flat on ground
  Ogre::Quaternion orientation(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);

  data.scene_node->setPosition(position);
  data.scene_node->setOrientation(orientation);

  data.scene_node->attachObject(data.manual_object);

  overlays_[overlay.id] = data;
}

void ImageOverlayDisplay::update(float wall_dt, float ros_dt)
{
  boost::mutex::scoped_lock lock(overlays_mutex_);

  for (auto& pair : overlays_)
  {
    if (pair.second.texture)
    {
      pair.second.texture->update();
    }
  }
}

}  // namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::ImageOverlayDisplay, rviz::Display)
