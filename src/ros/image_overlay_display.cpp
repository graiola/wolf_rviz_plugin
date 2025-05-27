#include "image_overlay_display.h"
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/ogre_logging.h>
#include <cv_bridge/cv_bridge.h>

namespace wolf_rviz_plugin
{

ImageOverlayDisplay::ImageOverlayDisplay()
{
}

ImageOverlayDisplay::~ImageOverlayDisplay()
{
  overlays_.clear();
}

void ImageOverlayDisplay::onInitialize()
{
  scene_root_ = scene_node_->createChildSceneNode();
  sub_ = nh_.subscribe("image_overlays", 1, &ImageOverlayDisplay::processMessage, this);
}

void ImageOverlayDisplay::reset()
{
  overlays_.clear();
  if (scene_root_) {
    scene_root_->removeAndDestroyAllChildren();
  }
}

void ImageOverlayDisplay::update(float wall_dt, float ros_dt)
{
  // No need to do anything unless animation is required
}

void ImageOverlayDisplay::processMessage(const wolf_msgs::ImageOverlayArray& msg)
{
  for (const auto& overlay : msg.overlays)
  {
    int id = overlay.id;
    const auto& pose = overlay.pose;
    const auto& label = overlay.label;
    sensor_msgs::Image image_msg = overlay.image;

    if (image_msg.encoding.empty()) {
      std::cerr << "ImageOverlayDisplay: Image encoding is empty, assuming RGBA8." << std::endl;
      image_msg.encoding = sensor_msgs::image_encodings::RGBA8;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, "rgba8");
    } catch (const cv_bridge::Exception& e) {
      std::cerr << "ImageOverlayDisplay: cv_bridge conversion failed: " << e.what() << std::endl;
      continue;
    }

    const cv::Mat& img = cv_ptr->image;
    if (img.empty() || img.cols == 0 || img.rows == 0)
    {
      std::cerr << "ImageOverlayDisplay: Skipping invalid image for id: " << id << std::endl;
      continue;
    }

    if (overlays_.find(id) == overlays_.end()) {
      overlays_[id] = std::make_shared<ImageOverlay>(
          context_->getSceneManager(), scene_root_, image_msg, std::to_string(id));
      overlays_[id]->setScale(1.0, 1.0);  // Tune this if needed
    }

    overlays_[id]->setPose(pose);
    overlays_[id]->setVisible(true);
  }
}

}  // namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::ImageOverlayDisplay, rviz::Display)
