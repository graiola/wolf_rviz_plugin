#include "pose_tool.h"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace wolf_rviz_plugin
{
    PoseTool::PoseTool() : rviz_common::Tool(), arrow_(nullptr), angle_(0)
    {
      projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    PoseTool::~PoseTool() = default;

    void PoseTool::onInitialize()
    {
      arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
      arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
      arrow_->getSceneNode()->setVisible(false);
    }

    void PoseTool::activate()
    {
      setStatus("Left click and drag mouse to set xy position and orientation. Right click and drag mouse to set z position.");
      state_ = Position;
    }

    void PoseTool::deactivate()
    {
      arrow_->getSceneNode()->setVisible(false);
    }

    int PoseTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
    {
      auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);

      if (event.leftDown())
      {
        return processMouseLeftButtonPressed(point_projection_on_xy_plane);
      }
      else if (event.type == QEvent::MouseMove && event.left())
      {
        return processMouseMoved(point_projection_on_xy_plane, event);
      }
      else if (event.leftUp())
      {
        return processMouseLeftButtonReleased();
      }

      return 0;
    }

    int PoseTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
    {
      int flags = 0;
      assert(state_ == Position);
      if (xy_plane_intersection.first)
      {
        arrow_position_ = xy_plane_intersection.second;
        arrow_->setPosition(arrow_position_);

        state_ = Orientation;
        flags |= Render;
      }
      return flags;
    }

    int PoseTool::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection, rviz_common::ViewportMouseEvent &event)
    {
      int flags = 0;
      static double init_z;
      static double prev_z;
      const double z_scale = 50;

      if (state_ == Orientation)
      {
        // compute angle in x-y plane
        if (xy_plane_intersection.first)
        {
          angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
          makeArrowVisibleAndSetOrientation(angle_);

          if (event.right())
            state_ = Height;

          init_z = arrow_position_.z;
          prev_z = event.y;
          flags |= Render;
        }
      }

      if (state_ == Height)
      {
        double z = event.y;
        double dz = z - prev_z;
        prev_z = z;
        arrow_position_.z -= dz / z_scale;
        arrow_->setPosition(arrow_position_);
        flags |= Render;
      }

      return flags;
    }

    int PoseTool::processMouseLeftButtonReleased()
    {
      int flags = 0;
      if (state_ == Orientation || state_ == Height)
      {
        onPoseSet(arrow_position_.x, arrow_position_.y, arrow_position_.z, angle_);
        flags |= (Finished | Render);
      }

      return flags;
    }

    void PoseTool::makeArrowVisibleAndSetOrientation(double angle)
    {
      arrow_->getSceneNode()->setVisible(true);

      // we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
      Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

      arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
    }

    double PoseTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
    {
      return std::atan2(start_point.y - end_point.y, start_point.x - end_point.x);
    }

    geometry_msgs::msg::Quaternion PoseTool::orientationAroundZAxis(double angle)
    {
      auto orientation = geometry_msgs::msg::Quaternion();

      orientation.x = 0.0;
      orientation.y = 0.0;
      orientation.z = std::sin(angle) / (2 * std::cos(angle / 2));
      orientation.w = std::cos(angle / 2);

      return orientation;
    }

    void PoseTool::logPose(std::string designation, geometry_msgs::msg::Point position,
                             geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
    {
      RVIZ_COMMON_LOG_INFO_STREAM(
          "Setting " << designation << " pose: Frame:" << frame << ", Position(" << position.x << ", "
                     << position.y << ", " << position.z << "), Orientation(" << orientation.x << ", "
                     << orientation.y << ", " << orientation.z << ", " << orientation.w << ") = Angle: " << angle);
    }
} // namespace wolf_rviz_plugin

