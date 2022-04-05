#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <rviz/geometry.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>

#include "pose_tool.h"

namespace wolf_rviz_plugin
{


PoseTool::PoseTool() : Tool(), arrow_(nullptr)
{
}

PoseTool::~PoseTool()
{
  delete arrow_;
}

void PoseTool::onInitialize()
{
  arrow_ = new rviz::Arrow(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void PoseTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int PoseTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  if (event.leftDown())
  {
    ROS_ASSERT(state_ == Position);

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
    {
      pos_ = intersection;
      arrow_->setPosition(pos_);

      state_ = Orientation;
      flags |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      // compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        arrow_->getSceneNode()->setVisible(true);

        // we need base_orient, since the arrow goes along the -z axis by default (for historical
        // reasons)
        Ogre::Quaternion orient_x =
            Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

        arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        flags |= Render;
      }
    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation)
    {
      // compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if(rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        onPoseSet(pos_.x, pos_.y, angle);

        flags |= (Finished | Render);
      }
    }
  }

  return flags;
}

} // namespace wolf_rviz_plugin

