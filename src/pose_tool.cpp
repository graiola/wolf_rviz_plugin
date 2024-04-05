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
  setStatus("Left click and drag mouse to set xy position and orientation. Right click and drag mouse to set z position.");
  state_ = Position;
}

void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int PoseTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;
  static Ogre::Vector3 ang_pos;
  static double initz;
  static double prevz;
  static double prevangle;
  const double z_scale = 50;
  Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

  if( event.leftDown() )
  {
    ROS_ASSERT( state_ == Position );
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if(rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
    {
      pos_ = intersection;
      arrow_->setPosition( pos_ );
      state_ = Orientation;
      flags |= Render;
    }
  }
  else if( event.type == QEvent::MouseMove && event.left() )
  {
    if( state_ == Orientation )
    {
      //compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if(rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, cur_pos ))
      {
        double angle = static_cast<double>(atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x ));
        arrow_->getSceneNode()->setVisible( true );
        arrow_->setOrientation( Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );
        if ( event.right() )
          state_ = Height;
        initz = pos_.z;
        prevz = event.y;
        prevangle = angle;
        flags |= Render;
      }
    }
    if ( state_ == Height )
    {
      double z = event.y;
      double dz = z - prevz;
      prevz = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition( pos_ );
      flags |= Render;
    }
  }
  else if( event.leftUp() )
  {
    if( state_ == Orientation || state_ == Height)
    {

      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished|Render);
    }
  }

  return flags;
}

} // namespace wolf_rviz_plugin

