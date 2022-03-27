#ifndef WAYPOINTS_TOOL_H
#define WAYPOINTS_TOOL_H

#include <rviz/tool.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/PointStamped.h>

#include <OgreVector3.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace wolf_rviz_plugin
{

class WaypointsTool: public rviz::Tool
{
Q_OBJECT
public:
  WaypointsTool();
  ~WaypointsTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  ros::Publisher waypoints_pub_;
  ros::NodeHandle nh_;
  geometry_msgs::PointStamped waypoint_;

  void makewaypoints( const Ogre::Vector3& position );

  std::vector<Ogre::SceneNode*> waypoints_nodes_;
  Ogre::SceneNode* moving_waypoints_node_;
  std::string waypoints_resource_;
  rviz::VectorProperty* current_waypoints_property_;

  Ogre::Vector3 waypoints_scale_;
};

} // end namespace wolf_rviz_plugin

#endif // WAYPOINTS_TOOL_H
