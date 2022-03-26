#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include<QDebug>

#include "waypoints_tool.h"

namespace wolf_rviz_plugin
{

WaypointsTool::WaypointsTool()
  : moving_waypoints_node_( nullptr )
  , current_waypoints_property_( nullptr )
{
  shortcut_key_ = 'l';
  waypoints_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/navpoints", 1000);  // FIXME
  waypoints_scale_.x = waypoints_scale_.y = waypoints_scale_.z = 0.25;
}

WaypointsTool::~WaypointsTool()
{
  for( unsigned i = 0; i < waypoints_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( waypoints_nodes_[ i ]);
  }
}

void WaypointsTool::onInitialize()
{
  waypoints_resource_ = "package://wolf_rviz_plugin/media/waypoints.dae";

  if( rviz::loadMeshFromResource( waypoints_resource_ ).isNull() )
  {
    ROS_ERROR( "WaypointsTool: failed to load model resource '%s'.", waypoints_resource_.c_str() );
    return;
  }

  moving_waypoints_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( waypoints_resource_ );
  moving_waypoints_node_->attachObject( entity );
  moving_waypoints_node_->setVisible( false );
  moving_waypoints_node_->scale(waypoints_scale_);
}

void WaypointsTool::activate()
{
  if( moving_waypoints_node_ )
  {
    moving_waypoints_node_->setVisible( true );

    current_waypoints_property_ = new rviz::VectorProperty( "waypoints " + QString::number( waypoints_nodes_.size() ));
    current_waypoints_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_waypoints_property_ );
  }
}

void WaypointsTool::deactivate()
{
  if( moving_waypoints_node_ )
  {
    moving_waypoints_node_->setVisible( false );
    delete current_waypoints_property_;
    current_waypoints_property_ = nullptr;
  }
}

int WaypointsTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_waypoints_node_ )
  {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_waypoints_node_->setVisible( true );
    moving_waypoints_node_->setPosition( intersection );
    current_waypoints_property_->setVector( intersection );

    if( event.leftDown() )
    {
      makewaypoints( intersection );
      current_waypoints_property_ = nullptr; // Drop the reference so that deactivate() won't remove it.
      return Render | Finished;
    }
  }
  else
  {
    moving_waypoints_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the waypoints.
  }
  return Render;
}

// This is a helper function to create a new waypoints in the Ogre scene and save its scene node in a list.
void WaypointsTool::makewaypoints( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( waypoints_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  node->setScale(waypoints_scale_);
  waypoints_nodes_.push_back( node );

  waypoint_.point.x = position.x;
  waypoint_.point.y = position.y;
  waypoint_.point.z = position.z;
  waypoint_.header.seq++;
  waypoint_.header.stamp = ros::Time::now();
  waypoint_.header.frame_id = "/map"; // FIXME

  waypoints_pub_.publish(waypoint_);
}

void WaypointsTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our Waypoints
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``Waypoints_config``) to store
  // the list.
  rviz::Config Waypoints_config = config.mapMakeChild( "Waypoints" );

  // To read the positions and names of the Waypoints, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single waypoints and append it to the Config list.
    rviz::Config waypoints_config = Waypoints_config.listAppendNew();
    // Into the waypoints's config we store its name:
    waypoints_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( waypoints_config );
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void WaypointsTool::load( const rviz::Config& config )
{
  // Here we get the "Waypoints" sub-config from the tool config and loop over its entries:
  rviz::Config Waypoints_config = config.mapGetChild( "Waypoints" );
  int num_Waypoints = Waypoints_config.listLength();
  for( int i = 0; i < num_Waypoints; i++ )
  {
    rviz::Config waypoints_config = Waypoints_config.listChildAt( i );
    // At this point each ``waypoints_config`` represents a single waypoints.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "waypoints " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``waypoints_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    waypoints_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( waypoints_config );
    // We finish each waypoints by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible waypoints object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makewaypoints( prop->getVector() );
  }
}


} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::WaypointsTool,rviz::Tool )
