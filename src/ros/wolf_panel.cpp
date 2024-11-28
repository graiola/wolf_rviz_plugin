#include <stdio.h>

#include "wolf_panel.h"

using namespace rt_gui;

namespace wolf_rviz_plugin
{

WolfPanel::WolfPanel( QWidget* parent )
  : rviz::Panel( parent ) 
{
  // Note: / is required to generate the panel in the global namespace
  // i.e. /wolf_panel/... otherwise it inherits whatever namespace rviz has
  //std::string ros_namespace = "/wolf_panel";
  //nh_ = ros::NodeHandle(ros_namespace);
  //ros_server_node_.init(nh_,ros_namespace,this);

  // Use wolf_rviz name
  std::string ros_namespace = "wolf_rviz";
  ros_server_node_.init(ros_namespace,this);
}

void WolfPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void WolfPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::WolfPanel,rviz::Panel)
