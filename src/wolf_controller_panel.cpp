#include <stdio.h>

#include "wolf_controller_panel.h"

using namespace rt_gui;

namespace wolf_rviz_plugin
{

WolfControllerPanel::WolfControllerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  ros_server_node_.spawn("wolf_controller_panel",this);
}

void WolfControllerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void WolfControllerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::WolfControllerPanel,rviz::Panel)
