#include <stdio.h>

#include "wolf_panel.h"

using namespace rt_gui;

namespace wolf_rviz_plugin
{

WolfPanel::WolfPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  ros_server_node_.init("wolf_panel",this);
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
