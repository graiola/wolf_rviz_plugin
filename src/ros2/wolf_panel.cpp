#include <stdio.h>

#include "wolf_panel.h"

using namespace rt_gui;

namespace wolf_rviz_plugin
{

WolfPanel::WolfPanel( QWidget* parent )
  : rviz_common::Panel( parent )
{
  // Use wolf_rviz name
  std::string ros_namespace = "wolf_rviz";
  ros_server_node_.init(ros_namespace,this);
}

void WolfPanel::save( rviz_common::Config config ) const
{
  rviz_common::Panel::save( config );
}

void WolfPanel::load( const rviz_common::Config& config )
{
  rviz_common::Panel::load( config );
}

} // end namespace wolf_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wolf_rviz_plugin::WolfPanel,rviz_common::Panel)
