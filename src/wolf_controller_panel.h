#ifndef WOLF_CONTROLLER_PANEL_H
#define WOLF_CONTROLLER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
# include <rt_gui/support/server.h>
#endif

namespace wolf_rviz_plugin
{

class WolfControllerPanel: public rviz::Panel
{

Q_OBJECT
public:

  WolfControllerPanel( QWidget* parent = nullptr );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:

  rt_gui::RosServerNode ros_server_node_;

};

} // end namespace wolf_rviz_plugin

#endif // WOLF_CONTROLLER_PANEL_H
