#ifndef WOLF_PANEL_H
#define WOLF_PANEL_H

#ifndef Q_MOC_RUN
# include <rviz_common/panel.hpp>
# include <rt_gui_ros/support/server.h>
#endif

namespace wolf_rviz_plugin
{

class WolfPanel: public rviz_common::Panel
{

Q_OBJECT
public:

  WolfPanel( QWidget* parent = nullptr );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;

private:

  rt_gui::RosServerNode ros_server_node_;
};

} // end namespace wolf_rviz_plugin

#endif // WOLF_PANEL_H
