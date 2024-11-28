cmake_minimum_required(VERSION 3.5)
project(wolf_rviz_plugin)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

#ROS
find_package(rt_gui_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    catkin_package(LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS rviz pluginlib)
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
    message(STATUS "Use RVIZ")

    find_package(rviz REQUIRED)
    find_package(geometry_msgs REQUIRED)

    include_directories(
        ${rviz_INCLUDE_DIRS}
        ${rt_gui_ros_INCLUDE_DIRS}
        ${pluginlib_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS})

    set(ROS_LIBRARIES
        ${pluginlib_LIBRARIES}
        ${rt_gui_ros_LIBRARIES}
        ${rviz_LIBRARIES})

elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
    message(STATUS "Use RVIZ2")

    find_package(rviz2 REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(rviz_rendering REQUIRED)
    find_package(rviz_ogre_vendor REQUIRED)
    find_package(rviz_default_plugins REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)

    include_directories(
        ${rviz2_INCLUDE_DIRS}
        ${rt_gui_ros_INCLUDE_DIRS}
        ${pluginlib_INCLUDE_DIRS}
        ${rviz_rendering_INCLUDE_DIRS}
        ${rviz_common_INCLUDE_DIRS}
        ${rviz_default_plugins_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS})

    set(ROS_LIBRARIES
        ${pluginlib_LIBRARIES}
        ${rt_gui_ros_LIBRARIES}
        ${rviz2_LIBRARIES}
        ${rviz_rendering_LIBRARIES}
        ${rviz_common_LIBRARIES}
        ${rviz_default_plugins_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${tf2_geometry_msgs_LIBRARIES})

    # export the plugin to be imported by rviz2
    pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
    ament_export_libraries(${PROJECT_NAME})
    ament_package()

endif()

find_package(${RVIZ_VERSION} REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(QT_LIBRARIES Qt5::Widgets)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
#add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/${ROS_VERSION}/wolf_panel.cpp
  src/${ROS_VERSION}/pose_tool.cpp
  src/${ROS_VERSION}/waypoint_tool.cpp
  src/${ROS_VERSION}/home_tool.cpp
  src/${ROS_VERSION}/pack_tool.cpp
  src/${ROS_VERSION}/goal_tool.cpp
  src/${ROS_VERSION}/initialpose_tool.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${ROS_LIBRARIES})

## Install rules
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(FILES plugin_description.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

install(DIRECTORY icons/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/icons)
