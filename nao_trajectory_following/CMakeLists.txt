cmake_minimum_required(VERSION 2.8.3)
project(nao_trajectory_following)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
CATKIN_DEPENDS 
   rospy
   std_msgs
   geometry_msgs
   nav_msgs
   visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   nodes/nao_writer_naoqi.py
   nodes/message_echoer_nao.py
   nodes/trajectory_visualizer.py
   nodes/writing_surface_positioner.py
   nodes/publish_svg_traj.py
   nodes/nao_camera_controllable.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.) 
 install(FILES
   launch/message_echoer_nao.launch
   launch/nao_writing_on_surface.launch
   launch/draw_svg_demo.launch
   launch/nao_camera_controllable.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )
 install(FILES
   config/nao_writing_rviz_config.rviz
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
 )
 install(FILES
   res/nao.svg
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/res
 )

