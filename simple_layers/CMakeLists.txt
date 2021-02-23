cmake_minimum_required(VERSION 3.0.2)
project(simple_layers)


find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
  pluginlib
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  std_msgs
  message_generation
  
)


add_service_files(
  FILES
  reset.srv
)

 generate_messages(
   DEPENDENCIES

 )

 generate_dynamic_reconfigure_options(
   cfg/ProgressiveLayer.cfg
 )


catkin_package(
  INCLUDE_DIRS
  LIBRARIES simple_layers arlo_navigation
  CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs costmap_2d pluginlib
  DEPENDS
)




include_directories(${catkin_INCLUDE_DIRS} include)
add_library(mylayer src/mylayer.cpp)
add_dependencies(mylayer ${catkin_EXPORTED_TARGETS})