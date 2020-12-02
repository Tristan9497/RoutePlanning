cmake_minimum_required(VERSION 3.0.2)
project(arlo_navigation)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  road_detection
  tf2_geometry_msgs
)



 generate_messages(
   DEPENDENCIES
   geometry_msgs   
   nav_msgs   
   sensor_msgs   
   std_msgs
 )

 generate_dynamic_reconfigure_options(
   cfg/markfreespace.cfg
   cfg/posefinder.cfg
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arlo_navigation
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(markfreespace src/markfreespace.cpp)
add_executable(posefinder src/posefinder.cpp)

add_dependencies(markfreespace ${${PROJECT_NAME}_EXPORTED_TARGETS} arlo_navigation_generate_messages_cpp)
add_dependencies(posefinder ${${PROJECT_NAME}_EXPORTED_TARGETS} arlo_navigation_generate_messages_cpp)

target_link_libraries(markfreespace ${catkin_LIBRARIES})
target_link_libraries(posefinder ${catkin_LIBRARIES})


