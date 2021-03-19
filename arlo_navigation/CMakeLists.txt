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
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  people_msgs
  visualization_msgs
  message_generation
  gazebo
  gazebo_msgs


)

add_service_files(
	FILES
	clearleftlane.srv
	goalnotfound.srv
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
   cfg/progressivelayer.cfg
 )


catkin_package(
  INCLUDE_DIRS 
  LIBRARIES arlo_navigation simple_layers
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS
)
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
 # include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_executable(markfreespace src/markfreespace.cpp)
add_executable(posefinder src/posefinder.cpp)
add_executable(radialposefinder src/radialposefinder.cpp)
add_executable(tester src/tester.cpp)

add_dependencies(markfreespace ${${PROJECT_NAME}_EXPORTED_TARGETS} arlo_navigation_generate_messages_cpp)
add_dependencies(posefinder ${${PROJECT_NAME}_EXPORTED_TARGETS} arlo_navigation_generate_messages_cpp)
add_dependencies(radialposefinder ${PROJECT_NAME}_gencfg)
add_dependencies(tester ${PROJECT_NAME}_gencfg)

target_link_libraries(markfreespace ${catkin_LIBRARIES})
target_link_libraries(posefinder ${catkin_LIBRARIES})
target_link_libraries(radialposefinder ${catkin_LIBRARIES})
target_link_libraries(tester ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
