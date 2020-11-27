#include <ros/ros.h>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "posefinder");
  ROS_INFO("Moin");
  ros::spin();
  return 0;
}
