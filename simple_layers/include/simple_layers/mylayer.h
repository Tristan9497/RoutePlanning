#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
//tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_client.h>

#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <string.h>
#include <simple_layers/reset.h>
namespace simple_layer_namespace
{

class MyLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  MyLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  //service to reset the layer usable as recovery or to help the planner to get past obstacles
  ros::ServiceServer service;
  bool reset(simple_layers::reset::Request &trigger, simple_layers::reset::Response &state);
  virtual void matchSize();
  geometry_msgs::Point32 TransformPoint(geometry_msgs::Point32 point,std::string frame2);
protected:
  void drawCost(int mx, int my,double xc,double yc, unsigned int rad, double maxcost, double startcost);
  void setcheckcost(double wx, double wy,double cost);
  void lineCallback(const sensor_msgs::PointCloud& points);
  ros::Subscriber line_sub_;
  sensor_msgs::PointCloud linepoints;
  boost::recursive_mutex lock_;
  bool rolling_window_;
  std::string topic;
  double oldx;
  double oldy;

private:
  bool layertrigger,sim_time;
  void reconfigureCB(simple_layers::ProgressiveLayerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<simple_layers::ProgressiveLayerConfig> *dsrv_;



};
}
#endif
