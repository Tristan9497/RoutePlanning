#include<simple_layers/mylayer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::MyLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

MyLayer::MyLayer() {}

void MyLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MyLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  line_sub_ = nh.subscribe("/people", 1, &MyLayer::lineCallback, this);
}

void MyLayer::lineCallback(const sensor_msgs::PointCloud& points)
{
  linepoints = points;
}


void MyLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void MyLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;

}

void MyLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double radius=0.5;
  double dist;
  double cost;
  double mark_x=0;
  double mark_y=0;
  double startcost;
  double maxcost;
  unsigned int mx;
  unsigned int my;

  for (int k=0;k<linepoints.points.size();k++)
  {
//	  radius=linepoints.channels.data()->values.at(k);
//	  mark_x=linepoints.points.at(k).x;
//	  mark_y=linepoints.points.at(k).y;
	  for(double j=mark_y-radius;j<=mark_y+radius;j)
	  {
		  for(double i=mark_x-radius;i<=mark_x+radius;i)
		  {
			  dist=sqrt(pow(mark_x-i,2)+pow(mark_y-j,2));
			  if(dist<radius)
			  {
				  if(worldToMap(i, j, mx, my))
				  {

					  cost=255*exp((log(50/255)*(dist/radius)));
					  //cost=-dist*((maxcost-startcost)/radius)+maxcost;
					  ROS_INFO("%f",cost);
					setCost(mx, my, (unsigned char) cost);


				  }
			  }

			  *min_x = std::min(*min_x, i);
			  *min_y = std::min(*min_y, j);
			  *max_x = std::max(*max_x, i);
			  *max_y = std::max(*max_y, j);
			  i+=getResolution()/2;


		  }
		  j+=getResolution()/2;
	  }
  }

  

}

void MyLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

} // end namespace

