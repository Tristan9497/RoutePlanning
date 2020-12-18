#include <simple_layers/ProgressiveLayerConfig.h>
#include <simple_layers/mylayer.h>
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

  dsrv_ = new dynamic_reconfigure::Server<simple_layers::ProgressiveLayerConfig>(nh);
  dynamic_reconfigure::Server<simple_layers::ProgressiveLayerConfig>::CallbackType cb = boost::bind(
      &MyLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  line_sub_ = nh.subscribe(topic, 1, &MyLayer::lineCallback, this);
  ros::param::set("/use_sim_time",true);

}

void MyLayer::lineCallback(const sensor_msgs::PointCloud& points)
{
	boost::recursive_mutex::scoped_lock lock(lock_);
	linepoints=points;
}


void MyLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void MyLayer::reconfigureCB(simple_layers::ProgressiveLayerConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  topic = config.point_topic;

}
geometry_msgs::Point32 MyLayer::TransformPoint(geometry_msgs::Point32 point, std::string frame,std::string frame2)
	{
	geometry_msgs::TransformStamped transformStamped;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	//function that transforms a point32 from frame2 to frame1 and returns the transformed point
	//if transformation fails an empty point is returned and an error msg gets printed
	geometry_msgs::Point pt;
	geometry_msgs::Point32 pt32;

	pt.x=point.x;
	pt.y=point.y;
	pt.z=point.z;


    try {
        transformStamped = tfBuffer.lookupTransform( frame,frame2,ros::Time(0),ros::Duration(1));

        tf2::doTransform(pt,pt, transformStamped);
    	pt32.x=pt.x;
    	pt32.y=pt.y;
    	pt32.z=pt.z;

    	return pt32;

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s  empty point returned", ex.what());
    }


	}
void MyLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double radius;
  double dist;
  double cost;
  double mark_x=0;
  double mark_y=0;
  double startcost=1;
  double maxcost=254;
  unsigned int mx;
  unsigned int my;
  double res=MyLayer::getResolution();
  geometry_msgs::Point32 pt;
  geometry_msgs::Point32 tpt;

	for(int k=0; k<linepoints.points.size();k++){
		std::string cloudtopic=linepoints.header.frame_id.c_str();
		pt.x=linepoints.points.at(k).x;
		pt.y=linepoints.points.at(k).y;
		tpt=TransformPoint(pt, "map",cloudtopic);

		  radius=linepoints.channels.at(0).values.at(k);
		  maxcost=linepoints.channels.at(1).values.at(k);
		  mark_x=tpt.x;
		  mark_y=tpt.y;

		  //reducing computation load by filtering points that aren't further away than one map resolution
		  if(res<sqrt(pow(oldx-mark_x,2)+pow(oldy-mark_y,2))){
			  oldx=mark_x;
			  oldy=mark_y;
		  for(double j=mark_y-radius;j<=mark_y+radius;j)
		  {
			  for(double i=mark_x-radius;i<=mark_x+radius;i)
			  {
				  dist=sqrt(pow(mark_x-i,2)+pow(mark_y-j,2));
				  if(dist<radius)
				  {
					  if(worldToMap(i, j, mx, my))
					  {

						  //cost=255*exp((log(50/255)*(dist/radius)));
						  cost=-dist*((maxcost-startcost)/radius)+maxcost;
//						  ROS_INFO("%d",MyLayer::getCost(mx,my));

						  if(MyLayer::getCost(mx, my)==255||MyLayer::getCost(mx, my)<cost){
						  	setCost(mx, my, (unsigned char) cost);
						  }



					  }
				  }

				  *min_x = std::min(*min_x, i);
				  *min_y = std::min(*min_y, j);
				  *max_x = std::max(*max_x, i);
				  *max_y = std::max(*max_y, j);
				  i+=res;
			  }
			  j+=res;
		  }
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

