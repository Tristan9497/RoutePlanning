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
  //necessary since static and rolling can't be treated the same
  rolling_window_ = layered_costmap_->isRolling();
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<simple_layers::ProgressiveLayerConfig>(nh);
  dynamic_reconfigure::Server<simple_layers::ProgressiveLayerConfig>::CallbackType cb = boost::bind(
      &MyLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  //define subcribers and services
  line_sub_ = nh.subscribe(topic, 1, &MyLayer::lineCallback, this);
  service=nh.advertiseService("MyLayer/reset", &MyLayer::reset, this);

  //
  nh.getParam("/enabled", enabled_);
  //get the parameters of the layer
  if(!enabled_)
  {
	  ROS_WARN("Layer: %s is inactive",name_.c_str());
  }


  //if we are in the simulation we need to set sim_time to true else we want to use real time
  if(nh.getParam("/sim_time",sim_time))
  {
	  ros::param::set("/use_sim_time",true);
  }
  layertrigger=true;

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

//function that transforms a point32 from frame2 to the global frame of the costmap and returns the transformed point
geometry_msgs::Point32 MyLayer::TransformPoint(geometry_msgs::Point32 point,std::string frame2)
	{
	geometry_msgs::TransformStamped transformStamped;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	geometry_msgs::Point pt;
	geometry_msgs::Point32 pt32;

	pt.x=point.x;
	pt.y=point.y;
	pt.z=point.z;

    try {
        transformStamped = tfBuffer.lookupTransform( layered_costmap_->getGlobalFrameID(),frame2,ros::Time(0),ros::Duration(1));

        tf2::doTransform(pt,pt, transformStamped);
    	pt32.x=pt.x;
    	pt32.y=pt.y;
    	pt32.z=pt.z;
    	return pt32;

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }


	}
void MyLayer::drawCost(int mx, int my,unsigned int xc,unsigned int yc, unsigned int rad, double maxcost, double startcost)
	{
		//draw a line from mx to -mx at my with the correct cost
		double cost,dist,dx,dy,res,radius;

		int px,py;
		res=MyLayer::resolution_;
		//ROS_INFO("%f",maxcost);
		//setCost(xc+mx,yc+my,250);

		//iterating over every cell in the row at my that is part of the first octant
		for (int i=my;i<mx;i++){
				//if maxcost is zero we want to clear something in the layer
				// in this case cost calculation is pointless and we set all cells to 255 or NO_information
				if(maxcost==0)
				{
					setCost(xc+i,yc+my,NO_INFORMATION);
					setCost(xc-i,yc+my,NO_INFORMATION);
					setCost(xc+i,yc-my,NO_INFORMATION);
					setCost(xc-i,yc-my,NO_INFORMATION);
					setCost(xc+my,yc+i,NO_INFORMATION);
					setCost(xc-my,yc+i,NO_INFORMATION);
					setCost(xc+my,yc-i,NO_INFORMATION);
					setCost(xc-my,yc-i,NO_INFORMATION);
				}
				//calculating the cost for the distance of the cell and set all 8 cell at the same time
				else
				{
					dx=i*res;
					dy=my*res;
					radius=rad*res;
					dist=sqrt(dx*dx+dy*dy);


					//linear decaying cost distribution
					cost=maxcost-dist*((maxcost-startcost)/radius);

					//projecting the cell into all other octants so we reduce computation be roughly 7/8
					if(MyLayer::getCost(xc+i,yc+ my)==NO_INFORMATION||MyLayer::getCost(xc+i,yc+ my)<cost){
					  setCost(xc+i,yc+ my, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc-i,yc+ my)==NO_INFORMATION||MyLayer::getCost(xc-i,yc+ my)<cost){
					  setCost(xc-i,yc+ my, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc+i,yc- my)==NO_INFORMATION||MyLayer::getCost(xc+i,yc- my)<cost){
					  setCost(xc+i,yc- my, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc-i,yc- my)==NO_INFORMATION||MyLayer::getCost(xc-i,yc- my)<cost){
					  setCost(xc-i,yc- my, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc+ my,yc+i)==NO_INFORMATION||MyLayer::getCost(xc+ my,yc+i)<cost){
					  setCost(xc+ my,yc+i, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc+ my,yc-i)==NO_INFORMATION||MyLayer::getCost(xc+ my,yc-i)<cost){
					  setCost(xc+ my,yc-i, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc- my,yc+i)==NO_INFORMATION||MyLayer::getCost(xc- my,yc+i)<cost){
					  setCost(xc- my,yc+i, (unsigned char) cost);
					}
					if(MyLayer::getCost(xc- my,yc-i)==NO_INFORMATION||MyLayer::getCost(xc- my,yc-i)<cost){
					  setCost(xc- my,yc-i, (unsigned char) cost);
					}
				}
			}

	}

//This function transforms the incoming pointcloud and inflates the coordinate using bresenham based circle rasterization
//The incoming pointcloud should have channel values for radius, maxcost and startcost for every point
//This allows the layer to inflate every incoming point differently (usefull for clearing)
void MyLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
	// robot pose has to be updated, otherwise we get huge offset in case rolling window costmap is selected

  if (rolling_window_)
	updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  //return if the layer is in reset mode or if it hasn't been enabled

  if (!enabled_||!layertrigger)
    return;


  //ELSE INFLATE EVERY POINT IN THE CLOUD BY ITS INDIVIDUAL RADIUS AND COSTS
  double radius;
  //default values... true values come with the pointcloud as channel info
  double startcost;
  double maxcost;

  //parameters in pixel dimension needs to be unsigned so we prevent out of range errors
  unsigned int xc,yc,rad;

  int mx,my;//coordinates of current pixel

  int error,dx,dy;//parameters of bresenham algorithm

  double res=MyLayer::getResolution();

  geometry_msgs::Point32 pt;
  geometry_msgs::Point32 tpt;


  if(linepoints.points.size()>0){


	  for(int k=0; k<linepoints.points.size();k++){
		std::string cloudtopic=linepoints.header.frame_id.c_str();
		pt.x=linepoints.points.at(k).x;
		pt.y=linepoints.points.at(k).y;
		tpt=TransformPoint(pt,cloudtopic);

		radius=linepoints.channels.at(0).values.at(k);
		maxcost=linepoints.channels.at(1).values.at(k);
		startcost=linepoints.channels.at(2).values.at(k);

		//converting to pixel dimension
		rad=(unsigned int)(radius/res);
		worldToMap(tpt.x,tpt.y,xc,yc);

//		mark_x=tpt.x;
//		mark_y=tpt.y;


		mx=rad;
		my=0;
		error=rad;
		//draw first line
		drawCost(mx,my,xc,yc,rad,maxcost,startcost);
		while(my<mx)
		{
			dy=my*2+1;
			my++;
			error-=dy;
			if(error<0)
			{
				dx=1-mx*2;
				mx--;
				error-=dx;
			}
			//drawing the lines for 1 quadrant
			drawCost(mx,my,xc,yc,rad,maxcost,startcost);
			//drawCost(my,mx,xc,yc,rad,maxcost,startcost);

			//updating bounds
			*min_x = std::min(*min_x, (xc-mx)*res);
			*min_y = std::min(*min_y, (yc-my)*res);
			*max_x = std::max(*max_x, (xc+mx)*res);
			*max_y = std::max(*max_y, (yc+my)*res);
		}

	  }
	linepoints.points.clear();
  }

}


//this service switches off the layer and resets its content depending on the trigger
//with trigger=true the layer will be reseted
//with false the default behavior will be started again

bool MyLayer::reset(simple_layers::reset::Request &trigger, simple_layers::reset::Response &state)
	{
	if(trigger.trigger==true)
	{
		layertrigger=false;
		  for (int j = 0;j<MyLayer::getSizeInCellsX(); j++)
		  {
			  for (int i = 0;i<MyLayer::getSizeInCellsX(); i++)
		    {
		      int index = getIndex(i, j);

		      	setCost(i, j, 0);

		    }
		  }

		  state.state=false;
	}
	else
	{
		layertrigger=true;

		state.state=true;
	}
	  return true;
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

