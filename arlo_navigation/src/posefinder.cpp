#include <ros/ros.h>

#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
//custom message types
#include "road_detection/LineMatch.h"
#include "road_detection/Line.h"
#include "road_detection/LineArray.h"
#include "road_detection/Road.h"

#include <tf2/LinearMath/Quaternion.h>

#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/posefinderConfig.h>

double searchradius=5;
bool Rightside;
ros::Publisher pub;
road_detection::Line Lane;

void callback(arlo_navigation::posefinderConfig &config, uint32_t level) {
	Rightside=config.Side;
	ROS_INFO("Side",Rightside);
}


class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{

			double mindist;
			double x;
			double y;
			double range;
			mindist=searchradius;
			int j;
			tf2::Quaternion myQuaternion;
			geometry_msgs::PoseStamped newpose;
			//publishing borders and middle line individual to give better flexibility
			if(Rightside){
				Lane=road->laneRight;
			}
			else
			{
				Lane=road->laneLeft;
			}
			if(Lane.points.size()>0){
				j=Lane.points.size()-1;

				double angle=atan((Lane.points.at(j).y-Lane.points.at(j-1).y)/(Lane.points.at(j).x-Lane.points.at(j-1).x));
				newpose.header.seq = 1;
				newpose.header.frame_id="base_footprint";
				newpose.header.stamp = ros::Time::now();
				newpose.pose.position.x=Lane.points.at(j).x;
				newpose.pose.position.y=Lane.points.at(j).y;
				newpose.pose.position.z=0;
				myQuaternion.setRPY( 0, 0, angle );
				newpose.pose.orientation.x=myQuaternion.getX();
				newpose.pose.orientation.y=myQuaternion.getY();
				newpose.pose.orientation.z=myQuaternion.getZ();
				newpose.pose.orientation.w=myQuaternion.getW();
				pub.publish(newpose);
			}

		};
	};

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
	ros::init(argc, argv, "posefinder");
	ros::NodeHandle n;


	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig> server;
	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);


	Listener listener;
	ros::Rate r(0.5);
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	while(ros::ok()){
		pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
		//TODO maybe 3 different point clouds for each line of the track

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
