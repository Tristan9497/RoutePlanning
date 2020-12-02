#include <ros/ros.h>


//TF
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>



#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
//custom message types
#include "road_detection/LineMatch.h"
#include "road_detection/Line.h"
#include "road_detection/LineArray.h"
#include "road_detection/Road.h"

#include <tf2/LinearMath/Quaternion.h>

#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/posefinderConfig.h>



double searchradius=5;
double searchangle=60*M_PI/180;
double costthreshold=100;
bool Rightside;
ros::Publisher pub;

std::vector<geometry_msgs::Point32> scan;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::Pose currentpose;
void publish_pose(road_detection::LineConstPtr& Lane);

void callback(arlo_navigation::posefinderConfig &config, uint32_t level) {
	Rightside=config.Side;
	ROS_INFO("%d",Rightside);
	searchradius=config.Searchradius;
	searchangle=config.Searchangle;
	costthreshold=config.Costthreshold;
}


class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{
			double x;
			double y;
			double range;
			double angle;
			int j;
			int size;
			tf2::Quaternion myQuaternion;
			geometry_msgs::PoseStamped newpose;


			if(Rightside)
			{
				j=road->laneRight.points.size()-1;
				angle=atan((road->laneRight.points.at(j).y-road->laneRight.points.at(j-1).y)/(road->laneRight.points.at(j).x-road->laneRight.points.at(j-1).x));
				newpose.pose.position.x=road->laneRight.points.at(j).x;
				newpose.pose.position.y=road->laneRight.points.at(j).y;
				size=road->laneRight.points.size();
			}
			else
			{
				j=road->laneLeft.points.size()-1;
				angle=atan((road->laneLeft.points.at(j).y-road->laneLeft.points.at(j-1).y)/(road->laneLeft.points.at(j).x-road->laneLeft.points.at(j-1).x));
				newpose.pose.position.x=road->laneLeft.points.at(j).x;
				newpose.pose.position.y=road->laneLeft.points.at(j).y;
				size=road->laneLeft.points.size();
			}

			if(size>0){

				newpose.header.seq = 1;
				newpose.header.frame_id="base_footprint";
				newpose.header.stamp = ros::Time::now();
				newpose.pose.position.z=0;
				myQuaternion.setRPY( 0, 0, angle );
				newpose.pose.orientation.x=myQuaternion.getX();
				newpose.pose.orientation.y=myQuaternion.getY();
				newpose.pose.orientation.z=myQuaternion.getZ();
				newpose.pose.orientation.w=myQuaternion.getW();
				pub.publish(newpose);
			}


		};
	public:
		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			geometry_msgs::Point32 scanpoint;
			scan.clear();
			double range,angle,minangle,maxangle;
			minangle=M_PI-(searchangle/2);
			maxangle=M_PI+(searchangle/2);
			for(int i=0;i<Scan->ranges.size();i++)
				{
					range=Scan->ranges.at(i);
					angle=(i*Scan->angle_increment)+Scan->angle_min;

					if((range<searchradius)&&(angle>minangle)&&(angle<maxangle)){

						//offset value from arlo_2stack urdf

						scanpoint.x=range*cos(angle)+0.175;
						scanpoint.y=range*sin(angle);
						scanpoint.z=0.0;

						scan.push_back(scanpoint);
					}

				}
		};


	};



int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
	ros::init(argc, argv, "posefinder");
	ros::NodeHandle n;
	Listener listener;

	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig> server;
	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig>::CallbackType f;
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scan = n.subscribe("/base_scan", 1000, &Listener::scanCallback, &listener);
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::Rate r=0.5;

	while(ros::ok()){

		pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
		//TODO maybe 3 different point clouds for each line of the track

		ros::spinOnce();
		r.sleep();

	}
	return 0;
}

