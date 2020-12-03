#include <ros/ros.h>

//TF
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
bool goaltrigger=false;
double roadangle;
move_base_msgs::MoveBaseGoal goal;
ros::Publisher pub;

std::vector<geometry_msgs::Point32> scan;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::Pose currentpose;

//storing the latest road data for data loss cases
road_detection::Line LeftLine;
road_detection::Line RightLine;
road_detection::Line MiddleLine;
road_detection::Line LeftLane;
road_detection::Line RightLane;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//should be reference???
double polynomial(double x, road_detection::Line::_polynomial_type::_a_type a) {
	double xp = 1;
	double result = 0;

	for ( int i = 0; i < a.size(); i++ ) {
		result += xp * a[i];
		xp *= x;
	}
	return result;
}
double aproxroadangle(void)
	{
	//approximate the road width based on the angle of the road relative to the robots direction
	//TODO check reliability
		double dx=	((LeftLine.points.at(1).x-LeftLine.points.at(0).x)+
					(RightLine.points.at(1).x-RightLine.points.at(0).x)+
					(MiddleLine.points.at(1).x-MiddleLine.points.at(0).x)+
					(LeftLane.points.at(1).x-LeftLane.points.at(0).x)+
					(RightLane.points.at(1).x-RightLane.points.at(0).x))/5;
		double dy=	((LeftLine.points.at(1).y-LeftLine.points.at(0).y)+
					(RightLine.points.at(1).y-RightLine.points.at(0).y)+
					(MiddleLine.points.at(1).y-MiddleLine.points.at(0).y)+
					(LeftLane.points.at(1).y-LeftLane.points.at(0).y)+
					(RightLane.points.at(1).y-RightLane.points.at(0).y))/5;

		return atan(dy/dx);
	}

void callback(arlo_navigation::posefinderConfig &config, uint32_t level) {
	Rightside=config.Side;
	searchradius=config.Searchradius;
	searchangle=config.Searchangle;
	costthreshold=config.Costthreshold;
}
void constructgoal(road_detection::Line &Line)
	{
		tf2::Quaternion myQuaternion;
		int j,size;
		float angle,x,y;
		goaltrigger=true;
		j=Line.points.size()-1;
		angle=atan((Line.points.at(j).y-Line.points.at(j-1).y)/(Line.points.at(j).x-Line.points.at(j-1).x));
		x=Line.points.at(j).x;
		y=Line.points.at(j).y;
		size=Line.points.size();
		ROS_INFO("x:%f,y:%f",x,y);
		goal.target_pose.header.frame_id="base_footprint";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x=x;
		goal.target_pose.pose.position.y=y;
		goal.target_pose.pose.position.z=0;
		myQuaternion.setRPY( 0, 0, angle );
		myQuaternion=myQuaternion.normalized();
		goal.target_pose.pose.orientation.x=myQuaternion.getX();
		goal.target_pose.pose.orientation.y=myQuaternion.getY();
		goal.target_pose.pose.orientation.z=myQuaternion.getZ();
		goal.target_pose.pose.orientation.w=myQuaternion.getW();
	}

class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{
			bool obstacleright=false;
			bool obstacleleft=false;

			//DataSync
			if(road->laneLeft.points.size()>0)
			{
				LeftLane=road->laneLeft;
			}
			if(road->laneRight.points.size()>0)
			{
				RightLane=road->laneRight;
			}

			if(road->lineLeft.points.size()>0)
			{
				LeftLine=road->lineLeft;
			}
			if(road->lineRight.points.size()>0)
			{
				RightLine=road->lineRight;
			}
			if(road->lineMiddle.points.size()>0)
			{
				MiddleLine=road->lineMiddle;
			}
			roadangle=aproxroadangle();

			road_detection::Line Line;
			for(int i=0; i<scan.size();i++){
				//check whether the point is in the lane by projecting it on to the 3 roadlines and comparing their y values
				//exact calculation not necessary since road makes wide curves
				if(polynomial(scan.at(i).x, LeftLine.polynomial.a)>scan.at(i).y&&polynomial(scan.at(i).x, MiddleLine.polynomial.a)<scan.at(i).y)
				{
					obstacleleft=true;
				}
				else if(polynomial(scan.at(i).x, RightLine.polynomial.a)<scan.at(i).y&&polynomial(scan.at(i).x, MiddleLine.polynomial.a)>scan.at(i).y)
				{
					obstacleright=true;
				}

			}


			if(obstacleleft&&!obstacleright)
			{
				ROS_INFO("LeftLane Blocked");
				Line=RightLane;
				constructgoal(Line);
			}
			else if(obstacleleft&&obstacleright)
			{
				ROS_INFO("Road Blocked");
				goaltrigger=false;
				obstacleleft=false;
				obstacleright=false;
			}
			else if((!obstacleleft&&obstacleright)||!Rightside)
			{
				ROS_INFO("RightLane Blocked");
				Line=LeftLane;
				constructgoal(Line);
			}

			else if(!obstacleright||!obstacleleft){
				ROS_INFO("Road Free");
				Line=RightLane;
				constructgoal(Line);
			}
		};
	public:
		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			geometry_msgs::Point32 scanpoint;
			scan.clear();
			double range,angle,minangle,maxangle;
			minangle=M_PI-(searchangle/2)+roadangle;
			maxangle=M_PI+(searchangle/2)+roadangle;
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
	MoveBaseClient ac("move_base", true);

	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig> server;
	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig>::CallbackType f;
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scan = n.subscribe("/base_scan", 1000, &Listener::scanCallback, &listener);
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);


	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::Rate r=0.5;

	while(ros::ok()){
		if(goaltrigger)
		{
			ac.sendGoal(goal);
			ROS_INFO("goal send");

		}
		else
		{
			ac.cancelAllGoals();
		}
//					ac.waitForResult();
//					if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//						ROS_INFO("Hooray, the base moved 1 meter forward");
//					else
//						ROS_INFO("The base failed to move forward 1 meter for some reason");

		//TODO maybe 3 different point clouds for each line of the track

		ros::spinOnce();
		r.sleep();

	}
	return 0;
}

