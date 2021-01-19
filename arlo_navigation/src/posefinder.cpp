#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

//TF
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
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
//Services
#include "arlo_navigation/clearleftlane.h"


double searchradius=5;
double searchangle=60*M_PI/180;
double costthreshold=100;
bool Rightside;
bool goaltrigger=false;
bool removeblockage=false;
double roadangle;
double robot_diameter=0.4;
double furthestpoint;

move_base_msgs::MoveBaseGoal goal;
ros::Publisher pub;
std::vector<geometry_msgs::PointStamped> scan;
geometry_msgs::Pose currentpose;

//storing the latest road data for data loss cases
road_detection::Line LeftLine;
road_detection::Line RightLine;
road_detection::Line MiddleLine;
road_detection::Line LeftLane;
road_detection::Line RightLane;
geometry_msgs::Point getPointTransform(std::string frame, ros::Time &Time,geometry_msgs::Point &Point);
std::string scanframe;
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

geometry_msgs::Point getPointTransform(std::string frame, ros::Time &Time)
	{

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transform;
	geometry_msgs::Point PointT;
	try{

	  transform = tfBuffer.lookupTransform(frame,frame,Time);
	  PointT.x=transform.transform.translation.x;
	  PointT.y=transform.transform.translation.y;
	  PointT.z=transform.transform.translation.z;
	  return PointT;
	}
	catch (tf2::TransformException &ex) {
	  ROS_WARN("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}


	}
class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{
			bool obstacleright=false;
			bool obstacleleft=false;
			float xobsleftmin=searchradius;
			float xobsleftmax;
			float xobsrightmin=searchradius;
			float xobsrightmax;
			geometry_msgs::Point Scan;
			geometry_msgs::Point LeftPoint;
			geometry_msgs::Point MiddlePoint;
			geometry_msgs::Point RightPoint;
			ros::Duration Timedif;
			geometry_msgs::TransformStamped transformStamped;
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener(tfBuffer);

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

			furthestpoint=sqrt(pow(LeftLine.points.back().x,2)+pow(LeftLine.points.back().y,2));
			searchradius=furthestpoint+robot_diameter;
			road_detection::Line Line;

			if(scan.size()>0)
			{
				Timedif=ros::Time::now()-scan.at(0).header.stamp;
			}
			else {
				ROS_INFO("Road Free");
				removeblockage=false;
				Line=RightLane;
				Timedif=ros::Duration(1);
			}

			//check if the scanned point is infront of the robot and if it is in a reasonable timeframe (not older thaen 500ms)
			if(Timedif.toSec()<=0.5){
				for(int i=0; i<scan.size();i++){
					if(scan.at(0).point.x>0){
						geometry_msgs::PointStamped Test;
						Test.header.frame_id=scan.at(i).header.frame_id;
						Test.header.stamp=scan.at(i).header.stamp;
						Test.point.x=scan.at(i).point.x;
						Test.point.y=scan.at(i).point.y;
						Test.point.z=scan.at(i).point.z;
						try{
							//transform scanned point into the frame of the road detection then compare it with the road
							transformStamped = tfBuffer.lookupTransform( road->header.frame_id,Test.header.frame_id,Test.header.stamp,ros::Duration(1));

							tf2::doTransform(Test,Test, transformStamped);
							pub.publish(Test);
							//check whether the point is in the lane by projecting it on to the 3 roadlines and comparing their y values
							//exact calculation not necessary since road makes wide curves
							if(polynomial(Test.point.x, LeftLine.polynomial.a)>Test.point.y && polynomial(Test.point.x, MiddleLine.polynomial.a)<Test.point.y)
							{
								obstacleleft=true;
							}
							else if(polynomial(Test.point.x, RightLine.polynomial.a)<Test.point.y && polynomial(Test.point.x, MiddleLine.polynomial.a)>Test.point.y)
							{
								obstacleright=true;
							}
						}
						catch(tf2::TransformException &ex)
						{
							ROS_WARN("%s", ex.what());
						}
					}
				}
				if(obstacleleft&&!obstacleright)
				{
					ROS_INFO("LeftLane Blocked");
					Line=RightLane;
					removeblockage=false;
	//				constructgoal(Line);
				}

				else if(!obstacleleft&&obstacleright)
				{
					ROS_INFO("RightLane Blocked");
					Line=LeftLane;
					removeblockage=true;
	//				constructgoal(Line);
				}
				else if(!obstacleleft&&!obstacleright)
				{
					ROS_INFO("Road Free");
					removeblockage=false;
					Line=RightLane;
				}
		}


    };
	public:
		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			geometry_msgs::PointStamped scanpoint;
			scan.clear();
			double range,angle,minangle,maxangle;
			minangle=M_PI-(searchangle/2)+roadangle;
			maxangle=M_PI+(searchangle/2)+roadangle;
			for(int i=0;i<Scan->ranges.size();i++)
				{
					range=Scan->ranges.at(i);
					angle=(i*Scan->angle_increment)+Scan->angle_min;
					if((range<searchradius)&&(angle>minangle)&&(angle<maxangle)){

						scanpoint.header.frame_id=Scan->header.frame_id.c_str();
						scanpoint.header.stamp=Scan->header.stamp;
						scanpoint.point.x=range*cos(angle);
						scanpoint.point.y=range*sin(angle);
						scanpoint.point.z=0.0;

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
	pub=n.advertise<geometry_msgs::PointStamped>("TestPoint", 1000);
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scan = n.subscribe("/scan_filtered", 1000, &Listener::scanCallback, &listener);
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	//service
	ros::ServiceClient client = n.serviceClient<arlo_navigation::clearleftlane>("clearblockage");
	arlo_navigation::clearleftlaneRequest req;
	arlo_navigation::clearleftlaneResponse res;




	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::Rate r=1;

	while(n.ok()){

		//switching left lane on and off
		if(removeblockage)
		{
			req.trigger=false;
			client.call(req,res);
		}
		else
		{
			req.trigger=true;
			client.call(req,res);
		}

//		ac.waitForResult(ros::Duration(5));
//		if(goaltrigger)
//		{
//			ac.sendGoal(goal);
//
//
//			ROS_INFO("goal send");
//
//		}
//		else
//		{
//			ac.cancelAllGoals();
//		}

    //TODO maybe 3 different point clouds for each line of the track
		ros::spinOnce();
		r.sleep();

	}
	return 0;
}

