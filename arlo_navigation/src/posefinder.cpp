#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>

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
#include "visualization_msgs/Marker.h"

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

using Eigen::MatrixXd;
using Eigen::Vector2d;

double searchradius=5;
double searchangle=60*M_PI/180;
double costthreshold=100;
bool Rightside;
bool goaltrigger=false;
bool removeblockage=false;
double roadangle;
double robot_diameter=0.4;
double furthestpoint;

struct CIRCLE
	{
	double x;
	double y;
	double r;
	ros::Time Stamp;
	};
move_base_msgs::MoveBaseGoal goal;
ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
std::vector<geometry_msgs::PointStamped> scan;
geometry_msgs::Pose currentpose;
visualization_msgs::Marker marker;
//storing the latest road data for data loss cases
road_detection::Line LeftLine;
road_detection::Line RightLine;
road_detection::Line MiddleLine;
road_detection::Line LeftLane;
road_detection::Line RightLane;

std::string scanframe;
std::string roadframe;
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
void circleintersect(struct CIRCLE c1,struct CIRCLE c2, double searchrad)
	{
		// evaluating the circles and finding a point
		//since we are only determining it in the base_footprint frame we dont need to think about the robot position
		//Based on Paul Bourke Circle Spere Paper
		double a,d,h;
		geometry_msgs::Point32 p2,p31,p32,p41,p42,pres;

		//intersect with left circle
		d=sqrt(pow(c1.x,2)+pow(c1.y,2));
		a=(pow(searchrad,2)-pow(c1.r,2)+pow(d,2))/2*d;
		h=sqrt(pow(c1.r,2)-pow(a,2));

		p2.x=c1.x*a/d;
		p2.y=c1.y*a/d;
		p31.x=p2.x+(h*c1.y)/d;
		p31.y=p2.y-(h*c1.x)/d;
		p32.x=p2.x-(h*c1.y)/d;
		p32.y=p2.y+(h*c1.x)/d;



		//intersect with right circle
		d=sqrt(pow(c2.x,2)+pow(c2.y,2));
		a=(pow(searchrad,2)-pow(c2.r,2)+pow(d,2))/2*d;
		h=sqrt(pow(c2.r,2)-pow(a,2));

		p2.x=c2.x*a/d;
		p2.y=c2.y*a/d;
		p41.x=p2.x+(h*c2.y)/d;
		p41.y=p2.y-(h*c2.x)/d;
		p42.x=p2.x-(h*c2.y)/d;
		p42.y=p2.y+(h*c2.x)/d;
		//check which intersect is closer to the current trajectory of the robot
		if (atan2(p31.y,p31.x)>atan2(p32.y,p32.x)) p31=p32;
		//since this
		if (atan2(p41.y,p41.x)<atan2(p42.y,p42.x)) p41=p42;
	}
struct CIRCLE lastsquarecircle(road_detection::Line Line)

	{
		//least square circle approximation as discribed by Randy Bullock in his Least-Squares Circle Fit Paper
		//takes a Point32 array and calculates the best fitting circle in x,y dimensions, returns the circle parameters xc,yc,r
		//used to predict the upcoming road that cannot be seen by the camera
		struct CIRCLE circle;
		MatrixXd m(2,2);
		Vector2d b;
		Vector2d x;
		//define size so we don't have to do this all the time
		int size=Line.points.size();
		//defining sums
		double Suuu,Svvv,Suvv,Svuu,Suv,Suu,Svv=0;

		//defining averages and parameters u,v that are the point coordinates relative to the average
		double u,v,xa,ya=0;
		//first the averages will be build
		if(size>0){
			for (int i=0;i<size;i++)
			{
				xa+=Line.points.at(i).x;
				ya+=Line.points.at(i).y;
			}
			xa=xa/size;
			ya=ya/size;

			//new the Sums need to be calculated
			for (int i=0;i<size;i++)
			{
				u=Line.points.at(i).x-xa;
				v=Line.points.at(i).y-ya;

				Suu+=u*u;
				Svv+=v*v;
				Suv+=u*v;
				Suvv+=u*v*v;
				Svuu+=v*u*u;
				Suuu+=u*u*u;
				Svvv+=v*v*v;
			}
			m(0,0)=Suu;
			m(0,1)=Suv;
			m(1,0)=Suv;
			m(1,1)=Svv;

			m=m.inverse();

			b(0)=0.5*(Suuu+Suvv);
			b(1)=0.5*(Svvv+Svuu);

			//multiplying b with the inverse of m to solve the linear equation system since m*x=b so x=m⁻¹*b
			x=m*b;

			//calculating radius
			circle.r=sqrt(pow(x(0),2)+pow(x(1),2)+(Suu+Svv)/size);

			//translating into original coordinates
			circle.x=x(0)+xa;
			circle.y=x(1)+ya;

		}
		circle.Stamp=Line.header.stamp;
		return circle;

	}

void checkroadblockage(void)
	{
		ros::Duration Timedif;
		geometry_msgs::TransformStamped transformStamped;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener(tfBuffer);

		bool obstacleright=false;
		bool obstacleleft=false;
		//furthestpoint=sqrt(pow(LeftLine.points.back().x,2)+pow(LeftLine.points.back().y,2));

		if(scan.size()>0)
		{
			Timedif=ros::Time::now()-scan.at(0).header.stamp;
		}
		else {
			//ROS_INFO("Road Free");
			removeblockage=false;
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
						transformStamped = tfBuffer.lookupTransform( roadframe,Test.header.frame_id,Test.header.stamp,ros::Duration(1));

						tf2::doTransform(Test,Test, transformStamped);
						//pub.publish(Test);
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
				removeblockage=false;
			}
			else if(!obstacleleft&&obstacleright)
			{
				ROS_INFO("RightLane Blocked");
				removeblockage=true;
			}
			else if(!obstacleleft&&!obstacleright)
			{
				ROS_INFO("Road Free");
				removeblockage=false;
			}
		}
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
void constructgoalbyprediction(struct CIRCLE c1,struct CIRCLE c2)
	{
	//angles to robot origin


		tf2::Quaternion myQuaternion;
		double a1=atan2(-c1.y,-c1.x);
		double a2=atan2(-c2.y,-c2.x);
		double x,y,z,c1x,c2x,c1y,c2y,dist,m,m1,m2=0;
		geometry_msgs::PointStamped Test;
		Test.header.frame_id="base_footprint";
		Test.header.stamp=ros::Time::now();
		if(a1>0) a1-=M_PI/5;
		else a1+=M_PI/5;
		if(a2>0) a2-=M_PI/5;
		else a2+=M_PI/5;
		c1x=c1.x+c1.r*cos(a1);
		c1y=c1.y+c1.r*sin(a1);
		c2x=c2.x+c2.r*cos(a2);
		c2y=c2.y+c2.r*sin(a2);
		//find the point at three quaters so its in the middle of the right lane
		x=(c1x+3*c2x)/4;
		y=(c1y+3*c2y)/4;
		dist=sqrt(x*x+y*y);
		if(dist>4){
			x=4*x/dist;
			y=4*y/dist;
		}

		//Gradient of the Goal must be calculated using the derivative of a circle
		//since we weighted the position this gets weighted equaly
		if(y>0){
			//left curve
			m1=(c1x)/sqrt(c1.r*c1.r-c1x*c1x);
			m2=(c2x)/sqrt(c2.r*c2.r-c2x*c2x);
		}
		else
		{
			//right curve
			m1=-(c1x)/sqrt(c1.r*c1.r-c1x*c1x);
			m2=-(c2x)/sqrt(c2.r*c2.r-c2x*c2x);
		}
		m=(m1+3*m2)/4;
		//ROS_INFO("%f",m);
		Test.point.x=x;
		Test.point.y=y;
		Test.point.z=0;
		goaltrigger=true;
		goal.target_pose.header.frame_id="base_footprint";
		goal.target_pose.header.stamp = c1.Stamp;
		goal.target_pose.pose.position.x=x;
		goal.target_pose.pose.position.y=y;
		goal.target_pose.pose.position.z=0;
		myQuaternion.setRPY( 0, 0, atan(m) );
		myQuaternion=myQuaternion.normalized();
		goal.target_pose.pose.orientation.x=myQuaternion.getX();
		goal.target_pose.pose.orientation.y=myQuaternion.getY();
		goal.target_pose.pose.orientation.z=myQuaternion.getZ();
		goal.target_pose.pose.orientation.w=myQuaternion.getW();
		//TODO fix goal angle and tf transform for circle sinto the past




	}
void constructgoal(road_detection::Line Line)
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

			//DataSync so each Line has always the newest data
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
			searchradius=road->laneWidthLeft+ 0.6*road->laneWidthRight;
			//roadangle=aproxroadangle();
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

	////For debugging only
	marker_pub = n.advertise<visualization_msgs::Marker>("Circle", 1);
	marker_pub2 = n.advertise<visualization_msgs::Marker>("Circle2", 1);




	/////


	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::Rate rate=0.5;

	//parameters for both circles
	struct CIRCLE leftcircle,rightcircle;
	while(n.ok()){
		//enabling or disabling the blockage of the left lane
		checkroadblockage();
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


		//predicting the future trace of the lanes with approximated circles
		leftcircle=lastsquarecircle(LeftLine);
		rightcircle=lastsquarecircle(RightLine);
		constructgoalbyprediction(leftcircle,rightcircle);
		/////////only for dubugging
		//publishing estimated circle
		marker.header.frame_id = "base_footprint";
		marker.header.stamp = leftcircle.Stamp;
		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x=leftcircle.x;
		marker.pose.position.y=leftcircle.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x=leftcircle.r*2;
		marker.scale.y=leftcircle.r*2;
		marker.scale.z=1;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 0.5;
		marker.lifetime = ros::Duration();
		//ROS_INFO("x: %f, y: %f",x(0),x(1));
		///////
		marker_pub.publish(marker);
		marker.header.stamp = rightcircle.Stamp;
		marker.pose.position.x=(3*rightcircle.x+leftcircle.x)/4;
		marker.pose.position.y=(3*rightcircle.y+leftcircle.y)/4;
		marker.scale.x=(3*rightcircle.r+leftcircle.r)/2;
		marker.scale.y=(3*rightcircle.r+leftcircle.r)/2;
		marker_pub2.publish(marker);

		//intersecting both circles and finding the point in the middle
		//circleintersect(leftcircle,rightcircle,searchradius);


		//sending goal to move_base via actionserver
		//ac.waitForResult(ros::Duration(5));
		if(goaltrigger)
		{
			ac.sendGoal(goal);


			ROS_INFO("goal send");

		}
		else
		{
			ac.cancelAllGoals();
		}

    //TODO maybe 3 different point clouds for each line of the track
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}

