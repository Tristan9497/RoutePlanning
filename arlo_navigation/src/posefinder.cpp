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
struct LINE
	{
	double m;
	double b;
	ros::Time Stamp;
	};


ros::Publisher pub;
std::vector<geometry_msgs::PointStamped> scan;
geometry_msgs::Pose currentpose;

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

void callback(arlo_navigation::posefinderConfig &config, uint32_t level) {
	Rightside=config.Side;
	searchradius=config.Searchradius;
	searchangle=config.Searchangle;
	costthreshold=config.Costthreshold;
}


class Drawer
	{
	//class for the draw functions so the publishers don't need to be defined all the time but only for the debugging
	//keeps things more tidy
	private:
		ros::NodeHandle n;
		ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>("Lines", 1);
		ros::Publisher circle_pub1 = n.advertise<visualization_msgs::Marker>("Circle1", 1);
		ros::Publisher circle_pub2 = n.advertise<visualization_msgs::Marker>("Circle2", 1);
	////For debugging only
	public:

		void circles(struct CIRCLE c1,struct CIRCLE c2)
			{
			//function to visualize predicted circles
				visualization_msgs::Marker marker;
				marker.header.frame_id = "base_footprint";
				marker.header.stamp = c1.Stamp;
				marker.ns = "basic_shapes";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.scale.x=c1.r*2;
				marker.scale.y=c1.r*2;
				marker.scale.z=1;
				marker.pose.position.x=c1.x;
				marker.pose.position.y=c1.y;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 0.5;
				marker.lifetime = ros::Duration();
				circle_pub1.publish(marker);
				marker.header.stamp = c2.Stamp;
				marker.scale.x=c2.r*2;
				marker.scale.y=c2.r*2;
				marker.pose.position.x=c2.x;
				marker.pose.position.y=c2.y;
				circle_pub2.publish(marker);
		}
		void lines(struct LINE l1,struct LINE l2)
		{
		//function to visualize predicted lines
			geometry_msgs::Point p;
			visualization_msgs::Marker marker;
			marker.header.frame_id = "base_footprint";
			marker.header.stamp = l1.Stamp;
			marker.ns = "basic_shapes";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.action = visualization_msgs::Marker::ADD;
			marker.scale.x=0.1;
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 0.5;
			marker.lifetime = ros::Duration();
			//ROS_INFO("x: %f, y: %f",x(0),x(1));
			///////

			marker.points.clear();
			p.x=0;
			p.y=l1.b;
			marker.points.push_back(p);
			p.x=4;
			p.y=l1.m*4+l1.b;
			marker.points.push_back(p);
			p.x=0;
			p.y=l2.b;
			marker.points.push_back(p);
			p.x=4;
			p.y=l2.m*4+l2.b;
			marker.points.push_back(p);

			line_pub.publish(marker);
		};

	};
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
class GoalFinder
	{

private:
	int Finder=0;
	bool goaltrigger=true;//might be neccessary to handle error scenarios

	Drawer draw;
	//functions to approximate the future road based on least squared error calculations
	struct LINE leastsquareline(road_detection::Line Line)
		{
			//least square line aproximation https://www.mathsisfun.com/data/least-squares-regression.html
			struct LINE line;
			double Sx,Sy,Sxx,Sxy,N;
			N=Line.points.size();
			for(int i=0;i<N;i++)
			{
				Sx+=Line.points.at(i).x;
				Sy+=Line.points.at(i).y;
				Sxy+=Line.points.at(i).x*Line.points.at(i).y;
				Sxx+=Line.points.at(i).x*Line.points.at(i).x;
			}
			try{
				line.m=((N*Sxy)-(Sx*Sy))/((N*Sxx)-pow(Sx,2));
				line.b=(Sy-line.m*Sx)/N;
				line.Stamp = Line.header.stamp;
				return line;
			}
			catch(...)
			{
				//Division by zero occured caused by all points having the same x
				//or b the Input-Line not containing any point
				// In this case we can't return anything, but this shouldn't
				//ever occur in this application

				//C++ doesn't offer a division by zero exception thats why we are catching all of them
			}
		}
	struct CIRCLE leastsquarecircle(road_detection::Line Line)

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

	//goal constructors for different road approximations
	void constructgoalbycircles(struct CIRCLE c1,struct CIRCLE c2, double dist, double angle)
		{
			//angle is in radiant
			tf2::Quaternion myQuaternion;

			//polar angles from circle origin to position of robot
			//since we are in the base_footprint frame robot is at 0,0
			double a1=atan2(-c1.y,-c1.x);
			double a2=atan2(-c2.y,-c2.x);
			double x,y,z,c1x,c2x,c1y,c2y,d,m,m1,m2=0;


			//if the approximated circles are very large
			//the covered distance is bigger than the distance we want to plan ahead
			//so we need to check that and adjust the angle accordingly
			if(((c1.r+3*c2.r)/4*angle)>dist)angle=dist*4/(c1.r+3*c2.r);

			//calc polar angle of new point for both circles
			//calc orientation of goal
			if(c1.y>0)
			{
				a1+=angle;
				m1=a1+M_PI/2;
			}
			else
			{
			a1-=angle;
			m1=a1-M_PI/2;
			}
			if(c2.y>0)
			{
				a2+=angle;
				m2=a2+M_PI/2;
			}
			else
			{
			a2-=angle;
			m2=a2-M_PI/2;
			}

			//calc cartesian coordinates
			c1x=c1.x+c1.r*cos(a1);
			c1y=c1.y+c1.r*sin(a1);
			c2x=c2.x+c2.r*cos(a2);
			c2y=c2.y+c2.r*sin(a2);

			//find the point at three quaters so its in the middle of the right lane
			x=(c1x+3*c2x)/4;
			y=(c1y+3*c2y)/4;
			m=(m1+3*m2)/4;

			goaltrigger=true;
			goal.target_pose.header.frame_id="base_footprint";
			goal.target_pose.header.stamp = c1.Stamp;
			goal.target_pose.pose.position.x=x;
			goal.target_pose.pose.position.y=y;
			goal.target_pose.pose.position.z=0;
			myQuaternion.setRPY( 0, 0, m );
			myQuaternion=myQuaternion.normalized();
			goal.target_pose.pose.orientation.x=myQuaternion.getX();
			goal.target_pose.pose.orientation.y=myQuaternion.getY();
			goal.target_pose.pose.orientation.z=myQuaternion.getZ();
			goal.target_pose.pose.orientation.w=myQuaternion.getW();

		}
	void constructgoalbylines(struct LINE l1,struct LINE l2, double dist)
		{
			tf2::Quaternion myQuaternion;

			double l1x,l2x,l1y,l2y;
			double a1,a2,off1,off2;
			double x,y,m;


			//geometric calculations to find point on crooked line in the right distance
			//if robot is turned on the road roadline will become so slanted that calulating with x only
			//will be inaccurate
			a1=atan(l1.m);
			a2=atan(l2.m);
			off1=cos(M_PI/2-a1)*l1.b;
			off2=cos(M_PI/2-a2)*l2.b;
			if(l1.m<0)l1x=x=cos(a1)*(dist+off1);
			else l1x=x=cos(a1)*(dist-off1);
			if(l2.m<0)l2x=x=cos(a2)*(dist+off2);
			else l2x=x=cos(a2)*(dist-off2);
			l1y=l1x*l1.m+l1.b;
			l2y=l2x*l2.m+l2.b;

			//find goal at ratio between both points
			x=(l1x+3*l2x)/4;
			y=(l1y+3*l2y)/4;
			m=(l1.m+3*l2.m)/4;

			goal.target_pose.header.frame_id="base_footprint";
			goal.target_pose.header.stamp = l1.Stamp;
			goal.target_pose.pose.position.x=x;
			goal.target_pose.pose.position.y=y;
			goal.target_pose.pose.position.z=0;
			myQuaternion.setRPY( 0, 0, atan(m) );
			myQuaternion=myQuaternion.normalized();
			goal.target_pose.pose.orientation.x=myQuaternion.getX();
			goal.target_pose.pose.orientation.y=myQuaternion.getY();
			goal.target_pose.pose.orientation.z=myQuaternion.getZ();
			goal.target_pose.pose.orientation.w=myQuaternion.getW();

		}
	void sendgoal()
	{
		//sending goal to move_base via actionserver
		//ac.waitForResult(ros::Duration(5));
		if(goaltrigger)
		{
			ac.sendGoal(goal);

		}
		else
		{
			ac.cancelAllGoals();
		}
	}
public:
	//Tuneable parameters TODO dynamic recon
	double CRadThresh=8;//The Threshold radius of the approximated circles if the circles get larger line approx will be used
	double GoalDist=4;//The Distance, at which a goal should be found
	double GoalAngle=M_PI/3;//The angle, we think we can trust the circle approximation
	GoalFinder() : ac("move_base",true){}
	MoveBaseClient ac;


	move_base_msgs::MoveBaseGoal goal;
	struct CIRCLE leftcircle,rightcircle;
	struct LINE leftline, rightline;
	//goal manager used to determine which goal needs to be build
	void constructgoal()
			{
				leftcircle=leastsquarecircle(LeftLine);
				rightcircle=leastsquarecircle(RightLine);
				switch (Finder)
				{
					case 0:
						//Circle approximation
						if(LeftLine.points.size()>0&&LeftLine.points.size()>0){
							//switching to line approx since straight is coming up
							if(leftcircle.r>CRadThresh||rightcircle.r>CRadThresh)
							{
								Finder=1;
								break;
							}
							else
							{
								constructgoalbycircles(leftcircle, rightcircle, GoalDist, GoalAngle);
								sendgoal();
								draw.circles(leftcircle,rightcircle);
							}
						}
						break;

					case 1:
						//Line approximation
						if(LeftLine.points.size()>0&&LeftLine.points.size()>0){
							leftline=leastsquareline(LeftLine);
							rightline=leastsquareline(RightLine);
							//switching to circle approx since a corner is comming up
							if(leftcircle.r<CRadThresh&&rightcircle.r<CRadThresh)
							{
								Finder=0;
								break;
							}
							else
							{
								constructgoalbylines(leftline, rightline, GoalDist);
								sendgoal();
								draw.lines(leftline,rightline);
							}
						}
						break;

					case 2:
						//Goal from Costmap TODO
						if(LeftLine.points.size()>0&&LeftLine.points.size()>0){
						}
						break;
				}
			}
	};


int main(int argc, char* argv[])
{

	ros::init(argc, argv, "posefinder");


	ros::NodeHandle n;
	Listener listener;
	GoalFinder finder;


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

	//wait untill the actionserverclient in the goalfinder finds the movebase server
	while(!finder.ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ros::Rate rate=0.5;
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

		//requesting the goalfinder to construct a new goal
		finder.constructgoal();

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

