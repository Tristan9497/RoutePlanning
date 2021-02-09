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
#include <actionlib/client/terminal_state.h>

#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
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
#include "arlo_navigation/goalnotfound.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;

//Structs
struct APoint
{
	double x;	//average x
	double y;	//average y
	int N;		//amount of points included in the average
};
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
struct map_inf {

    double size_x;
    double size_y;
    double scale;
    double origin_x;
    double origin_y;
};

//definitions to help with map coordinates and indices
#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
//get coordinates relative to /map frame from cell coordinates
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)
//get cell coordinates from coordinates relative to /map
#define MAP_CX(map,i) (i-round(map.origin_x/map.scale)+floor(map.size_x/2))
#define MAP_CY(map,j) (j-round(map.origin_y/map.scale)+floor(map.size_y/2))



ros::Publisher pub;
std::vector<geometry_msgs::PointStamped> scan;
geometry_msgs::PoseStamped robotorigin;
//storing the latest road data for data loss cases
road_detection::Line LeftLine;
road_detection::Line RightLine;
road_detection::Line MiddleLine;
road_detection::Line LeftLane;
road_detection::Line RightLane;
nav_msgs::OccupancyGrid Map;
std::string roadframe;

double CRadThresh=8;//The Threshold radius of the approximated circles if the circles get larger line approx will be used
double GoalDist=4;//The Distance, at which a goal should be found
double GoalAngle=M_PI/3;//The angle, we think we can trust the circle approximation
double reductionradius=1.5;
double MapCostThresh=50;//probability of occupancy in percent



double searchradius=5;
double searchangle=60*M_PI/180;
double costthreshold=100;
bool Rightside;

bool removeblockage=false;
double roadangle;
double robot_diameter=0.4;

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
	CRadThresh=config.CRadThresh;
	GoalDist=config.GoalDist;
	GoalAngle=config.GoalAngle*M_PI;
	reductionradius=config.reductionradius;
	MapCostThresh=config.MapCostThresh;
}

class Drawer
	{
	//class for the draw functions so the publishers don't need to be defined all the time but only for the debugging
	//keeps things more tidy
	private:
		ros::NodeHandle n;
		ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>("Lines", 1);
		ros::Publisher mappoint_pub = n.advertise<geometry_msgs::PointStamped>("mappoints", 1);
		ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloudpoints", 1);
		ros::Publisher circle_pub1 = n.advertise<visualization_msgs::Marker>("Circle1", 1);
		ros::Publisher circle_pub2 = n.advertise<visualization_msgs::Marker>("Circle2", 1);
	////For debugging only
	public:
		void cloud(sensor_msgs::PointCloud cloud)
		{
			cloud_pub.publish(cloud);
		}
		void mappoints(geometry_msgs::PointStamped p)
		{
			mappoint_pub.publish(p);
		}
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
		void mapCallback(const nav_msgs::OccupancyGridConstPtr &map)
		{

			Map.data=map->data;//update global Map variable
			Map.header=map->header;
			Map.info=map->info;
		}
		void odomCallback(nav_msgs::Odometry odom)
		{
			//update global pose variable
			robotorigin.header=odom.header;
			robotorigin.pose=odom.pose.pose;

		}
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

	int Finder=0;//start with searching a goal on a circle
	bool goaltrigger=true;//might be neccessary to handle error scenarios
	sensor_msgs::PointCloud cloud;
	std::vector<geometry_msgs::PointStamped> goalpoints;


	MoveBaseClient ac;
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
	//void construcctor for costmap

	void ConstructPointCloud(double searchradius)
	{
		//check for not zero stamp so we only look for positions if we have received a map
		if(Map.header.stamp.isZero()==false){
			//this function renders a circle with sub-pixel accuracy around the circle at a given radius
			map_inf map_info;
			//collect all cost data in a circle of radius searchradius
			map_info.size_x = Map.info.width;
			map_info.size_y = Map.info.height;
			map_info.scale = Map.info.resolution;
			map_info.origin_x = Map.info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
			map_info.origin_y = Map.info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

			double gt_x = Map.info.origin.position.x;
			double gt_y = Map.info.origin.position.y;

			tf2_ros::Buffer tf_buffer;
			tf2_ros::TransformListener tf2_listener(tf_buffer);
			geometry_msgs::TransformStamped base_footprint_to_map;
			geometry_msgs::TransformStamped map_to_base_footprint;
			//tranform robot pose into map frame
			geometry_msgs::PoseStamped pt;
//			robotorigin.header.stamp =  Map.header.stamp;

//			try{
//				base_footprint_to_map=tf_buffer.lookupTransform("map",robotorigin.header.frame_id, ros::Time(0), ros::Duration(1.0) );
//
//				tf2::doTransform(robotorigin, robotorigin, base_footprint_to_map);
//
//
//			}
//			catch (tf2::TransformException &ex){
//				ROS_ERROR("%s",ex.what());
//			}

			std::vector<geometry_msgs::Point32> Points;
			geometry_msgs::Point32 Point;
			double pt_x=robotorigin.pose.position.x;
			double pt_y=robotorigin.pose.position.y;

			double pt_th;// = tf::getYaw(pt.pose.orientation);


			double minx=round((pt_x-searchradius)/map_info.scale);
			double maxx=round((pt_x+searchradius)/map_info.scale);
			double miny=round((pt_y-searchradius)/map_info.scale);
			double maxy=round((pt_y+searchradius)/map_info.scale);


			//since circle wont ever be point symetrical we need to iterate through all quadrants
			//this is caused since we wont shift the circlecenter to the middle of the nearest field otherwise symmetry could be used
			Points.clear();
			//first quadrant
			int i,j;
			j=round(pt_y/map_info.scale);
			for (i=maxx; i >round(pt_x/map_info.scale); i--) {
				while(checkcellincircle(i,j,pt_x,pt_y,map_info,Points))
				{

					j++;
				}
				j--;
			}

			//second quadrant
			i=round(pt_x/map_info.scale);
			for (j=maxy; j >round(pt_y/map_info.scale); j--) {
				while(checkcellincircle(i,j,pt_x,pt_y,map_info,Points))
				{
					i--;
				}
				i++;
			}
			//third quadrant
			j=round(pt_y/map_info.scale);
			for (i=minx; i<round(pt_x/map_info.scale); i++) {
				while(checkcellincircle(i,j,pt_x,pt_y,map_info,Points))
				{
					j--;
				}
				j++;
			}
			//fourth quadrant
			i=round(pt_x/map_info.scale);
			for (j=miny; j <round(pt_y/map_info.scale); j++) {
				while(checkcellincircle(i,j,pt_x,pt_y,map_info,Points))
				{
					i++;
				}
				i--;
			}

			cloud.points.clear();
			cloud.header.stamp=robotorigin.header.stamp;
			cloud.header.frame_id="map";
			int cellx,celly;
			//now we have a vector containing all cells intersected by the circle ordered by their angle starting on the x axis
			//we need to calculate their position relative to the map frame
			for(i=0; i<Points.size();i++)
			{
				cellx=MAP_CX(map_info,Points[i].x);
				celly=MAP_CY(map_info,Points[i].y);

				try{
					//check bounds of map and don't add them to the cloud
					if((cellx>=0&&cellx<map_info.size_x)&&(celly>=0&&celly<map_info.size_y))
						{
						geometry_msgs::Point32 test;
						test.x=Points[i].x*map_info.scale;
						test.y=Points[i].y*map_info.scale;
						if(Map.data.at(MAP_INDEX(map_info,MAP_CX(map_info,Points[i].x),MAP_CY(map_info,Points[i].y)))>MapCostThresh){


							test.z=0;
							cloud.points.push_back(test);

						}
					}
				}
				catch(...){}
			}

			PointCloudReducer(reductionradius);//reducing the pointcloud to a minimum
			draw.cloud(cloud);

			//after the reduction we should have 4 or more points since both road borders will intersect the circle twice
			//now we need to delete the points behind the robot easiest is to transform them into the base_footrpint frame
			//this will make the evaluation a lot easier

			geometry_msgs::PointStamped p;
			goalpoints.clear();


			for(int i=0;i<cloud.points.size();i++)
			{
				p.header.frame_id="map";
				p.header.stamp=ros::Time::now();
				p.point.x=cloud.points.at(i).x;
				p.point.y=cloud.points.at(i).y;
				p.point.z=cloud.points.at(i).z;
				try{
					map_to_base_footprint=tf_buffer.lookupTransform("base_footprint","map", ros::Time(0), ros::Duration(1.0) );
					//trying to transform the points back into the base_footprint frame
					tf2::doTransform(p,p, map_to_base_footprint);
				}
				catch (tf2::TransformException &ex){
					ROS_ERROR("%s",ex.what());
				}
				//adding all points infront of the robot to the vector

				if(p.point.x>0)
				{
					draw.mappoints(p);
					goalpoints.push_back(p);
				}
			}
		//now we should have a cleaned up vector consisting solely of points infront of the robot on the borders
		}
	}
	void PointCloudReducer(double distthresh)
	{
		//this function takes a point cloud and reduces it by averaging all points in a given radius

		geometry_msgs::Point32 p;


		std::vector<struct APoint> BorderPoints;
		struct APoint POINT;
		bool trigger;


		for(int i=0; i<cloud.points.size();i++)
		{
			POINT.x=cloud.points.at(i).x;
			POINT.y=cloud.points.at(i).y;
			POINT.N=1;

			//check if the next point is close to one of the chunks and calculate it into the average
			//else a new chunk will be generated

			trigger=true;
			for (int j=0;j<BorderPoints.size();j++)
			{
				if(sqrt(pow(BorderPoints.at(j).x-POINT.x,2)+pow(BorderPoints.at(j).y-POINT.y,2))<distthresh)
				{
					BorderPoints.at(j).x=(BorderPoints.at(j).x*BorderPoints.at(j).N+POINT.x)/(BorderPoints.at(j).N+1);
					BorderPoints.at(j).y=(BorderPoints.at(j).y*BorderPoints.at(j).N+POINT.y)/(BorderPoints.at(j).N+1);
					BorderPoints.at(j).N+=1;
					j=BorderPoints.size()+1;
					trigger=false;
				}
			}
			if(trigger)
			{
				BorderPoints.push_back(POINT);
			}
		}
		cloud.points.clear();

		for (int i=0;i<BorderPoints.size();i++)
		{
			p.x=BorderPoints.at(i).x;
			p.y=BorderPoints.at(i).y;
			p.z=0;
			cloud.points.push_back(p);
		}
	}
	bool checkcellincircle(int &i, int &j,double &pt_x,double &pt_y, map_inf &map,std::vector<geometry_msgs::Point32> &Points)
		{
			//this function checks the shortest distance from pt_x/pt_y to the given cell
			//if the distance is <= the searchradius the point gets added to the Pointsvector and the function returns true
			//else the function returns false
			double cminx,cminy,dx,dy;
			geometry_msgs::Point32 Point;

			//transform from origin of cell to center of cell
			if(i*map.scale>pt_x)
				dx=abs(pt_x-i*map.scale-map.scale/2);
			else if(i*map.scale<pt_x)
				dx=abs(pt_x-i*map.scale+map.scale/2);
			else dx=0;
			if(j*map.scale>pt_y)
				dy=abs(pt_y-j*map.scale-map.scale/2);
			else if(j*map.scale<pt_y)
				dy=abs(pt_y-j*map.scale+map.scale/2);
			else dy=0;

			//getting coordinates of closest point on cell edge by using intercept theorem so we don't need to use cos/sin/tan
			if((abs(dx)>=abs(dy))||dy==0)
			{
				cminx=map.scale;
				cminy=(abs(dy)/abs(dx))*map.scale;
			}

			else if((abs(dx)<abs(dy))||dx==0)
			{
				cminy=map.scale;
				cminx=(abs(dx)/abs(dy))*map.scale;
			}
			if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=GoalDist)
			{
				Point.x=i;
				Point.y=j;
				Points.push_back(Point);
				return true;

			}

			else
				{
				return false;
				}
		}
	void constructgoalbymap(void)
	{

		//since we cant be sure that the points infront of the robot are only from the road
		//we can only take the average instead of the weighted average
		tf2::Quaternion myQuaternion;

		double x,y,m1,m2,m;
		double xmax,xmin,ymax,ymin;
		for(int i=0; i<goalpoints.size();i++)
		{
			x+=goalpoints.at(i).point.x;
			y+=goalpoints.at(i).point.y;

		}
		//if circles are large the slope is calculated as a straight line from the robot to the goal
		if(leftcircle.r>CRadThresh||rightcircle.r>CRadThresh)
		{
			m=atan2(y,x);
		}
		//else we estimate the slope the same way we do at the circle approximation
		else
		{
			m1 =atan2(y-leftcircle.y,x-leftcircle.x);
			m2 =atan2(y-rightcircle.y,x-rightcircle.x);
			if(leftcircle.y>0) m1+=M_PI/2;
			else m1-=M_PI/2;

			if(rightcircle.y>0)m2+=M_PI/2;
			else m2-=M_PI/2;
			m=(m1+m2)/2;

		}

		x/=goalpoints.size();
		y/=goalpoints.size();


		goal.target_pose.header.frame_id=goalpoints.at(0).header.frame_id;
		goal.target_pose.header.stamp = goalpoints.at(0).header.stamp;
		goal.target_pose.pose.position.x=x;
		goal.target_pose.pose.position.y=y;
		goal.target_pose.pose.position.z=0;
		myQuaternion.setRPY( 0, 0, atan(m) );
		myQuaternion=myQuaternion.normalized();
		goal.target_pose.pose.orientation.x=myQuaternion.getX();
		goal.target_pose.pose.orientation.y=myQuaternion.getY();
		goal.target_pose.pose.orientation.z=myQuaternion.getZ();
		goal.target_pose.pose.orientation.w=myQuaternion.getW();
		ROS_INFO("goal from map");

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
			ROS_INFO("goal from lines");

		}



public:
	//Tuneable parameters TODO dynamic recon


	//move base action client
	Drawer draw;

	GoalFinder() : ac ("move_base",true)
	{
		ac.waitForServer();
	};
	move_base_msgs::MoveBaseGoal goal;


	struct CIRCLE leftcircle,rightcircle;
	struct LINE leftline, rightline;
	//goal manager used to determine which goal needs to be build
	void constructgoal()
			{
				leftcircle=leastsquarecircle(LeftLine);
				rightcircle=leastsquarecircle(RightLine);
				ConstructPointCloud(GoalDist);
				switch (Finder)
				{
					case 0:
						if(goalpoints.size()>=1)Finder=2;
						//Circle approximation
						else if(LeftLine.points.size()>0&&LeftLine.points.size()>0){
							//switching to line approx since straight is coming up
							if(leftcircle.r>CRadThresh||rightcircle.r>CRadThresh)
							{
								Finder=1;
								break;
							}
							else
							{
								ROS_INFO("goal from circles");
								constructgoalbycircles(leftcircle, rightcircle, GoalDist, GoalAngle);
								sendgoal();
								draw.circles(leftcircle,rightcircle);
							}
						}
						break;

					case 1:
						if(goalpoints.size()>=1)Finder=2;
						//Line approximation
						else if(LeftLine.points.size()>0&&LeftLine.points.size()>0){
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
								ROS_INFO("goal from lines");
								constructgoalbylines(leftline, rightline, GoalDist);
								sendgoal();
								draw.lines(leftline,rightline);
							}
						}
						break;

					case 2:
						if(goalpoints.size()>=1)
						{
							constructgoalbymap();
							sendgoal();
							ROS_INFO("goal from map");}
						else
						{
							Finder=0;
						}

						break;
				}
			}
	void sendgoal()
	{


		//sending goal to move_base via actionserver
		//ac.waitForResult(ros::Duration(5));
		if(goaltrigger)
		{
		    ac.sendGoal(goal,
		    		MoveBaseClient::SimpleDoneCallback(),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&GoalFinder::fb_callback, this, _1));

		    actionlib::SimpleClientGoalState state=ac.getState();

	    }
			//if(ac.waitForResult(ros::Duration(0.5))==ac.){}
		else
		{
			ac.cancelAllGoals();
		}
	}
	void fb_callback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
	{
		//Doesnt return wanted results
		//ROS_INFO("%f",feedback->base_position.pose.position.x);
	}

	bool goalnotfound(arlo_navigation::goalnotfoundRequest &request,arlo_navigation::goalnotfoundResponse &response)
	{
		//this service call is meant to help the robot when it doesnt find a path to the goal
		//everytime this service gets called the Distance shrinks to 90% and a new goal will be planned
		ConstructPointCloud(GoalDist*0.9);
		constructgoalbymap();
		response.result=GoalDist*0.9;
		ROS_INFO("Constructed a new Goal since Planning failed with the old goal");
		return true;
	}
	};

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "posefinder");


	ros::NodeHandle n;
	Listener listener;
	GoalFinder finder;



	//pub=n.advertise<geometry_msgs::PointStamped>("TestPoint", 1000);

	//subscribers
	ros::Subscriber map = n.subscribe("/map", 1000, &Listener::mapCallback, &listener);
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scan = n.subscribe("/scan_filtered", 1000, &Listener::scanCallback, &listener);
	ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);

	//dynmic reconfigure
	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig> server;
	dynamic_reconfigure::Server<arlo_navigation::posefinderConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	//service
	ros::ServiceClient client = n.serviceClient<arlo_navigation::clearleftlane>("clearblockage");
	ros::ServiceServer goalserver = n.advertiseService("posefinder", &GoalFinder::goalnotfound, &finder);
	arlo_navigation::clearleftlaneRequest req;
	arlo_navigation::clearleftlaneResponse res;

	//wait untill the actionserverclient in the goalfinder finds the movebase server


	ros::Rate rate=1;
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

