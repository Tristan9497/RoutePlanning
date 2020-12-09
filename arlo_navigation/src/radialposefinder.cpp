
//general includes
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>

//custom message types
#include "road_detection/LineMatch.h"
#include "road_detection/Line.h"
#include "road_detection/LineArray.h"
#include "road_detection/Road.h"



#include <people_msgs/People.h>

//PCL Stuff
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//Lidar
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


//dyn_recon
#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/markfreespaceConfig.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>


#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

struct map_inf {
    double size_x;
    double size_y;
    double scale;
    double origin_x;
    double origin_y;
};
//TODO odom listener
geometry_msgs::PoseStamped robotorigin;

double searchradius=3;//Searchradius in meter
double avg_lanewidth;
double road_width_tolerance=0.1;//in percent

ros::Publisher pub;

class Listener
	{
	public:
		void odom_callback(nav_msgs::Odometry odom)
		{
			robotorigin.header.frame_id=odom.child_frame_id;
			robotorigin.header.stamp=ros::Time::now();
			robotorigin.pose=odom.pose.pose;

		}
	public:
		void map_callback(const nav_msgs::OccupancyGridConstPtr &map)
		{


		    map_inf map_info;
		    //collect all cost data in a circle of radius searchradius
		    map_info.size_x = map->info.width;
		    map_info.size_y = map->info.height;
		    map_info.scale = map->info.resolution;
		    map_info.origin_x = map->info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
		    map_info.origin_y = map->info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

		    double gt_x = map->info.origin.position.x;
		    double gt_y = map->info.origin.position.y;

		    tf::TransformListener tf_listener(ros::Duration(10));
		    //tranform robot pose into map frame
		    geometry_msgs::PoseStamped pt;
		    robotorigin.header.stamp =  map->header.stamp;
		    try{
		        tf_listener.waitForTransform(map->header.frame_id,robotorigin.header.frame_id, ros::Time(0), ros::Duration(3));
		        tf_listener.transformPose(map->header.frame_id, robotorigin, pt);

		    }
		    catch (tf::TransformException &ex){
		        ROS_ERROR("%s",ex.what());
		    }
		    ROS_INFO("%s",robotorigin.header.frame_id.c_str());

		    std::vector<geometry_msgs::Point32> Points;
		    geometry_msgs::Point32 Point;
		    double pt_x=pt.pose.position.x;
		    double pt_y=pt.pose.position.y;
		    ROS_INFO("%f",pt_x);
		    double pt_th;// = tf::getYaw(pt.pose.orientation);


		    double minx=round((pt_x-searchradius)/map_info.scale);
		    double maxx=round((pt_x+searchradius)/map_info.scale);
		    double miny=round((pt_y-searchradius)/map_info.scale);
		    double maxy=round((pt_y+searchradius)/map_info.scale);
		    double cminx,cminy,dx,dy;

		    //since circle wont ever be point symetrical we need to iterate through all quadrants
		    //this is caused since we wont shift the circlecenter to the middle of the nearest field otherwise symmetry could be used
		    Points.clear();
		    //first quadrant
		    int i,j;
		    j=round(pt_y/map_info.scale);
			for (i=maxx; i >round(pt_x/map_info.scale); i--) {
				Point.x=i;
				Point.y=j;
				Points.push_back(Point);
				while(true)
				{
					j++;
					dx=abs(pt_x-i*map_info.scale);
					dy=abs(pt_y-j*map_info.scale);
					//getting coordinates of closest point on cell edge
					if(abs(dx)>=abs(dy)||dy==0)
					{
						cminx=map_info.scale;
						cminy=(abs(dy)/abs(dx))*cminx;
					}
					else if(abs(dx)<abs(dy)||dx==0)
					{
						cminy=map_info.scale;
						cminx=(abs(dx)/abs(dy))*cminy;
					}

					//check distance of closest point and compare
					if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=searchradius)
					{
						Point.x=i;
						Point.y=j;
						Points.push_back(Point);

					}
					else
					{
						j--;
						break;
					}
				}
			}
			//second quadrant
			i=round(pt_x/map_info.scale);
			for (j=maxy; j >round(pt_y/map_info.scale); j--) {
				Point.x=i;
				Point.y=j;
				Points.push_back(Point);
				while(true)
				{
					i--;
					dx=abs(pt_x-i*map_info.scale);
					dy=abs(pt_y-j*map_info.scale);
					//getting coordinates of closest point on cell edge
					if(abs(dx)>=abs(dy)||dy==0)
					{
						cminx=map_info.scale;
						cminy=(abs(dy)/abs(dx))*cminx;
					}
					else if(abs(dx)<abs(dy)||dx==0)
					{
						cminy=map_info.scale;
						cminx=(abs(dx)/abs(dy))*cminy;
					}

					//check distance of closest point and compare
					if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=searchradius)
					{
						Point.x=i;
						Point.y=j;
						Points.push_back(Point);

					}
					else
					{
						i++;
						break;
					}
				}
			}
			//third quadrant
		    j=round(pt_y/map_info.scale);
			for (i=minx; i<round(pt_x/map_info.scale); i++) {
				Point.x=i;
				Point.y=j;
				Points.push_back(Point);
				while(true)
				{
					j--;
					dx=abs(pt_x-i*map_info.scale);
					dy=abs(pt_y-j*map_info.scale);
					//getting coordinates of closest point on cell edge
					if(abs(dx)>=abs(dy)||dy==0)
					{
						cminx=map_info.scale;
						cminy=(abs(dy)/abs(dx))*cminx;
					}
					else if(abs(dx)<abs(dy)||dx==0)
					{
						cminy=map_info.scale;
						cminx=(abs(dx)/abs(dy))*cminy;
					}

					//check distance of closest point and compare
					if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=searchradius)
					{
						Point.x=i;
						Point.y=j;
						Points.push_back(Point);

					}
					else
					{
						j++;
						break;
					}
				}
			}
			//fourth quadrant
			i=round(pt_x/map_info.scale);
			for (j=miny; j <round(pt_y/map_info.scale); j++) {
				Point.x=i;
				Point.y=j;
				Points.push_back(Point);
				while(true)
				{
					i++;
					dx=abs(pt_x-i*map_info.scale);
					dy=abs(pt_y-j*map_info.scale);
					//getting coordinates of closest point on cell edge
					if(abs(dx)>=abs(dy)||dy==0)
					{
						cminx=map_info.scale;
						cminy=(abs(dy)/abs(dx))*cminx;
					}
					else if(abs(dx)<abs(dy)||dx==0)
					{
						cminy=map_info.scale;
						cminx=(abs(dx)/abs(dy))*cminy;
					}

					//check distance of closest point and compare
					if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=searchradius)
					{
						Point.x=i;
						Point.y=j;
						Points.push_back(Point);

					}
					else
					{
						i--;
						break;
					}
				}
			}
			sensor_msgs::PointCloud cloud;
			//now we have a vector containing all cells intersected by the circle ordered by their angle starting on the x axis
			for(i=0; i<Points.size();i++)
			{


				geometry_msgs::Point32 test;
				test.x=Points[i].x*map_info.scale;
				test.y=Points[i].y*map_info.scale;
				try{
					test.z=map->data.at(MAP_INDEX(map_info,Points[i].x,Points[i].y));
				}
				catch(...){}
				cloud.header.stamp=ros::Time::now();
				cloud.header.frame_id="base_footprint";
				cloud.points.push_back(test);



			}
			ROS_INFO("%f",map_info.scale);
			pub.publish(cloud);

		};
	public:
		void road_callback(road_detection::Road &road)
		{

			//calculate average lane width with so detection of road in costmap can be flexible
			double widthr=road.laneWidthRight;
			double widthl=road.laneWidthLeft;
			avg_lanewidth=(widthl+widthr)/2;
			ROS_INFO("%f",avg_lanewidth);
		}
	};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "radialposefinder");
	ros::NodeHandle n;
	Listener listener;
	tf::TransformListener tflistener;
	tf::StampedTransform transform;
	pub=n.advertise<sensor_msgs::PointCloud>("radialpose",1000);
	ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odom_callback, &listener);
	ros::Subscriber map = n.subscribe("/map", 1000, &Listener::map_callback, &listener);
	//ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::road_callback, &listener);



	ros::spin();
  return 0;
}
