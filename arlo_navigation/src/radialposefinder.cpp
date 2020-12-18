
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

//tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//dyn_recon
#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/markfreespaceConfig.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>


#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
//getcoordinates relativ to /map frame from cell coordinates
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

//get cell cooardinates from coordinates relative to /map
#define MAP_CX(map,i) (i-round(map.origin_x/map.scale)+floor(map.size_x/2))
#define MAP_CY(map,j) (j-round(map.origin_y/map.scale)+floor(map.size_y/2))
///
#include <visualization_msgs/Marker.h>
////
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
ros::Publisher marker_pub;







visualization_msgs::Marker marker;
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
		if(sqrt(pow(dx-cminx,2)+pow(dy-cminy,2))<=searchradius)
		{
			Point.x=i;
			Point.y=j;
			Points.push_back(Point);
			return true;

		}
		else return false;
	}
class Listener
	{
	public:
		void odom_callback(nav_msgs::Odometry odom)
		{
			robotorigin.header.frame_id=odom.header.frame_id;
			robotorigin.header.stamp=ros::Time::now();
			robotorigin.pose=odom.pose.pose;
			//ROS_INFO("%f,%f",odom.pose.pose.position.x,odom.pose.pose.position.y);

		}
	public:
		void map_callback(const nav_msgs::OccupancyGridConstPtr &map)
		{

			// in this callback a vector of points on a circle around the robot gets created. These will be ordered by their angle and get the cost of the specified map
			//the points in the vector have coordinates as integer relativ to the frame of the map
			//this need to be converted to map index relative to its bottom left position

		    map_inf map_info;
		    //collect all cost data in a circle of radius searchradius
		    map_info.size_x = map->info.width;
		    map_info.size_y = map->info.height;
		    map_info.scale = map->info.resolution;
		    map_info.origin_x = map->info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
		    map_info.origin_y = map->info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

		    double gt_x = map->info.origin.position.x;
		    double gt_y = map->info.origin.position.y;

		    tf2_ros::Buffer tf_buffer;
		    tf2_ros::TransformListener tf2_listener(tf_buffer);
		    geometry_msgs::TransformStamped base_footprint_to_map;
		    //tranform robot pose into map frame
		    geometry_msgs::PoseStamped pt;
		    robotorigin.header.stamp =  map->header.stamp;
		    try{
		    	base_footprint_to_map=tf_buffer.lookupTransform(map->header.frame_id, robotorigin.header.frame_id, ros::Time(0), ros::Duration(1.0) );

		    	tf2::doTransform(robotorigin, robotorigin, base_footprint_to_map);


		    }
		    catch (tf2::TransformException &ex){
		        ROS_ERROR("%s",ex.what());
		    }

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
			sensor_msgs::PointCloud cloud;
			cloud.header.stamp=ros::Time::now();
			cloud.header.frame_id="map";
			int cellx,celly;
			//now we have a vector containing all cells intersected by the circle ordered by their angle starting on the x axis
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
						//if(map->data.at(MAP_INDEX(map_info,MAP_CX(map_info,Points[i].x),MAP_CY(map_info,Points[i].y)))>50){


							test.z=0;


						//}
						//else test.z=-1;
						cloud.points.push_back(test);
					}

				}
				catch(...){}




			}

			pub.publish(cloud);
			//marker_pub.publish(marker);

		};
	public:
		void road_callback(road_detection::Road &road)
		{

			//calculate average lane width with so detection of road in costmap can be flexible
			double widthr=road.laneWidthRight;
			double widthl=road.laneWidthLeft;
			avg_lanewidth=(widthl+widthr)/2;
			//ROS_INFO("%f",avg_lanewidth);
		}
	};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "radialposefinder");
	ros::param::set("/use_sim_time",true);
	ros::NodeHandle n;
	Listener listener;
	tf::TransformListener tflistener;
	tf::StampedTransform transform;
	pub=n.advertise<sensor_msgs::PointCloud>("radialpose",1000);
	ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odom_callback, &listener);
	ros::Subscriber map = n.subscribe("/map", 1000, &Listener::map_callback, &listener);
	//ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::road_callback, &listener);
	///
	marker_pub = n.advertise<visualization_msgs::Marker>("Circle", 1);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 2.0;
	marker.scale.y = 2.0;
	marker.scale.z = 0.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

	///


	ros::spin();
  return 0;
}
