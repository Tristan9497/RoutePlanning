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

//PCL Stuff
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//Lidar
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>



//TODO dyn_recon
double searchradius=1000; //distance in mm

geometry_msgs::Quaternion orientation;
geometry_msgs::Point position;

//Thresholds for slam data evaluation everything between unknown thresh and known thresh gets treated as empty space
//TODO make these configurable using dyn_recon
//double unknownThresh;
//double knownThresh;

ros::Publisher pub;
sensor_msgs::PointCloud2 constructcloud(std::vector<geometry_msgs::Point32> &points, float intensity, std::string frame);
std::vector<geometry_msgs::Point32> points;
class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{
			//Transforming points of road_detection into sensor_msgs::PointCloud2 so they can be used in the costmap
			points.clear();
			//publishing borders and middle line individual to give better flexibility
			for(int i=0;i<road->lineLeft.points.size();i++)
			{
				points.push_back(road->lineLeft.points[i]);
			}
			for(int j=0;j<road->lineRight.points.size();j++)
			{

				points.push_back(road->lineRight.points[j]);
			}

//			for(int i=0;i<road->lineMiddle.points.size();i++)
//			{
//				points.push_back(road->lineMiddle.points[i]);
//			}

		};

	public:

		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			//generates combined pointcloud of lidar and roaddetection
			std::vector<geometry_msgs::Point32> scan;
			geometry_msgs::Point32 scanpoint;
			float angle;
			float range;
			for(int i=0;i<Scan->ranges.size();i++)
			{
				angle=(i*Scan->angle_increment)+Scan->angle_min;
				range=Scan->ranges.at(i);
				//offset value from arlo_2stack urdf

				scanpoint.x=range*cos(angle)+0.175;
				scanpoint.y=range*sin(angle);
				scanpoint.z=0;
				scan.push_back(scanpoint);

			}
			//copy smaller vector to larger and publish it as pointcloud2
			if(scan.size()>=points.size()){
				scan.insert( scan.end(), points.begin(), points.end() );
				pub.publish(constructcloud(scan,100,"base_link"));
			}
			else
			{
				points.insert( points.end(), scan.begin(), scan.end() );
				pub.publish(constructcloud(points,100,"base_link"));
			}
		}
//	public:
//		void mapCallback(const nav_msgs::OccupancyGridConstPtr& grid)
//		{
//			//Transforming points of slam map into sensor_msgs::PointCloud2 so they can be used in the costmap to have costs for points we have seen before but not atm
//			int map_height=grid->info.height;
//			int map_width=grid->info.width;
//
//			int pixel_x;
//			int pixel_y;
//			double coord_x;
//			double coord_y;
//
//			//resolution in m/cell
//			double resolution = grid->info.resolution;
//			double Distance;
//			std::vector<geometry_msgs::Point32> points;
//			geometry_msgs::Point32 currentpoint;
//
//			for (int i=0; i<grid->data.size(); i++)
//			{
//
//				//get pixel coordinate relative to map origin
//				pixel_x=grid->info.origin.position.x-(i%map_width);
//				pixel_y=grid->info.origin.position.y-((int)floor(i/map_width));
//
//				currentpoint.x=pixel_x*resolution;
//				currentpoint.y=pixel_y*resolution;
//				currentpoint.z=0;
//
//				if(grid->data.at(i)>=50)
//				{
//					points.push_back(currentpoint);
//				}
//
//				//calc Distance between Arlo and given Pixel (assumption map origin=robot pos reference)
////				Distance=sqrt(pow(abs(pixel_x-position.x),2.0)+pow(abs(pixel_x-position.x),2.0));
//
////				if(Distance<searchradius)
////				{
////					currentpoint.x=grid->info.resolution;
////					points.push_back(currentpoint);
////				}
//			}
//			pub.publish(constructcloud(points,100,"map"));
//
//		};
//	public:
//		void odomCallback(const nav_msgs::OdometryConstPtr& odom)
//		{
//			//get current position
//
//			position=odom->pose.pose.position;
//			orientation=odom->pose.pose.orientation;
//		};

	};



sensor_msgs::PointCloud2 constructcloud(std::vector<geometry_msgs::Point32> &points, float intensity, std::string frame)
	{

	//Construct PointCloud2 Message with given array of points tf_frame and intensity

		int Count=points.size();
		// Populate message fields
		const uint32_t POINT_STEP = 16;
		sensor_msgs::PointCloud2 msg;
		msg.header.frame_id =frame;
		msg.header.stamp = ros::Time::now();
		msg.fields.resize(4);
		msg.fields[0].name = "x";
		msg.fields[0].offset = 0;
		msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		msg.fields[0].count = 1;
		msg.fields[1].name = "y";
		msg.fields[1].offset = 4;
		msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		msg.fields[1].count = 1;
		msg.fields[2].name = "z";
		msg.fields[2].offset = 8;
		msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		msg.fields[2].count = 1;
		msg.fields[3].name = "intensity";
		msg.fields[3].offset = 12;
		msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
		msg.fields[3].count = 1;

		msg.data.resize(Count * POINT_STEP);

		int i;
		uint8_t *ptr = msg.data.data();
		for(i=0; i<Count;i++)
		{
			*((float*)(ptr + 0))=points[i].x;
			*((float*)(ptr + 4))=points[i].y;
			*((float*)(ptr + 8))=0.01;
			*((float*)(ptr + 12))=intensity;
			ptr += POINT_STEP;
		}

		msg.point_step = POINT_STEP;
		msg.row_step = ptr - msg.data.data();
		msg.height = 1; //since data is not ordered
		msg.width = msg.row_step / POINT_STEP;
		msg.is_bigendian = false;
		msg.is_dense = true; //maybe invalid points
		msg.data.resize(msg.row_step); // Shrink to actual size
		return msg;
	}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "markfreespace");
	ros::NodeHandle n;
	Listener listener;
	tf::TransformListener tflistener;
	tf::StampedTransform transform;
	pub = n.advertise<sensor_msgs::PointCloud2>("RoadPointCloud2", 1000);
	//TODO maybe 3 different point clouds for each line of the track
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scan = n.subscribe("/base_scan", 1000, &Listener::scanCallback, &listener);
	//ros::Subscriber map = n.subscribe("/map", 1000, &Listener::mapCallback, &listener);
	//ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);

	ros::spin();
  return 0;
}

