#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"


#include "road_detection/LineMatch.h"
#include "road_detection/Line.h"
#include "road_detection/LineArray.h"
#include "road_detection/Road.h"


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//#include <pcl/point_types.h>

#include <math.h>

#include <sstream>

//TODO dyn_recon
double searchradius=1000; //distance in mm

geometry_msgs::Quaternion orientation;
geometry_msgs::Point position;

//Thresholds for slam data evaluation everything between unknown thresh and known thresh gets treated as empty space
//TODO make these configurable using dyn_recon
double unknownThresh;
double knownThresh;

ros::Publisher pub;

class Listener
	{
	public:
		void roadCallback(const road_detection::Road& road)
		{

			geometry_msgs::Point32 left_line[]= road.lineLeft.points;
			constructcloud(left_line, 100, "base_footprint");
		};


	public:
		void mapCallback(const nav_msgs::OccupancyGrid& grid)
		{

			int map_height=grid.info.height;
			int map_width=grid.info.width;
			int pixel_x;
			int pixel_y;
			double Distance;

			for (int i=0; i<grid.data.size(); i++)
			{

				//get pixel coordinate relative to map origin
				pixel_x=grid.info.origin.position.x-(i%grid.info.width);
				pixel_y=grid.info.origin.position.y-((int)floor(i/grid.info.width));

				//calc Distance between Arlo and given Pixel (assumption map origin=robot pos reference)
				Distance=sqrt(pow(abs(pixel_x-position.x),2.0)+pow(abs(pixel_x-position.x),2.0));
				if(grid.data.at(i)>0&&grid.data.at(i)<100)
				{
					ROS_INFO("%d",grid.data.at(i));

				}

			}
		};
	public:
		void odomCallback(const nav_msgs::Odometry& odom)
		{

			position=odom.pose.pose.position;
			orientation=odom.pose.pose.orientation;
		};

	};



void constructcloud(geometry_msgs::Point32 points[], int intensity, char frame_id[])
	{
	//Construct PointCloud2 Message

		int Count=sizeof(points);
		int borderintensity=100;
		int middleintensity=100;
		// Populate message fields
		const uint32_t POINT_STEP = 16;
		sensor_msgs::PointCloud2 msg;
		msg.header.frame_id = frame_id;
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
			*((float*)(ptr + 8))=points[i].z;
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
		pub.publish(msg);
	}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "markfreespace");
	ros::NodeHandle n;
	Listener listener;

	pub = n.advertise<sensor_msgs::PointCloud2>("RoadPointCloud2", 1000);
	//TODO maybe 3 different point clouds for each line of the track
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber map = n.subscribe("/map", 1000, &Listener::mapCallback, &listener);
	ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);
	ros::spin();
  return 0;
}

