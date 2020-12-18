//general includes
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
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
#include <sensor_msgs/ChannelFloat32.h>

//Lidar
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>




//dyn_recon
#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/markfreespaceConfig.h>

//TODO dyn_recon
double searchradius=1000; //distance in mm

geometry_msgs::Quaternion orientation;
geometry_msgs::Point position;

//Thresholds for slam data evaluation everything between unknown thresh and known thresh gets treated as empty space
//TODO make these configurable using dyn_recon

bool middletrigger;

ros::Publisher pub;
ros::Publisher inflationpub;
ros::Publisher linepub;
sensor_msgs::PointCloud2 constructcloud(std::vector<geometry_msgs::Point32> &points, float intensity, std::string frame);
std::vector<geometry_msgs::Point32> points;
std::vector<geometry_msgs::Point32> navpoints;
std::vector<geometry_msgs::Point32> scan;

double leftrad=0.8;
double rightrad=0.15;
double middlerad=0.3;
double leftmax=254;
double rightmax=254;
double middlemax=100;


void callback(arlo_navigation::markfreespaceConfig &config, uint32_t level) {
	middletrigger=config.middleline;
}

class Listener
	{
	public:
		void roadCallback(const road_detection::RoadConstPtr& road)
		{

			geometry_msgs::Point32 Buffer;
			sensor_msgs::PointCloud InflatePoints;
			sensor_msgs::ChannelFloat32 RadInfo;
			sensor_msgs::ChannelFloat32 MaxCost;

			RadInfo.name="InflationRadius";
			MaxCost.name="MaxCost";
			InflatePoints.header.frame_id=road->header.frame_id;
			InflatePoints.header.stamp=ros::Time::now();
			//Transforming points of road_detection into sensor_msgs::PointCloud2 so they can be used for slam and the costmap
			points.clear();
			//publishing borders and middle line individual to give better flexibility
			for(int i=0;i<road->lineLeft.points.size();i++)
			{
				Buffer.x=road->lineLeft.points[i].x;
				Buffer.y=road->lineLeft.points[i].y;

				points.push_back(Buffer);
			}
			//Filtering out error points so they won't go into the costmap
			//errorpoints are caused by roaddetection not seeing the leftline for a moment but the other lines are getting detected
			if(road->lineLeft.points.size()!=0){
				InflatePoints.points.push_back(Buffer);
				RadInfo.values.push_back(leftrad);
				MaxCost.values.push_back(leftmax);
				InflatePoints.channels.push_back(RadInfo);
				InflatePoints.channels.push_back(MaxCost);
				inflationpub.publish(InflatePoints);
			}
			for(int j=0;j<road->lineRight.points.size();j++)
			{
				Buffer.x=road->lineRight.points[j].x;
				Buffer.y=road->lineRight.points[j].y;
				points.push_back(Buffer);
			}
//			InflatePoints.points.push_back(Buffer);
//			RadInfo.values.push_back(rightrad);
//			MaxCost.values.push_back(rightmax);
			for(int k=0;k<road->lineMiddle.points.size();k++)
			{
				Buffer.x=road->lineMiddle.points[k].x;
				Buffer.y=road->lineMiddle.points[k].y;
				//functionality to switch of middle line
				if(middletrigger) points.push_back(Buffer);
			}
//			InflatePoints.points.push_back(Buffer);
//			RadInfo.values.push_back(middlerad);
//			MaxCost.values.push_back(middlemax);


			//Line Publisher for navigation (costmap)
			navpoints=points;

			linepub.publish(constructcloud(navpoints,100,"base_footprint"));
		};

	public:

		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			//generates combined pointcloud of lidar and roaddetection
			scan.clear();
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
				scanpoint.z=0.0;
				scan.push_back(scanpoint);

			}
			if(scan.size()>=points.size()){
				scan.insert( scan.end(), points.begin(), points.end() );
				pub.publish(constructcloud(scan,100,"base_footprint"));
			}
			else
			{
				points.insert( points.end(), scan.begin(), scan.end() );
				pub.publish(constructcloud(points,100,"base_footprint"));
			}

		};

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
		return msg;
	}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "markfreespace");
	ros::NodeHandle n;
	Listener listener;
	tf::TransformListener tflistener;
	tf::StampedTransform transform;

	dynamic_reconfigure::Server<arlo_navigation::markfreespaceConfig> server;
	dynamic_reconfigure::Server<arlo_navigation::markfreespaceConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	inflationpub = n.advertise<sensor_msgs::PointCloud>("/points",1000);
	pub = n.advertise<sensor_msgs::PointCloud2>("RoadPointCloud2", 1000);
	linepub=n.advertise<sensor_msgs::PointCloud2>("NavPointData",1000);

	//TODO maybe 3 different point clouds for each line of the track
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber scansub = n.subscribe("/base_scan", 1000, &Listener::scanCallback, &listener);
	//ros::Subscriber map = n.subscribe("/map", 1000, &Listener::mapCallback, &listener);
	//ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);

	//copy smaller vector to larger and publish it as pointcloud2


	ros::spin();
  return 0;
}

