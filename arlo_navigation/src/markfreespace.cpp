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

//Services
#include "arlo_navigation/clearleftlane.h"


//dyn_recon
#include <dynamic_reconfigure/server.h>
#include <arlo_navigation/markfreespaceConfig.h>


//ft2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


geometry_msgs::Quaternion orientation;
geometry_msgs::Point position;

//Thresholds for slam data evaluation everything between unknown thresh and known thresh gets treated as empty space
//TODO make these configurable using dyn_recon

bool middletrigger;
bool blockagetrigger=true;
ros::Publisher pub;
ros::Publisher inflationpub;
ros::Publisher linepub;
sensor_msgs::PointCloud2 constructcloud(std::vector<geometry_msgs::Point32> &points, float intensity, std::string frame, ros::Time stamp);

std::vector<geometry_msgs::Point32> navpoints;



sensor_msgs::PointCloud InflatePoints;
sensor_msgs::ChannelFloat32 RadInfo;
sensor_msgs::ChannelFloat32 MaxCost;
sensor_msgs::ChannelFloat32 MinCost;

double inflaterad;
double maxcost;
double mincost;
double clearrad,obstaclerad,inflatedist,cleardist;


void callback(arlo_navigation::markfreespaceConfig &config, uint32_t level) {
	middletrigger=config.middleline;

	maxcost=config.MaxCost;
	mincost=config.MinCost;

	inflaterad=config.LeftInflation;
	clearrad=config.RightRemoval;
	obstaclerad=config.ObstacleRemoval;

	inflatedist=config.InflPointDistance;
	cleardist=config.ClearPointDistance;
}



class Listener
	{
	private:
		road_detection::Line Left,Right,Middle;
		double lwidth,rwidth;
		ros::Time obstaclstamp;

		std::vector<geometry_msgs::Point32> points;
		std::vector<geometry_msgs::Point32> scan;
		std::vector<geometry_msgs::Point32> obstacles;

		double polynomial(double x, road_detection::Line::_polynomial_type::_a_type a) {
			double xp = 1;
			double result = 0;

			for ( int i = 0; i < a.size(); i++ ) {
				result += xp * a[i];
				xp *= x;
			}
			return result;
		}
	public:
		std::string roadframe="base_footprint";//starting value since wee have to wait for the road detection
		ros::Time roadstamp;
		void roadCallback(const road_detection::RoadConstPtr& road)
		{


			geometry_msgs::Point32 Buffer;


			Left=road->lineLeft;
			Right=road->lineRight;
			lwidth=road->laneWidthLeft;
			rwidth=road->laneWidthRight;
			roadframe=road->header.frame_id;
			roadstamp=road->header.stamp;
			//Transforming points of road_detection into sensor_msgs::PointCloud2 so they can be used for slam and the costmap
			points.clear();

			//publishing borders and middle line individual to give better flexibility
			for(int i=0;i<road->lineLeft.points.size();i++)
			{
				Buffer.x=road->lineLeft.points[i].x;
				Buffer.y=road->lineLeft.points[i].y;

				points.push_back(Buffer);
			}

			inflaterad=road->laneWidthRight*0.4;

			for(int j=0;j<road->lineRight.points.size();j++)
			{
				Buffer.x=road->lineRight.points[j].x;
				Buffer.y=road->lineRight.points[j].y;

				points.push_back(Buffer);
			}

			for(int k=0;k<road->lineMiddle.points.size();k++)
			{
				Buffer.x=road->lineMiddle.points[k].x;
				Buffer.y=road->lineMiddle.points[k].y;
				//functionality to switch of middle line
				if(middletrigger) points.push_back(Buffer);
			}


			//Line Publisher for navigation (costmap)
			navpoints=points;


		}
		void Inflate()
		{
			geometry_msgs::Point32 Buffer, Old;

			RadInfo.values.clear();
			MaxCost.values.clear();
			InflatePoints.channels.clear();
			InflatePoints.points.clear();


			RadInfo.name="InflationRadius";
			MaxCost.name="MaxCost";
			InflatePoints.header.frame_id=Left.header.frame_id;
			InflatePoints.header.stamp=Left.header.stamp;


			if(inflatedist==0&&Left.points.size()>0)
			{

				Buffer.x=Left.points.back().x;
				Buffer.y=Left.points.back().y;
				InflatePoints.points.push_back(Buffer);
				RadInfo.values.push_back(lwidth+rwidth*inflaterad);
				MaxCost.values.push_back(maxcost);
				MinCost.values.push_back(mincost);
			}
			else{
				for(int i=0;i<Left.points.size();i++)
				{
					Buffer.x=Left.points[i].x;
					Buffer.y=Left.points[i].y;

					if(i==0||sqrt(pow(Old.x-Buffer.x,2)+pow(Old.y-Buffer.y,2))>inflatedist){

						InflatePoints.points.push_back(Buffer);
						RadInfo.values.push_back(lwidth+rwidth*inflaterad);
						MaxCost.values.push_back(maxcost);
						MinCost.values.push_back(mincost);
						Old=Buffer;
					}
				}
			}
			if(cleardist==0&&Right.points.size()>0)
			{
				Buffer.x=Right.points.back().x;
				Buffer.y=Right.points.back().y;
				InflatePoints.points.push_back(Buffer);
				RadInfo.values.push_back(rwidth*clearrad);//adding points of right line so we allways have a free lane on the right even when street moves
				MaxCost.values.push_back(0);
				MinCost.values.push_back(0);
			}
			else{
				for(int j=0;j<Right.points.size();j++)
				{
					Buffer.x=Right.points[j].x;
					Buffer.y=Right.points[j].y;

					if(j==0||sqrt(pow(Old.x-Buffer.x,2)+pow(Old.y-Buffer.y,2))>cleardist){

						InflatePoints.points.push_back(Buffer);
						RadInfo.values.push_back(rwidth*clearrad);//adding points of right line so we allways have a free lane on the right even when street moves
						MaxCost.values.push_back(0);
						MinCost.values.push_back(0);
						Old=Buffer;
					}
				}
			}


			for(int j=0;j<obstacles.size();j++)
			{
				Buffer.x=obstacles[j].x;
				Buffer.y=obstacles[j].y;

				if(j==0||sqrt(pow(Old.x-Buffer.x,2)+pow(Old.y-Buffer.y,2))>obstaclerad*0.66){

					InflatePoints.points.push_back(Buffer);
					RadInfo.values.push_back(obstaclerad);//adding points of right line so we allways have a free lane on the right even when street moves
					MaxCost.values.push_back(0);
					MinCost.values.push_back(0);
					Old=Buffer;
				}
			}
		}

		void scanCallback(const sensor_msgs::LaserScanConstPtr& Scan)
		{
			//generates combined pointcloud of lidar and roaddetection
			scan.clear();
			obstacles.clear();
			geometry_msgs::Point32 scanpoint;
			geometry_msgs::PointStamped tpt;
			float angle;
			float range;

			geometry_msgs::TransformStamped transformStamped;
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener(tfBuffer);

			for(int i=0;i<Scan->ranges.size();i++)
			{
				angle=(i*Scan->angle_increment)+Scan->angle_min;
				range=Scan->ranges.at(i);
				//offset value from arlo_2stack urdf

				tpt.point.x=range*cos(angle);
				tpt.point.y=range*sin(angle);
				tpt.point.z=0.0;
				try{
					//transform scanned point into the frame of the road detection then compare it with the road
					transformStamped = tfBuffer.lookupTransform( roadframe,Scan->header.frame_id,Scan->header.stamp,ros::Duration(1));

					tf2::doTransform(tpt,tpt, transformStamped);
					scanpoint.x=tpt.point.x;
					scanpoint.y=tpt.point.y;
					scanpoint.z=tpt.point.z;
					//pub.publish(Test);
					//check whether the point is in the lane by projecting it on to the 3 roadlines and comparing their y values
					//exact calculation not necessary since road makes wide curves
					if(polynomial(scanpoint.x, Left.polynomial.a)>scanpoint.y && polynomial(scanpoint.x, Right.polynomial.a)<scanpoint.y)
					{
						obstacles.push_back(scanpoint);
					}
//					else if(polynomial(scanpoint.point.x, Right.polynomial.a)<scanpoint.point.y && polynomial(scanpoint.point.x, Middle.polynomial.a)>scanpoint.point.y)
//					{
//					}
					scan.push_back(scanpoint);

				}
				catch(tf2::TransformException &ex)
				{
					//ROS_WARN("%s", ex.what());
				}

			}

			if(scan.size()>=points.size()){
				scan.insert( scan.end(), points.begin(), points.end() );
				if(scan.size()>0) pub.publish(constructcloud(scan,100,"base_footprint",roadstamp));
			}

			else
			{
				points.insert( points.end(), scan.begin(), scan.end() );
				if(points.size()>0) pub.publish(constructcloud(points,100,"base_footprint",roadstamp));
			}

		}
		bool clearleftline(arlo_navigation::clearleftlaneRequest &request,arlo_navigation::clearleftlaneResponse &response)
		{
			//inverting this bit switches the blockage of the left lane on and off.
			//is meant to be controlled by the pose finder and can be used in escape manouvers

			blockagetrigger=request.trigger;
			response.result=blockagetrigger;
			return true;
		}

	};



sensor_msgs::PointCloud2 constructcloud(std::vector<geometry_msgs::Point32> &points, float intensity, std::string frame, ros::Time stamp)
	{

	//Construct PointCloud2 Message with given array of points tf_frame and intensity

		int Count=points.size();
		// Populate message fields
		const uint32_t POINT_STEP = 16;
		sensor_msgs::PointCloud2 msg;
		msg.header.frame_id =frame;
		msg.header.stamp = stamp;
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
	ros::Subscriber scansub = n.subscribe("/scan_filtered", 1000, &Listener::scanCallback, &listener);
	//ros::Subscriber map = n.subscribe("/map", 1000, &Listener::mapCallback, &listener);
	//ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);
	ros::ServiceServer clearblockage = n.advertiseService("clearblockage", &Listener::clearleftline, &listener);

	//TODO configure frequency
	ros::Rate rate(10);
	ros::Rate inflaterate(0.25);
	ros::Time oldstamp;
	//copy smaller vector to larger and publish it as pointcloud2
	while(n.ok()){
		linepub.publish(constructcloud(navpoints,100,"base_footprint",listener.roadstamp));
		if((ros::Time::now().sec-oldstamp.sec)>inflaterate.cycleTime().sec)
		{
			listener.Inflate();
			if(InflatePoints.points.size()>0){


				InflatePoints.channels.push_back(RadInfo);
				InflatePoints.channels.push_back(MaxCost);
				InflatePoints.channels.push_back(MinCost);
				inflationpub.publish(InflatePoints);
			}
			oldstamp=ros::Time::now();
		}
		ros::spinOnce();
		rate.sleep();
	}

  return 0;
}

