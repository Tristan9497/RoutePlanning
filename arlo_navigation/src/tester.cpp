#include "ros/ros.h"
#include "road_detection/Road.h"
#include "gazebo/gazebo.hh"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
//ft2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::Pose truepose;
road_detection::Road road;
road_detection::Line rightlane;
class Listener
	{
public:

	void roadCallback(const road_detection::RoadConstPtr& curroad)
	{
		road=*curroad;
		ros::Duration diff=ros::Time::now()-curroad->laneRight.header.stamp;
		if(curroad->laneRight.points.size()>0){rightlane=curroad->laneRight;rightlane.header.stamp=curroad->header.stamp;}

	};
	void linkCallback(const gazebo_msgs::LinkStatesConstPtr& state)
	{
		//search for base footprint and offset by 45cm so it starts with 0/0
		int index;

		for(int i =0; i<state->name.size();i++)
		{
			if(state->name[i]=="Arlo::base_footprint") index=i;

		}
		truepose=state->pose[index];
		truepose.position.y+=0.45;

	}

	};



class Tester
	//this class incorporates test evaluation procedures for the arlo_navigation
	{
	private:
	bool offlanetrigger=false;
	std_msgs::Int64 offlanecounter;
	double polynomial(double x, road_detection::Line::_polynomial_type::_a_type a) {
		double xp = 1;
		double result = 0;

		for ( int i = 0; i < a.size(); i++ ) {
			result += xp * a[i];
			xp *= x;
		}
		return result;
	}
	double angle(double x, road_detection::Line line)
	{
		double evaldist=x+0.1;
		return atan2(polynomial(evaldist,line.polynomial.a)-polynomial(x,line.polynomial.a),evaldist);
	}

	public:
	std_msgs::Float64 lanediff(void)
	{
		std_msgs::Float64 diff;
		geometry_msgs::TransformStamped transformStamped;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener(tfBuffer);
		geometry_msgs::PointStamped pt;
		pt.header.frame_id="base_footprint";
		pt.header.stamp=rightlane.header.stamp;
		//getting transform for the last occurance of the right lane
		//getting x offset and checking diff

		try{
			transformStamped = tfBuffer.lookupTransform("base_footprint","odom",rightlane.header.stamp,ros::Duration(1));
			tf2::doTransform(pt,pt, transformStamped);
			ROS_INFO("%f,%f",pt.point.x,pt.point.y);
			diff.data = abs(polynomial(transformStamped.transform.translation.x,rightlane.polynomial.a)*sin((M_PI/2)-angle(transformStamped.transform.translation.x,rightlane)));
		}
		catch(...){}
		return diff;
	}
		double diff;
		std_msgs::Int64 checklanediff()
		{
			if(!offlanetrigger&&lanediff().data>0.45){
				offlanetrigger=true;
				offlanecounter.data++;
			}
			else if(offlanetrigger&&lanediff().data<0.45)
			{
				offlanetrigger=false;
			}
			return offlanecounter;
		}

	};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arlo_tester");
	ros::NodeHandle n;
	Listener listener;
	Tester tester;

	ros::Publisher trueposepub =n.advertise<geometry_msgs::Pose>("/tester/truepose",1000);
	ros::Publisher diffpub =n.advertise<std_msgs::Float64>("/tester/lanediff",1000);
	ros::Publisher diffcounterpub =n.advertise<std_msgs::Int64>("/tester/offlanecounter",1000);
	//TODO maybe 3 different point clouds for each line of the track
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber state = n.subscribe("/gazebo/link_states", 1000, &Listener::linkCallback, &listener);
	//TODO configure frequency
	ros::Rate rate(10);
	bool offlanetrigger;
	//copy smaller vector to larger and publish it as pointcloud2

	while(n.ok()){
		ros::spinOnce();
		trueposepub.publish(truepose);
		diffpub.publish(tester.lanediff());
		diffcounterpub.publish(tester.checklanediff());

		rate.sleep();
	}

  return 0;
}
