#include "ros/ros.h"
#include "road_detection/Road.h"
#include "gazebo/gazebo.hh"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "nav_msgs/Odometry.h"
//ft2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::Pose truepose;
road_detection::Road road;
road_detection::Line rightlane;
nav_msgs::Odometry odom;
nav_msgs::Odometry oldodom;
std::vector<geometry_msgs::Point> obstacles;

class Listener
	{
public:

	void roadCallback(const road_detection::RoadConstPtr& curroad)
	{
		road=*curroad;
		ros::Duration diff=ros::Time::now()-curroad->laneRight.header.stamp;
		if(curroad->laneRight.points.size()>0){rightlane=curroad->laneRight;rightlane.header.stamp=curroad->header.stamp;oldodom=odom;}

	}
	void linkCallback(const gazebo_msgs::LinkStatesConstPtr& state)
	{
		//search for base footprint and offset by 45cm so it starts with 0/0
		int index;
		geometry_msgs::Point obstacle;
		obstacles.clear();
		for(int i =0; i<state->name.size();i++)
		{
			if(state->name[i]=="Arlo") index=i;//gett index of base_frame and update

			if(state->name[i].find("unit")!=std::string::npos)//getting the real positions of all obstacles in the simulation
			{

				//gazebo contains a unit box clyinder or sphere


				if(state->pose[i].position.z>0.01)
				{
						obstacle.x=state->pose[i].position.x;
						obstacle.y=state->pose[i].position.y+0.45;
						obstacles.push_back(obstacle);
				}

			}

		}
		truepose=state->pose[index];
		truepose.position.y+=0.45;

	}
	void odomCallback(const nav_msgs::OdometryConstPtr& odomptr)
		{
			odom=*odomptr;
			}


	};

class Avoidance
	{
	//class that holds analytics for avoidance maneuvers
	public:
		geometry_msgs::Point start,endpoint;
		double startdist,enddist;
		ros::Time stamp;
		ros::Duration duration;
		bool running;

		void begin(double dist)
		{
			start.x=odom.pose.pose.position.x;
			start.y=odom.pose.pose.position.y;
			stamp=ros::Time::now();
			//dist=0;
			ROS_INFO("Avoidance has begun");
			startdist=dist-0.5;
			running=true;
		}
		void end(double dist)
		{
			endpoint.x=odom.pose.pose.position.x;
			endpoint.y=odom.pose.pose.position.y;
			enddist=dist-0.5;
			//dist=sqrt(pow(endpoint.x-start.x,2)+pow(endpoint.y-start.y,2));
			duration=ros::Time::now()-stamp;
			ROS_INFO("Avoidance has been accomplished in %f [s], startet %f [m] away, ended %f [m] away,",duration.toSec(), startdist,enddist);//-0.5 since unit volumes
			running =false;


		}
	};

class Tester
	//this class incorporates test evaluation procedures for the arlo_navigation
	{
	private:
	Avoidance avoid;
	geometry_msgs::TransformStamped transformStamped;
	double pdiff;
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
		double evaldist=0.1;
		return atan2(polynomial(x+evaldist,line.polynomial.a)-polynomial(x,line.polynomial.a),evaldist);
	}

	public:



	std_msgs::Float64 lanediff(void)
	{
		//function tracks the difference between the robot and the right lane
		//approximates the distance by tf transforming the position if the lane can not be seen.
		std_msgs::Float64 diff;

		geometry_msgs::Point pt;

		tf2_ros::Buffer tfBuffer(ros::Duration(30.0));
		tf2_ros::TransformListener tfListener(tfBuffer);

		pt.x=oldodom.pose.pose.position.x;
		pt.y=oldodom.pose.pose.position.y;

		//getting x position in last lane by transforming the old odometry in to the latest transformation

		try{
			if(rightlane.points.size()>0){
			ros::Time now = ros::Time::now();
			transformStamped = tfBuffer.lookupTransform("base_footprint","odom",ros::Time(0),ros::Duration(1.0));
			tf2::doTransform(pt,pt, transformStamped);
			diff.data = abs(polynomial(-pt.x,rightlane.polynomial.a)*sin((M_PI/2)-angle(-pt.x,rightlane)));
			}

		}
		catch(tf2::TransformException &e)
		{
			ROS_INFO("%s",e.what());

		}
		pdiff=diff.data;
		return diff;
	}
		double diff;
		std_msgs::Int64 checklanediff()

		{
			double mindist=10;//minimal distance of robot to obstacles
			//rectifying the lane diff
			for(int i =0;i<obstacles.size();i++)
			{
				double dist=sqrt(pow(truepose.position.x-obstacles[i].x,2)+pow(truepose.position.y-obstacles[i].y,2));
				mindist=std::min(dist,mindist);
			}
			double width;

			if(road.laneWidthRight<0.5)width=0.925;
			else width=road.laneWidthRight;

			if(!offlanetrigger&&pdiff>(width-0.45)/2){
				offlanetrigger=true;
				offlanecounter.data++;
				avoid.begin(mindist);

			}
			else if(offlanetrigger&&pdiff<(width-0.45)/2)
			{
				offlanetrigger=false;
				if(avoid.running)
				{
					offlanecounter.data--;//resetting offlanecounter if it was an observation
					avoid.end(mindist);
				}
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
	ros::param::set("use_sim_time", true);
	ros::Publisher trueposepub =n.advertise<geometry_msgs::Pose>("/tester/truepose",1000);
	ros::Publisher diffpub =n.advertise<std_msgs::Float64>("/tester/lanediff",1000);
	ros::Publisher diffcounterpub =n.advertise<std_msgs::Int64>("/tester/offlanecounter",1000);
	//TODO maybe 3 different point clouds for each line of the track
	ros::Subscriber road = n.subscribe("/roadDetection/road", 1000, &Listener::roadCallback, &listener);
	ros::Subscriber state = n.subscribe("/gazebo/model_states", 1000, &Listener::linkCallback, &listener);
	ros::Subscriber odom = n.subscribe("/odom", 1000, &Listener::odomCallback, &listener);
	//TODO configure frequency
	ros::Rate rate(10);
	bool offlanetrigger;
	//copy smaller vector to larger and publish it as pointcloud2
	ros::Rate wait(0.5);

	wait.sleep();
	while(n.ok()){
		ros::spinOnce();
		trueposepub.publish(truepose);
		diffpub.publish(tester.lanediff());
		diffcounterpub.publish(tester.checklanediff());

		rate.sleep();
	}

  return 0;
}
