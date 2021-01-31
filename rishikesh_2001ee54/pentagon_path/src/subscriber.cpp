#include "ros/ros.h"
#include "turtlesim/Pose.h"

void print_position(const turtlesim::Pose& msg)
{

	ROS_INFO_STREAM(std::setprecision(3) 
	<< "coordinates: "<<msg.x << ", " <<msg.y <<"\n"
	<<"direction: "<<msg.theta);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle node;
	
	ros::Subscriber sub = node.subscribe("/turtle1/pose", 1000, print_position);
	
	ros::spin();
	
	return 0;

}
