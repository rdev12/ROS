#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>

const double PI = 3.14159265358979323846;


//Helper Methods
void move(double speed, double distance, bool isForward, 
ros::Publisher velocity_publisher);
void rotate(double a_speed, double a_distance, bool isCC, ros::Publisher velocity_publisher);

int main(int argc, char **argv)
{
	ros::init(argc,argv, "publisher");
	ros::NodeHandle n;
	
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Rate loop_rate(1);
	int count = 0;
	
	
	while (ros::ok() && count < 5)
	{
	
		move (0.2, 2.0, true, velocity_publisher);
		rotate (0.2, 2*PI/5, true, velocity_publisher);
		count++;
		
		loop_rate.sleep();
	
	}

	ros::spin();
	
	return 0;
} 

void move(double speed, double distance, bool isForward, ros::Publisher velocity_publisher)
{
	geometry_msgs::Twist vel_msg;
	
	if (isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);

	
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	
			
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;		
	vel_msg.angular.z = 0;
	
	double initTime = ros::Time::now().toSec();
	double covered  = 0;
	
	
	ros::Rate loop_rate(1000);
		
	do
	{
		velocity_publisher.publish(vel_msg);
		
		double timeNow = ros::Time::now().toSec();
		covered = vel_msg.linear.x * (timeNow - initTime);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	while (covered < distance);
	
	//Reset
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}

void rotate(double a_speed, double a_distance, bool isCC, ros::Publisher velocity_publisher)
{
	geometry_msgs::Twist vel_msg;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	
	if (isCC)
		vel_msg.angular.z = abs(a_speed);
	else
		vel_msg.angular.z = -abs(a_speed);
		
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;		

	
	double initTime = ros::Time::now().toSec();
	double covered  = 0.0;
	ros::Rate loop_rate(1000);
		
	do
	{
		velocity_publisher.publish(vel_msg);
		
		double timeNow = ros::Time::now().toSec();
		covered = vel_msg.angular.z * (timeNow - initTime);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	while (covered < a_distance);
	
	//Reset
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}
