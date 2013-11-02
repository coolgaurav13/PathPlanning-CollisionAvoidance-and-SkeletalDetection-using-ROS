#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#define PI 3.141593

void Move(double distance)
{
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);	
      	ros::Rate loop_rate(10);

        geometry_msgs::Twist msg;
	
	float count=2.2;
	while(count>0)
	{
		msg.linear.x = distance;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-0.2;
	}	
}

void Rotate(double angle)
{
 	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);
	
	geometry_msgs::Twist msg;
	
	float count=2.2;
	while(count>0)
	{
		msg.angular.z = angle;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-0.2;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");  
	ros::NodeHandle n;
	
    Move(1.414);		// try case 1
	Rotate(PI/2);
	
    Move(2.0);		// try case 2
	Rotate(PI/3);
}

