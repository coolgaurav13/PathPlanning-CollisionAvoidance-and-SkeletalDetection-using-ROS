#include "ros/ros.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"	
#include "stdlib.h"

double angle_increment,ranges[]={0};
int sectors;

void Motion(double distance,double angle)  // For angular motion
{
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);
	
	geometry_msgs::Twist msg;
	
	float count=2.2;
	while(count>0)
	{
		msg.angular.z = angle;
		msg.linear.x = distance;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-0.2;
	}

}

void Laser_Data(sensor_msgs::LaserScan msg)
{
	int i;
	double min_angle = msg.angle_min;
	double max_angle = msg.angle_max;
	
	angle_increment = msg.angle_increment;
	sectors = (int)((max_angle-min_angle)/angle_increment) ; 

	for(i=0;i<sectors;i++)
	ranges[i] = msg.ranges[i];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"node");     
	ros::NodeHandle n;	
	ros::Rate loop_rate(10);	
	ros::Subscriber sub = n.subscribe("base_scan", 1000, Laser_Data);
	ros::spin();
	
	double d=5,angle,distance;
	int i,cons=30,choice1=0,choice2=0,final_choice=0,flag=0;
	
	while(1)
	{
		choice1=choice2=final_choice=0 ;
	 	flag=0;
		for(i=(sectors/2)-cons;i<((sectors/2)+cons);i++)
		{
			if(ranges[i]>d)
			continue
			else
			flag=1;
			if(flag==1)
			break;
		}
		if(flag==0)
		{
			distance = 2.0; 
			angle = 0.0;// Move straight
		}
		else
		{
			flag = 0;             // Check left side
			for(i=(sectors/2)+cons;i<sectors;i++)
			{
				if(ranges[i]>d)
				{
					choice1 = i;				
					flag=1;
				}
				if(flag==1)
				break;
			}
			
			flag = 0;			 //Check right side
			for(i=((sectors/2)-cons);i>=0;i--)
			{
				if(ranges[i]>d)
				{
					choice2 = i;	
					flag=1;	
				}	
				if(flag==1)
				break;					
			}	
			
			if(((choice1-(sectors/2)<=(sectors/2)-choice2))) // select smallest angle
			final_choice = choice1;
			else if(((choice1-(sectors/2)>(sectors/2)-choice2)))
			final_choice = choice2;
			else 							// If angles are same then neglect smallest length sector 
			{
				if(ranges[choice1]>ranges[choice2])
				final_choice = choice1;
				else if(ranges[choice1]<ranges[choice2])
				final_choice = choice2;
				else
				final_choice = choice2;
			}
			distance = ranges[final_choice]/5;
			angle = (final_choice-(sectors/2))*angle_increment ;	
		}
		Motion(distance,angle);	
		loop_rate.sleep();
	} // End of While
}













