// just put starting coordinates as same as your stage shows in 4th quadrant.

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "stdlib.h"
#include "math.h"
#include <sstream>

#define PI 3.141593

int sectors;
float min_angle,max_angle,angle_increment,range[1080];

void Laser_Data(sensor_msgs::LaserScan msg)
{
	int i;
	min_angle = msg.angle_min;
	max_angle = msg.angle_max;
	angle_increment = msg.angle_increment;

	sectors = (int)((max_angle - min_angle)/angle_increment); 
	//printf("\nno of sector  :  %d \n\n",sectors);

	for(i=0;i<sectors;i++)
	{
		range[i] = msg.ranges[i];
	}
	//printf("Check1\n");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node");
	ros::NodeHandle nod;
	ros::Rate loop_rate(10);
	ros::Subscriber sub = nod.subscribe("base_scan", 1000, Laser_Data);
	
	int i,j,k,laser_index=0,m=50,n=35,count=0;
	int newmap[m+1][n+1];
	int start_x,start_y,end_x,end_y;
	double angle=0;
	
	for(i=1;i<=m;i++)
	{
		for(j=1;j<=n;j++)
		newmap[i][j] = 0;
	}

	
	/*printf("\n");
	for(i=1;i<=m;i++)
	{
		for(j=1;j<=n;j++)
		printf("%d",newmap[i][j]);
		printf("\n");
	}*/
	
	//printf("%d\n",count);

	while(count<5)
	{
		printf("Please Enter the starting coordinate of robot(x,y) : ");
		scanf("%d %d",&start_x,&start_y); // e.g. (17,-32)
		
		newmap[m+start_y][start_x] = 2;
		
		loop_rate.sleep();
		ros::spinOnce();

		//printf("%f , %f\n",range[0],range[1079]);
		laser_index = 0;

		while(laser_index<sectors)//for(laser_index=sectors/2+15;laser_index<=sectors/2+15;laser_index++)//
		{
			//printf("%f\n",range[laser_index]);
			if(range[laser_index] < 30.0)
			{
				if(laser_index < (sectors/2))
				{
					angle = (-1)*((sectors/2)-laser_index)*angle_increment;	
					//printf("%f, %f\n",range[laser_index],angle);
					//printf("[%d, %d]\n", end_x, end_y);
				}
				else{ 
					if(laser_index > (sectors/2))
					{
						angle = (laser_index-(sectors/2))*angle_increment;	
						//printf("%f, %f\n",range[laser_index],angle);
						//printf("[%d, %d]\n", end_x, end_y);
					}
					else
					{
						angle = 0;	
						//printf("%f, %f\n",range[laser_index],angle);					
						//printf("[%d, %d]\n", end_x, end_y);
					}
				}
				end_x = start_x + range[laser_index]*cos(angle);
				end_y = start_y + range[laser_index]*sin(angle);
//				end_y = 50 - end_y;
				//printf("\n(%d, %d)\n",end_x,end_y);
				newmap[m+end_y][end_x] = 1;
			}
			
			laser_index++;
		} // End of Inner While Loop
	
		printf("\n\nNew Map #%d\n\n",count+1);
		for(i=m;i>=1;i--)
		{
			for(j=1;j<=n;j++)
			{
				printf("%d",newmap[i][j]);
			}
			printf("\n");
		}
		count++;
	} // End of Outer While Loop*/
}

