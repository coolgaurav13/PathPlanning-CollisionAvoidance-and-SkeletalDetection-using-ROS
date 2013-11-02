#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

#define PRINTBOUNDARY printf("|_______________________|_______________________|\n");
#define PRINTLINE printf("_________________________________________________\n");

void Laser_Data(sensor_msgs::LaserScan msg)
{
	PRINTLINE;
	printf("|\tType\t\t|\tValue\t\t|\n");
	PRINTBOUNDARY;

	printf("|\tangle_min\t|\t%f\t|\n",msg.angle_min);
	printf("|\tangle_max\t|\t%f\t|\n",msg.angle_max);
	printf("|\tangle_increment |\t%f\t|\n",msg.angle_increment);

	printf("|\tscan_time\t|\t%f\t|\n",msg.scan_time);
	printf("|\ttime_increment  |\t%f\t|\n",msg.time_increment);

	printf("|\trange_min\t|\t%f\t|\n",msg.range_min);
	printf("|\trange_max\t|\t%f\t|\n",msg.range_max);
	PRINTBOUNDARY;
	
	printf("\n\t\t\t<--Ranges-->\n\n");
	for(int i=0;i<20; i++)
	printf(" %f, ",msg.ranges[i]);
		
	printf("\n\n\t\t\t<--Intensities-->\n\n");
	for(int i=0;i<20; i++)
	printf(" %f, ",msg.intensities[i]);

	printf("\n\n");	
}
    
int main(int argc, char **argv)
{
	ros::init(argc, argv,"laser_data");     
	ros::NodeHandle n;	

	ros::Subscriber sub = n.subscribe("base_scan", 1000, Laser_Data);
	ros::spin();
}



