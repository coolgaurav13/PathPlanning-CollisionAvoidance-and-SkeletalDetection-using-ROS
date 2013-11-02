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
typedef struct node
{
	double prob;		
	int walkable;	// whether cell is walkable or not.  if walkable=0 then Yes(walkable) otherwise No
}data;

int m,n;
double **z;

double update(int i_of_next_grid, int j_of_next_grid, int des_x, int des_y)
{
		if(i_of_next_grid>=des_x-1 && i_of_next_grid<=des_x+1 && j_of_next_grid>=des_y-1 && j_of_next_grid<=des_y+1)
		return (1/9);
		else
		return 0.000000;
}

data **Motion_Update(data **init_grid)
{
	int a,b,i,j,k,l,center_x,center_y,des_x,des_y;

	data **next_grid = (data**)malloc(m*sizeof(data*)); 
	for (i=0;i<m;i++)
    	next_grid[i] = (data*)malloc(n*sizeof(data));

	double r,v=1,omega=0,delta_t=1,delta_theta=0,theta1=0,theta2=0;
	
	init_grid[0][0].prob = 1.0;

	for(k=0;k<m;k++)
	{
		for(l=0;l<n;l++)
		{
			for(a=0;a<m;a++)
			{
				for(b=0;b<n;b++)
				{
					next_grid[a][b].prob = 0.0;
					next_grid[a][b].walkable = init_grid[a][b].walkable;
				}
			}

			for(i=0;i<m;i++)
			{
				for(j=0;j<n;j++)
				{
					if(omega==0)
					{
						des_x = j + v*delta_t;
						des_y = i + v*delta_t;
					}
					else
					{
						r = v/omega;
						delta_theta = omega*delta_t;
						theta2 = theta1 + delta_theta;
						center_x = j+(r*cos((PI/2)+theta1)+0.5);
						center_y = i-(r*sin((PI/2)+theta1)+0.5);
						des_x = center_y-(r*sin((PI/2)-theta2)+0.5);
						des_y = center_x+(r*cos((PI/2)-theta2)+0.5);
						theta1 = theta2;
					}
					if(des_x>=0 && des_x<m && des_y>=0 && des_y<n && init_grid[des_x][des_y].walkable==0)			
					{
						for(a=0;a<m;a++)
						{			
							for(b=0;b<n;b++)
							next_grid[a][b].prob = next_grid[a][b].prob+init_grid[i][j].prob*update(a,b,des_x,des_y);
						}
					}
						
				} //End of j's Loop
			} ////End of i's Loop	
			
			for(i=0;i<m;i++)
			{
				for(j=0;j<n;j++)
				printf("%.6f", next_grid[i][j].prob);
				printf("\n");
			}
			printf("\n\n");
	
	
			for(a=0;a<m;a++)
			{
				for(b=0;b<n;b++)
				{
					init_grid[a][b].prob=next_grid[a][b].prob;
					init_grid[a][b].walkable=next_grid[a][b].walkable;
				}
			}

		} //End of l's Loop
	} //End of k's Loop
	return next_grid;
}
double prob(int i,int j)
{
	int i,j,r,r_cap;
	double p=0,z_cap[1080];
	r_cap=2,r=1;
	double sum;
	for(i=0;i<1080;i++)
	{
			//
	}

	p = exp(-0.5*sum);
	return p;
}
void Measure(data **bel_cap)
{
	int i,j,k,laser_index=0,count=0;
	m=50,n=35;
	double bel[m][n],p[m][n];
	
	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			bel[i][j] = prob(i,j)*bel_cap[i][j].prob;
		}
	}
}
void GetData(nav_msgs::OccupancyGrid msg)
{
	m = msg.info.height;
	n = msg.info.width;
	
	int d[m*n];
	int i,j,k=0;
	for(i=0;i<m*n;i++)			// Converting values into 0 and 1 only
	{
		if(msg.data[i] != 0)		// All values other than 0 converted into 1
		d[i] = 1;
		else
		d[i] = 0;
	}

	data **grid = (data**)malloc(m*sizeof(data*)); 
	for (i=0;i<m;i++)
    	grid[i] = (data*)malloc(n*sizeof(data));

	
	for(i=0;i<m;i++)				// converting 1-D array into 2-D array-Grid map
	{
		for(j=0;j<n;j++)
		{
			grid[i][j].prob = 0;	
			grid[i][j].walkable = d[k];
			k++;
		}
	}
	grid = Motion_Update(grid);
	Measure(grid);
	
}

void Laser_Data(sensor_msgs::LaserScan msg)
{
	int i;
	min_angle = msg.angle_min;
	max_angle = msg.angle_max;
	angle_increment = msg.angle_increment;

	sectors = (int)((max_angle - min_angle)/angle_increment); 
	//printf("\nno of sector  :  %d \n\n",sectors);
	if(z==NULL)
	{
			z = (double **)malloc(sizeof(double *)*sectors);	
			for(i=0;i<sectors;i++)
			{
					z[i] = (double *)malloc(sizeof(double)*2);
			}
	}
	for(i=0;i<sectors;i++)
	{
		range[i] = msg.ranges[i];
		z[i][0] = range[i];
		z[i][1] = min_angle+i*angle_increment;
	}
	//printf("Check1\n");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node");
	ros::NodeHandle nod;
	ros::Rate loop_rate(10);
	ros::Subscriber sub = nod.subscribe("base_scan", 1000, Laser_Data);
	ros::Subscriber sub1 = nod.subscribe("map",1000,GetData);
	
	ros::spin();
}

