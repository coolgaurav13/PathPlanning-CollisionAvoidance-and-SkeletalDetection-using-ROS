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
#define m 50
#define n 35

int sectors;
int newmap[m+1][n+1];
int mynew_x=0,mynew_y=0;
float min_angle,max_angle,angle_increment,range[1080];

typedef struct node
{
	int prob;		// probability whether cell is walkable or not.  if prob=0 then Yes(walkable) otherwise No
	int f_cost;
	int visited;		
}data;

typedef struct node1
{
	int i,j;
}PATH;

double z=0;
float covered_angle,old_covered_angle;

double Abs(double a)
{
	if(a<0)
	return -a;
	else 
	return a;
}
void Move(double distance)
{
	ros::NodeHandle n1;
	ros::Publisher chatter_pub = n1.advertise<geometry_msgs::Twist>("cmd_vel", 1000);	
      	ros::Rate loop_rate(10);

	geometry_msgs::Twist msg;

	float count=1.2;
	while(count>0)
	{
		msg.linear.x = distance;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-0.1;
	}	
}
void Covered_Angle(nav_msgs::Odometry od)
{
	//printf("z---> %f \n",2*asin(od.pose.pose.orientation.z));
	z = 2*asin(od.pose.pose.orientation.z);
	covered_angle = 2*asin(od.pose.pose.orientation.z) - old_covered_angle;
}
void Rotate(double angle,int dir)
{
	double velocity ;
	covered_angle = 0;
	
	if(angle<=0.001)
	return;
 	ros::NodeHandle n;
	ros::Rate loop_rate(10);	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	while(angle - fabs(covered_angle) > 0.001)
	{
		if(angle - fabs(covered_angle) > 0.1)		
		velocity = 0.2;
		else
		velocity = 0.01;
		geometry_msgs::Twist msg;
		if(dir==0)
		msg.angular.z=velocity;	       
		else if(dir==1)
		msg.angular.z=-velocity;	       
		pub.publish(msg);
	  	ros::spinOnce();
		loop_rate.sleep();
	}
	old_covered_angle+=covered_angle;
	printf("{%f}\n",angle);
}
/*void Rot(nav_msgs::Odometry msg)
{
	//printf("<--%f-->\n",msg.pose.pose.orientation.z);
	z = 2*asin(msg.pose.pose.orientation.z);
	//printf("<--%f-->\n",z);
}
void Rotate(double angle)
{
 	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);

	geometry_msgs::Twist msg;

	float count=0.7;
	while(count>0)
	{
		msg.angular.z = angle;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count=count-0.1;
	}
}*/

void Motion(PATH path[],int size)
{
	int i=0;
	float current_x = path[0].i;
	float current_y = path[0].j;
	double angle,distance;

	for(i=1;i<=size;i++)
	{
		printf("(%f,%f)\n",path[i].i,path[i].j);
		if(path[i].j-current_y == -1)
		{	
			if(path[i].i-current_x == 0)
			{
				angle = PI;
				distance = 1;
			}
			else
			{
				if(path[i].i-current_x == -1)
				{
					angle = 3*(PI/4); 
					distance = 1.414214;
				}
				else
				{
					if(path[i].i-current_x == 1)
					{
						angle = -3*(PI/4);	
						distance = 1.414214;
					}
				}
			}
			angle = -(z-angle);
		}
		else if(path[i].j-current_y == 1)
		{
			if(path[i].i-current_x == 0)
			{
				angle = 0;
				distance = 1;
			}
			else 
			{
				if(path[i].i-current_x == -1)
				{
					angle = -(PI/4); 
					distance = 1.414214;
				}
				else 
				{
					if(path[i].i-current_x == 1)
					{
						angle = (PI/4);	
						distance = 1.414214;
					}
				}
			}

			angle = -(angle+z);

		}
		else if(path[i].j-current_y == 0)
		{
			if(path[i].i-current_x == -1)
			{
				angle = ((PI/2)-z); 
				distance = 1;
			}
			else
			{			
				if(path[i].i-current_x == 1)
				{
					angle = -((PI/2)+z);	
					distance = 1;
				}
			}
		}
		printf("%f,%f\n",angle,distance);
		if(angle<0)
		Rotate(-angle,1);
		else
		Rotate(angle,0);
		//printf("(%f,%f)\n",path[i].i,path[i].j);		
		//printf("{%f}\n",angle);
		//Rotate(angle,d);
		Move(distance);

		current_x = path[i].i;
		current_y = path[i].j;
	}
}

float H_Cost(int current_i,int current_j,float destination_i,float destination_j)
{
	float a = (float)current_i - destination_i;
	if(a<0)
	a=(-1)*a;

	float b = (float)current_j - destination_j;
	if(b<0)
	b=(-1)*b;

	return (a+b);  
}

void Get_Path(int map[m+1][n+1],int si,int sj,int di,int dj)
{
	//int m=40 , n=120;
	int i=0,j=0,k,l;
	//int si,sj,di,dj;	
	int counter = 0,G_Cost = 0,min;

	data **grid = (data**)malloc(m*sizeof(data*)); 
	for (i=0;i<m;i++)
    	grid[i] = (data*)malloc(n*sizeof(data));

	//printf("\nCheck1\n");

	
	
	for(i=0;i<m;i++)                               // For printing the map(in form of 0 and 1) on console
	{
		for(j=0;j<n;j++)
		grid[i][j].prob = map[i+1][j+1];
	}
	printf("\n\n");
	
	for(i=m-1;i>=0;i--)                     // For printing the map(in form of 0 and 1) on console
	{
		for(j=0;j<n;j++)
		printf("%d",grid[i][j].prob);
		printf("\n");
	}
	//printf("Enter Starting point's coordinate : ");
	//scanf("%d %d",&si,&sj);

	if(grid[si][sj].prob == 1)
	{
		printf("\nWrong coordinate . . .!!!\nThere is obstacle.Robot can't be there\nPlease Try again.\n");
		exit(1);
	}

	//printf("Enter End point's coordinate : ");
	//scanf("%d %d",&di,&dj);

	if(grid[di][dj].prob == 1)
	{
		printf("\nWrong destination . . .!!!\nThere is obstacle.Robot can't move there\nPlease Try again.\n");
		exit(1);
	}

	PATH path[m*n];

	path[0].i = si;
	path[0].j = sj; 
	printf("<<%d,%d>>\n",si,sj);
	i=si,j=sj;
	grid[i][j].prob = 5;
	printf("\nCheck2\n");
	//di=m-1-di;
	//dj=dj-1;
	while(i!=di || j!=dj)
	{
		/*for(k=m-1;k>=0;k--)                     // For printing the map(in form of 0 and 1) on console
		{
			for(l=0;l<n;l++)
			printf("%d",grid[k][l].prob);
			printf("\n");
		}
		printf("\n");*/
		counter++;
		G_Cost++;
		min = 1000000;
		//j = j-1;
		
		if((i-1)>=0)
		{
			//i = m+1-i;
			if(grid[i-1][j].prob==0 && grid[(i-1)][j].visited == 0)
			{
				grid[i-1][j].f_cost = G_Cost + H_Cost(i-1,j,di,dj);	
				if(grid[i-1][j].f_cost <= min)
				{
					min = grid[(i-1)][j].f_cost;
					path[counter].i = (i-1);
					path[counter].j = j; 
				}	
			}
			if(j-1>=0)
			{
				if(grid[i-1][j-1].prob==0 && grid[i-1][j-1].visited == 0)
				{
					grid[i-1][j-1].f_cost = G_Cost + H_Cost(i-1,j-1,di,dj);
					if(grid[i-1][j-1].f_cost <= min)
					{
						min = grid[i-1][j-1].f_cost;
						path[counter].i = i-1;
						path[counter].j = j-1; 
					}
				}
			}
			if(j+1<n)
			{
				if(grid[i-1][j+1].prob==0 && grid[i-1][j+1].visited == 0)
				{	
					grid[i-1][j+1].f_cost = G_Cost + H_Cost(i-1,j+1,di,dj);
					if(grid[i-1][j+1].f_cost <= min)
					{
						min = grid[i-1][j+1].f_cost;
						path[counter].i = i-1;
						path[counter].j = j+1; 
					}		
				}
			}
		}
		if(i+1<m)
		{
			//i = m-i-3;
			if(grid[i+1][j].prob==0 && grid[i+1][j].visited == 0)
			{			
				grid[i+1][j].f_cost = G_Cost + H_Cost(i+1,j,di,dj);	
				if(grid[i+1][j].f_cost <= min)
				{
					min = grid[i+1][j].f_cost;
					path[counter].i = i+1;
					path[counter].j = j; 
				}			
			}
			if(j-1>=0)
     			{
				if(grid[i+1][j-1].prob==0 && grid[i+1][j-1].visited == 0)
				{
					grid[i+1][j-1].f_cost = G_Cost + H_Cost(i+1,j-1,di,dj);	
					if(grid[i+1][j-1].f_cost <= min)
					{
						min = grid[i+1][j-1].f_cost;
						path[counter].i = i+1;
						path[counter].j = j-1; 
					}		
				}
		   	}
			if(j+1<n)
		       	{
				if(grid[i+1][j+1].prob==0 && grid[i+1][j+1].visited == 0)
				{			
					grid[i+1][j+1].f_cost = G_Cost + H_Cost(i+1,j+1,di,dj);	
					if(grid[i+1][j+1].f_cost <= min)
					{
						min = grid[i+1][j+1].f_cost;
						path[counter].i = i+1;
						path[counter].j = j+1; 
					}		
				}
			}
		}
		if(j-1>=0)
		{
			//i = m-1-i;
			if(grid[i][j-1].prob==0 && grid[i][j-1].visited == 0)
			{
				grid[i][j-1].f_cost = G_Cost + H_Cost(i,j-1,di,dj);	
				if(grid[i][j-1].f_cost <= min)
				{
					min = grid[i][j-1].f_cost;
					path[counter].i = i;
					path[counter].j = j-1; 
				}					
			}
		}
		if(j+1<n)
		{
			//i = m-1-i;
			if(grid[i][j+1].prob==0 && grid[i][j+1].visited == 0)
			{
				grid[i][j+1].f_cost = G_Cost+ H_Cost(i,j+1,di,dj);	
				if(grid[i][j+1].f_cost <= min)
				{
					min = grid[i][j+1].f_cost;
					path[counter].i = i;
					path[counter].j = j+1; 
				}	
			}
		}

		i = path[counter].i;
		j = path[counter].j;
	
		grid[i][j].visited = 1;
		grid[i][j].prob = 7;

	} // End of While Loop


	for(i=0;i<m;i++)                               
	{
		//for(j=0;j<n;j++)
		//printf("%d",grid[i][j].prob);
		//printf("\n");
	}

	printf("\nTotal No. of moves taken by robot are = %d\n\n",counter);

	for(k=0;k<=counter;k++)
	printf("(%d,%d)->",path[k].i,path[k].j);

	Motion(path,counter);

}
void Astar(int map[m+1][n+1], int si, int sj, int di, int dj)
{
	/*si = Abs(double(si));
	sj = Abs(double(sj));
	di = Abs(double(di));
	dj = Abs(double(dj));*/
	printf("<si=%d,sj=%d,di=%d,dj=%d>\n",si,sj,di,dj);
	Get_Path(map,si,sj,di,dj);
}
void Laser_Data(sensor_msgs::LaserScan msg)
{
	int i;
	min_angle = msg.angle_min;
	max_angle = msg.angle_max;
	angle_increment = msg.angle_increment;

	sectors = (int)((max_angle - min_angle)/angle_increment); 

	for(i=0;i<sectors;i++)
	{
		range[i] = msg.ranges[i];
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node");
	ros::NodeHandle nod;
	ros::Rate loop_rate(10);
	ros::Subscriber sub = nod.subscribe("base_scan", 1000, Laser_Data);
	
	int i,j,laser_index=0,count=0,flag1=0;
	int start_x,start_y,end_x,end_y,des_x,des_y;
	double angle=0;
	
	for(i=1;i<=m;i++)
	for(j=1;j<=n;j++)
	newmap[i][j] = 5;

	start_x = 17;
	start_y = -14;
	float g;
	while(count<1)
	{

		for(i=m;i>=1;i--)
		{
			for(j=1;j<=n;j++)
			printf("%d",newmap[i][j]);
			printf("\n");
		}
		//printf("Please Enter the starting coordinate of robot(x,y) : ");
		//scanf("%d %d",&start_x,&start_y); // e.g. (17,-32)
		
		newmap[m+start_y][start_x] = 2;
		
		loop_rate.sleep();
		ros::spinOnce();

		laser_index = 0;
		//printf("Check\n");
		while(laser_index<sectors)
		{
			if(range[laser_index] < 30.0)
			{
				if(laser_index < (sectors/2))
				angle = (-1)*((sectors/2)-laser_index)*angle_increment;	
				else if(laser_index > (sectors/2))
				angle = (laser_index-(sectors/2))*angle_increment;	
				else
				angle = 0;	
			}

			end_x = start_x + range[laser_index]*cos(angle);
			end_y = start_y + range[laser_index]*sin(angle);
			newmap[m+end_y][end_x] = 1;
			for(g=0.5;g<range[laser_index]-0.5;g=g+0.5)
			{
				//printf("%f ,",g);
				end_x = start_x + g*cos(angle);
				end_y = start_y + g*sin(angle);
				if(newmap[m+end_y][end_x] == 1)
				continue;
				else
				newmap[m+end_y][end_x] = 0;
			}
			//printf("Check1\n");
			laser_index++;
		} // End of 1st Inner While Loop

		printf("\n\nNew Map #%d\n\n",count+1);
		for(i=m;i>=1;i--)
		{
			for(j=1;j<=n;j++)
			printf("%d",newmap[i][j]);
			printf("\n");
		}

		laser_index = 1;
		flag1 = 0;

		//select frontier where we want to move
		
		for(i=2;i<m;i++)
		{
			for(j=2;j<n;j++)
			{	
				if(newmap[i][j] == 0 && flag1!=1)
				{
					if(newmap[i-1][j-1] == 5 || newmap[i-1][j] == 5 || newmap[i-1][j+1] == 5)
					{	
						des_x = i;
						des_y = j;
						flag1 = 1;
					}
					else if(newmap[i][j-1] == 5 || newmap[i][j+1] == 5)
					{	
						des_x = i;
						des_y = j;
						flag1 = 1;
					}
					else if(newmap[i+1][j-1] == 5 || newmap[i+1][j] == 5 || newmap[i+1][j+1] == 5)
					{	
						des_x = i;
						des_y = j;
						flag1 = 1;
					}
					//if(newmap[des_x-1][des_y-1]==1 || )
				}
			}
		}
		printf("\n1.{%d,%d}\n",des_x,des_y);
		newmap[des_x][des_y] = 7;
		
		for(i=m;i>=1;i--)
		{
			for(j=1;j<=n;j++)
			printf("%d",newmap[i][j]);
			printf("\n");
		}
		
		Astar(newmap,start_x,start_y,des_x,des_y);

		start_x = des_y;
		start_y = -des_x;
		//printf("\n2.{%d,%d}\n",start_x,start_y);

		count++;
	} // End of Outer While Loop*/
}

