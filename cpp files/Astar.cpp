// when it ask you to enter starting and ending coordinates then put in reverse order of whatever your stage shows and neglect all negetive sign
//(eg. 17,-32)  then  enter (32,17) 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "stdlib.h"
#include "math.h"
#include <sstream>

#define PI 3.141593

typedef struct node
{
	int prob;		// probability whether cell is walkable or not.  if prob=0 then Yes(walkable) otherwise No
	int f_cost;
	int visited;		
}data;

typedef struct node1
{
	float i,j;
}PATH;

double z=0;
float covered_angle,old_covered_angle;

void Move(double distance)
{
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);	
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
void Rotate(double angle,int dir)
{
	if(angle<=0.004)
	return;
 	ros::NodeHandle n;
	ros::Rate loop_rate(10);	
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	covered_angle = 0.0;
       	float velocity ;
	while(angle - fabs(covered_angle) > 0.004)
	{
		int fq = 10;
		if(angle - fabs(covered_angle) > 0.08)		
			velocity = 0.2;
		else
			velocity = 0.01;
		
		geometry_msgs::Twist dis;
		if(dir == 0)
		dis.angular.z=velocity;	       
		if(dir == 1)
		dis.angular.z=-velocity;	       
		pub.publish(dis);
	  	ros::spinOnce();
		loop_rate.sleep();
	}
	old_covered_angle+=covered_angle;
}

void Covered_Angle(nav_msgs::Odometry od)
{
	//printf("z---> %f \n",2*asin(od.pose.pose.orientation.z));
	z = 2*asin(od.pose.pose.orientation.z);//orient = 2*asin(od.pose.pose.orientation.z);
	covered_angle = 2*asin(od.pose.pose.orientation.z) - old_covered_angle;
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


void Get_Path(int d[],int h,int w)
{
	//int h=40 , w=120;
	int i=0,j=0,k=0;
	int si,sj,di,dj;	
	int counter = 0,G_Cost = 0,min;

	data **grid = (data**)malloc(h*sizeof(data*)); 
	for (i=0;i<h;i++)
    	grid[i] = (data*)malloc(w*sizeof(data));

	//printf("\nCheck1\n");
	
	for(i=h-1;i>=0;i--)				// converting 1-D array into 2-D array-Grid map
	{
		for(j=0;j<w;j++)
		{
			grid[i][j].prob = d[k];	
			grid[i][j].visited = 0;    // initialize all values to 0
			grid[i][j].f_cost = 0;
			k++;
		}
	}
	//printf("\nCheck2\n");
	for(i=0;i<h;i++)                               // For printing the map(in form of 0 and 1) on console
	{
		for(j=0;j<w;j++)
		printf("%d",grid[i][j].prob);
		printf("\n");
	}
	printf("\n\n");

	printf("Enter Starting point's coordinate : ");
	scanf("%d %d",&si,&sj);
	
	if(grid[si][sj].prob == 1)
	{
		printf("\nWrong coordinate . . .!!!\nThere is obstacle.Robot can't be there\nPlease Try again.\n");
		exit(1);
	}

	printf("Enter End point's coordinate : ");
	scanf("%d %d",&di,&dj);

	if(grid[di][dj].prob == 1)
	{
		printf("\nWrong destination . . .!!!\nThere is obstacle.Robot can't move there\nPlease Try again.\n");
		exit(1);
	}

	PATH path[h*w];
	
	path[0].i = si;
	path[0].j = sj; 

	i=si,j=sj;
	grid[i][j].prob = 5;
	
	while(i!=di || j!=dj)
	{
		counter++;
		G_Cost++;
		min = 1000000;

		if(i-1>=0)
		{
			if(grid[i-1][j].prob==0 && grid[i-1][j].visited == 0)
			{
				grid[i-1][j].f_cost = G_Cost + H_Cost(i-1,j,di,dj);	
				if(grid[i-1][j].f_cost <= min)
				{
					min = grid[i-1][j].f_cost;
					path[counter].i = i-1;
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
			if(j+1<w)
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
		if(i+1<h)
		{
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
		        if(j+1<w)
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
		if(j+1<w)
		{
			
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
	
	
	for(i=0;i<h;i++)                               
	{
		for(j=0;j<w;j++)
		printf("%d",grid[i][j].prob);
		printf("\n");
	}
	
	for(i=0;i<h;i++)
	{
		data* currentPtr = grid[i];
    		free(currentPtr);
	}

	free(grid);
	
	//printf("%d , %d\n",grid[1][50].prob,grid[1][50].visited);
	printf("\n\n");
	
	printf("\nTotal No. of moves taken by robot are = %d\n\n",counter);
	
	//for(k=0;k<=counter;k++)
	//printf("(%d,%d)->",path[k].i,path[k].j);

	Motion(path,counter);

}
void GetData(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{      
	int i;

	//for(i=5*50;i<8*50;i++)
        //printf("%d ",msg->data[i]);
        
	//float r = msg->info.resolution; // Resolution of 1 pixel [eg. 0.1 meter] 

	int w = msg->info.width;      // Horizontal size of map (No. of columns)
	int h = msg->info.height;     // Vertical size of map (No. of Rows)
	
	int data[w*h];
	
	for(i=0;i<h*w;i++)			// Converting values into 0 and 1 only
	{
		if(msg->data[i] != 0)		// All values other than 0 converted into 1
		data[i] = 1;
		else
		data[i] = 0;
	}
	// Now my map has only two possibilities that is 0 and 1 , 0 means walkable and 1 means there is an obstacle .
	Get_Path(data,h,w);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "node");

	ros::NodeHandle n;

     ros::Subscriber path = n.subscribe("map",1000,GetData);
	//ros::Subscriber path1 = n.subscribe("odom",1000,Rot);
	ros::Subscriber path2 = n.subscribe("odom", 1000, Covered_Angle);
	ros::spin();
}

