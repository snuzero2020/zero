#include <vector>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cost_map_generator");
	ros::NodeHandle n;
	ros::Publisher cost_map_pub = n.advertise<nav_msgs::OccupancyGrid>("cost_map_with_goal_vector",10);
	ros::Publisher goals_pub = n.advertise<nav_msgs::Path>("goals", 10);
	ros::Rate loop_rate(10);

	nav_msgs::OccupancyGrid cost_map;
	/*
	   cost_map.info.width = 200;
	   cost_map.info.height = 200;
	   int tmp = 10;//(int)(((double)rand()/RAND_MAX)*100);
	   for(int i = 0;i<200;i++){
	   for(int j = 0;j<200;j++){
	   if(i >=90 && i<=110 && j>=90 && j<= 110) cost_map.data.push_back((int8_t)tmp);
	   else cost_map.data.push_back((int8_t)10);
	   }
	   }
	   cost_map.data.push_back(50);
	   cost_map.data.push_back(98);
	   cost_map.data.push_back(0);
	   ROS_INFO("pub");
	   cost_map_pub.publish(cost_map);
	   ros::spinOnce();
	   loop_rate.sleep();
	   }
	 */
	cost_map.info.width = 300;
	cost_map.info.height = 300;
	int tmp = 10;
	for(int i = 0;i<300;i++){
		for(int j = 0;j<300;j++){
			if(i >=100 && i<=200) 
			{
				if(i >=140 && i<=160 && j >=140 && j<=180)
				{
					//cost_map.data.push_back((int8_t)50);
					cost_map.data.push_back((int8_t)tmp);
				}
				else
					cost_map.data.push_back((int8_t)tmp);
			}
			else cost_map.data.push_back((int8_t)100);
		}
	}

while(ros::ok()){
	nav_msgs::Path goals;
	geometry_msgs::PoseStamped pose;
	pose.pose.position.y=50;
	pose.pose.position.x=0;
	goals.poses.push_back(pose);
	pose.pose.position.y=100;
	pose.pose.position.x=0;
	goals.poses.push_back(pose);
	pose.pose.position.y=200;
	pose.pose.position.x=0;
	goals.poses.push_back(pose);
	goals_pub.publish(goals);

	ROS_INFO("pub");
	cost_map_pub.publish(cost_map);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;
}
