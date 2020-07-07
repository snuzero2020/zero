#include <vector>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cost_map_generator");
	ros::NodeHandle n;
	ros::Publisher cost_map_pub = n.advertise<nav_msgs::OccupancyGrid>("cost_map", 50000);
	ros::Rate loop_rate(10);
	nav_msgs::OccupancyGrid cost_map;
	cost_map.info.width = 200;
	cost_map.info.height = 200;
	for(int i = 0;i<200;i++){
		for(int j = 0;j<200;j++){
			if(i >=90 && i<=110) cost_map.data.push_back((int8_t)10);
			else cost_map.data.push_back((int8_t)100);
		}
	}
	while(ros::ok()){
		ROS_INFO("pub");
		cost_map_pub.publish(cost_map);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
