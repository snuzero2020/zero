#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void pathCallback(const nav_msgs::Path & path)
{

	static int cnt = 0;
	ROS_INFO("%d : I get path", cnt++);
	for(geometry_msgs::PoseStamped poseStamp : path.poses){
		if(poseStamp.header.seq == 0) break;
		ROS_INFO("%lf %lf",poseStamp.pose.position.x, poseStamp.pose.position.y);
	}
	ROS_INFO("End");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_listener");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("local_path", 1000, pathCallback);
	ros::spin();
	return 0;
}
