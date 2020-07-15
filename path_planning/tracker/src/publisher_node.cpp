#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

class Publisher
{
	private:
		ros::NodeHandle nh;

		// configuration constants
		double current_vel{0};

	public:
		
		Path curr_local_path;
		Odometry curr_odom;

		//ros::Publisher local_path_pub;
		ros::Publisher odometry_pub;
		ros::Publisher recommend_vel_pub;

		Float32 recommend_vel;

		// initializer
		Publisher() 
			:curr_local_path(Path()), curr_odom(Odometry())
			{
				//local_path_pub = nh.advertise<Path>("local_path",100);
				odometry_pub = nh.advertise<Odometry>("odometory",100);
				recommend_vel_pub = nh.advertise<Float32>("recommend_vel",100);
			}

};


int main(int argc, char *argv[])
{
	double vel_x;
	double vel_y;

	ros::init(argc,argv,"publisher_temp");

	cout << "aloha1" << endl;
	Publisher publisher{Publisher()};
	cout << "aloha2" << endl;
	//ros::spin();
	ros::Rate loop_rate(10);
	cout << "aloha3" << endl;

	PoseStamped pose;
	for (int i{5}; i<100; i+=5)
	{
		pose.header.seq = i/5;
		pose.pose.position.x = i;
		pose.pose.position.y = 200*sin(i/double(200)*3.141592);
		publisher.curr_local_path.poses.push_back(pose);
	}
	pose.header.seq = 0;
	publisher.curr_local_path.poses.push_back(pose);

	int count{0};
	while (ros::ok())
	{
		cout << "hello" << endl;
		if (vel_x>14)
		{
			vel_x = 0;
		}
		if (vel_y>14)
		{
			vel_y = 0;
		}
		publisher.curr_odom.twist.twist.linear.x = vel_x;
		publisher.curr_odom.twist.twist.linear.y = vel_y;
		//publisher.local_path_pub.publish(publisher.curr_local_path);
		publisher.odometry_pub.publish(publisher.curr_odom);

		publisher.recommend_vel.data = 0.5;
		publisher.recommend_vel_pub.publish(publisher.recommend_vel);
		loop_rate.sleep();
		vel_x += 1;
		vel_y += 2;
		count++;
		cout << count << endl;
	}

	return 0;
}
