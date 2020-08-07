#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
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
		double time{0.0};
		double start_time{0.0};

		ros::Publisher local_path_pub;
		ros::Publisher odometry_pub;
		ros::Publisher recommend_vel_pub;
		ros::Subscriber time_sub_vel_sub;

		Float32 recommend_vel;

		// initializer
		Publisher() 
			:curr_local_path(Path()), curr_odom(Odometry())
			{
				//local_path_pub = nh.advertise<Path>("local_path",100);
				//odometry_pub = nh.advertise<Odometry>("odometory",100);
				recommend_vel_pub = nh.advertise<Float32>("recommend_vel",100);
				time_sub_vel_sub = nh.subscribe("start_time",100, &Publisher::time_callback, this);
			}

		void time_callback(const std_msgs::Float64::ConstPtr);

};

void Publisher::time_callback(const std_msgs::Float64::ConstPtr msg){
    start_time = msg->data;
}


int main(int argc, char *argv[])
{
	double vel1{1.0};
	double vel2{0.5};
	int num{0};

	ros::init(argc,argv,"pub_vel");
	cout << "aloha1" << endl;
	Publisher publisher{Publisher()};
	cout << "aloha2" << endl;
	ros::Rate loop_rate(10);
	cout << "aloha3" << endl;

	int count{0};
	while (ros::ok())
	{
		ros::NodeHandle nh;

		// if (publisher.start_time > 0){
			
		// 	//nh.getParam("/vel1", vel1);
		// 	publisher.recommend_vel.data = vel2;
		// 	cout << vel1 << endl;
		// }

		// else{
		// 	//nh.getParam("/vel2", vel2);
		// 	publisher.recommend_vel.data = vel1;
		// 	cout << vel2 << endl;
		// }
		if(num>50)
		{
			publisher.recommend_vel.data = vel1 - (vel1-vel2)*2*count/100.0;
			cout<< publisher.recommend_vel << endl;
			
			if(publisher.recommend_vel.data < vel2)
			{
				publisher.recommend_vel.data = vel2;
			}
			else
			{
				count++;
			}
		}
		else
		{

			publisher.recommend_vel.data = vel1;
			cout<< publisher.recommend_vel << endl;
		}			

		publisher.recommend_vel_pub.publish(publisher.recommend_vel);
		loop_rate.sleep();
		ros::spinOnce();
		num++;
	}

	return 0;
}
