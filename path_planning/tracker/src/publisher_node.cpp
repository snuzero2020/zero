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

		ros::Publisher local_path_pub;
		ros::Publisher odometry_pub;
		ros::Publisher recommend_vel_pub;

		Float32 recommend_vel;

		// initializer
		Publisher() 
			:curr_local_path(Path()), curr_odom(Odometry())
			{
				local_path_pub = nh.advertise<Path>("local_path",100);
				odometry_pub = nh.advertise<Odometry>("odometory",100);
				recommend_vel_pub = nh.advertise<Float32>("recommend_vel",100);
			}

};


int main(int argc, char *argv[])
{
	double vel_x{5.0};
	double vel_y{5.0};
	double radius_test;
	double r = 5.0/0.03;
	double i = 0.03;
	double di = 3.0/r;
	double x = 0;
	double y = 0;
	int seq = 1;
	ros::init(argc,argv,"publisher_temp");

	cout << "aloha1" << endl;
	Publisher publisher{Publisher()};
	cout << "aloha2" << endl;
	//ros::spin();
	ros::Rate loop_rate(10);
	cout << "aloha3" << endl;

	PoseStamped pose;
	while(1)
	{
		x= r- r*cos(i) - (r- sqrt(r*r -1.05*100/3.0*1.05*100/3.0));
		y= r*sin(i)-1.05*100/3.0;
		cout<<"x: "<<x<<endl;
		cout<<"y: "<<y<<endl;
		cout<< x << ", "<< y<<endl;
		if(x>100 || x< -100 || y> 200 || y <0)
		{
			cout<<"?"<<endl;
			i += di;
			if(i>3.14) break;
			continue;
		}
		//if(i > 2*3.141592) break;
		pose.pose.position.x= x;
		pose.pose.position.y = y;
		pose.header.seq = seq;
		i+=di;
		if(i>3.14) break;
		// left
		//pose.pose.position.x = radius_test * cos(i/double(200) * 3.141592) - radius_test;
		// right
		//pose.pose.position.x = -1 * (radius_test * cos(i/double(200) * 3.141592) - radius_test);
		//pose.pose.position.y = radius_test * sin(i/double(200)*3.141592);
		publisher.curr_local_path.poses.push_back(pose);		
		seq++;
		//cout<<"---"<<endl;
	}

/*
	pose.pose.position.x = 0;
	pose.pose.position.y = 100;
	pose.header.seq = 1;
	publisher.curr_local_path.poses.push_back(pose);
*/
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
		publisher.local_path_pub.publish(publisher.curr_local_path);
		publisher.odometry_pub.publish(publisher.curr_odom);

		publisher.recommend_vel.data = 0.5;
		publisher.recommend_vel_pub.publish(publisher.recommend_vel);
		loop_rate.sleep();
		//vel_x += 1;
		//vel_y += 2;
		count++;
		cout << count << endl;
	}

	return 0;
}
