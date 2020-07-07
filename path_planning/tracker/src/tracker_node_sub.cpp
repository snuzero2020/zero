#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

//using namespace std;
//using namespace std_msgs;
//using namespace nav_msgs;
//using namespace geometry_msgs;

/*
struct Point
{
	// the variables to represent relative location and direction
	double x;
	double y;
	double theta; 
}
*/

class Tracker
{
	private:
		std_msgs::Float32 steering_angle;
		nav_msgs::Path curr_local_path;
		nav_msgs::Odometry curr_odom;

		// configuration constants
		double look_ahead_oval_ratio{2}; // ratio of look ahead area which is oval shape
		double upper_radius{160};
		double lower_radius{80};

		double current_vel{0};
		geometry_msgs::Point look_ahead_point{Point()};
		double curvature{0};
		double nonslip_steering_angle{0}; // right side is positive
		double rotational_radius{0};

	public:
		// initializer
		Traker() 
			:string_angle(std_msgs::Float32()), curr_local_path(nav_msgs::Path()), curr_odom(nav_msgs::Odometry()),
				look_ahead_oval_ratio(0) {}

		// setter function
		void set_steering_angle(const double angle) {steering_angle.data = angle;}
		void set_curr_local_path(const nav_msgs::Path& path) {curr_local_path=path;}
		void set_curr_odom(const nav_msgs::Odometry odom) {curr_odom = odom;}
		// getter function
		const std_msgs::Float32 get_steering_angle() return steering_angle;
		const nav_msgs::Path get_current_path() return curr_local_path;
		const nav_msgs::Odometry get_current_odom() return curr_odom;
		// callback function
		void local_path_callback(const nav_msgs::Path& path);
		void odometory_callback(const nav_msgs::Odometry odometry);

		// Fuctions for determind steering angle
		// 1. set look ahead point under considering velocity
		void set_look_ahead_point();
		// 2. solve pure pursuit and yield curvature
		void solve_pure_pursuit();
		// 3. using experimental result to determind actual steering angle
		void adjust_steering_angle(); 
		// 1~3. determind steering angle
		void determind_steering_angle();

		double determind_major_axis_radius();
};

void Tracker::local_path_callback(const nav_msgs::Path& path)
{
	set_curr_local_path(path);
}

void Tracker::odometory_callback(const nav_msgs::Odometry odometry)
{
	set_curr_odom(odometry);
}

// input : curren_vel, variables related to look ahead area, curr_local_path
// output : look_ahead_point
void Tracker::set_look_ahead_point()
{
	current_vel = sqrt((curr_odom.twist.twist.linear.x)^2+(curr_odom.twist.twist.linear.y)^2);
	double major_axis_radius{determind_major_axis_radius()};
	int idx{0};
	double check_outside{0};
	while(1)
	{
		if (curr_local_path.poses[idx].header.seq == -1)
		{
			major_axis_radius -= 50;
			idx = 0;
			continue;
		}
		check_outside = (curr_local_path.poses[idx].pose.position.x)^2/(major_axis_radius/look_ahead_oval_ratio)^2 
			+ (curr_local_path.poses[idx].pose.position.y)^2/(major_axis_radius)^2;
		if (check_outside>1)
		{
			look_ahead_point.x = curr_local_path.poses[idx].pose.position.x;
			look_ahead_point.y = curr_local_path.poses[idx].pose.position.y;
			break;
		}
		idx++;
	}
}

// input : look_ahead_point
// output : rotational_radius, curvature, nonslip_steering_angle
void Tracker::solve_pure_pursuit()
{
	Point rotational_center{Point()};
	rotational_center.x = look_ahead_point.x/2 - look_ahead_point.y*(-1.7-look_ahead_point.y/2)/look_ahead_point.x;
	rotational_center.y = -1.7;
	curvature = 1/(sqrt(rotational_center.x^2+rotational_center.y^2));
	rotational_radius = 1/curvature;
	nonslip_steering_angle = -atan2(-rotational_center.y,-rotational_center.x);
}

// input : curvature, current_vel (!!!!!!!!!! discussion is required. choose between current_vel vs goal_vel)
// output : steering_angle
void Tracker::adjust_steering_angle()
{
	// experimental result should be reflected
	cout << "Experimental result should be reflected.\n";
	steering_angle = nonslip_steering_angle;
}

// capsulized module
void Tracker::determind_steering_angle()
{
	set_look_ahead_point();
	solve_pure_pursuit();
	adjust_steering_angle();
}

// input : current_vel
// output : major_axis_radius
// check is needed. 
double Tracker::determind_major_axis_radius()
{
	double major_axis_radius{0};
	major_axis_radius = lower_radius + (upper_radius-lower_radius)*current_vel/20;
	return major_axis_radius;
}


int main(int argc, char *argv[])
{
	ros::init(argc,argv,"path_traker");
	ros::NodeHandle nh;

	Tracker tracker;

	ros::Publisher steering_angle_pub = nh.advertise<std_msgs::Float32>("configuration",1000);
	ros::Subscriber local_path_sub = nh.subscribe("local_path",nav_msgs::Path,tracker.local_path_callback);
	ros::Subscriber odometory_sub = nh.subscribe("odometory",nav_msgs::Odometry,tracker.odometory_callback);
	ros::spin();

	ros::Rate loop_rate(10);
	int count{0};

	while (ros::ok())
	{
		tracker.determind_steering_angle();
		conf_pub.publish(tracker.get_steering_angle());
		loop_rate.sleep();
		++count;
	}

	return 0;
}
