#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include "core_msgs/Control.h"
#include "core_msgs/VehicleState.h"
#include "core_msgs/Encoderfilter.h"
#include "std_msgs/UInt32.h"


enum taskState{
      DRIVING_SECTION,
      INTERSECTION_STRAIGHT,
      INTERSECTION_LEFT,
      INTERSECTION_RIGHT,
      INTERSECTION_STRAIGHT_UNSIGNED,
      INTERSECTION_LEFT_UNSIGNED,
      INTERSECTION_RIGHT_UNSIGNED,
      OBSTACLE_STATIC,
      OBSTACLE_SUDDEN,
      CROSSWALK,
      PARKING
};

// if LEFT_LIGHT && RED_LIGHT, then light_state = 1010 (bit) = 10
enum lightState{
      GREEN_LIGHT,
      LEFT_LIGHT,
      YELLOW_LIGHT,
      RED_LIGHT
};

enum motionState{
      FORWARD_MOTION,
      FORWARD_SLOW_MOTION,
      HALT_MOTION,
      LEFT_MOTION,
      RIGHT_MOTION,
      PARKING_MOTION
};

enum parkingState{
    SEARCHING_PARKING_SPOT,
    PARKING_SPOT_0,
    PARKING_SPOT_1,
    PARKING_SPOT_2,
    PARKING_SPOT_3,
    PARKING_SPOT_4,
    PARKING_SPOT_5
};


using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace core_msgs;

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
		ros::NodeHandle nh;

		Float32 steering_angle;
		Path curr_local_path;
		double current_vel{0};

		// configuration constants
		double look_ahead_oval_ratio{2}; // ratio of look ahead area which is oval shape
		double upper_radius;
		double lower_radius;

		Point look_ahead_point{Point()};
		double curvature{0};
		double nonslip_steering_angle{0}; // right side is positive, degree
		double rotational_radius{0};

		// PID control
		double recommend_vel{0}; // determine by mission master and location
		double desired_vel_before{0}; // get from current_vel and recommend_vel
		double desired_vel_after{0};
		double desired_vel_buff{0};
		double slope{0};

		double P_gain;
		double I_gain;
		double D_gain;
		double error{0};
		double integral_error{0};
		double differential_error{0};
		double Prev_error{0};
		double pid_input{0};
		clock_t time = clock();

		int decel_level = 0;
		int decel_check = 0;

		int prev_gear_state{0};

	public:

		ros::Publisher steering_angle_pub;
		ros::Publisher car_signal_pub;
		ros::Subscriber local_path_sub;
		ros::Subscriber current_vel_sub;
		ros::Subscriber recommend_vel_sub;
		ros::Subscriber mission_state_sub;
		ros::Subscriber gear_state_sub;
		ros::Subscriber vehicle_state_sub;

		int motion{-1};
		int light{-1};
		int task{-1};
		int parking_space{-1};
		double max_vel_increase;
		double max_obstacle_vel;
		double max_parking_vel;

		VehicleState curr_vehicle_state{VehicleState()};
		
		// initializer
		Tracker() 
			:steering_angle(Float32()), curr_local_path(Path()), look_ahead_oval_ratio(2)
		{
			steering_angle_pub = nh.advertise<Float32>("configuration",1000);
			car_signal_pub = nh.advertise<core_msgs::Control>("/car_signal", 2);
			local_path_sub = nh.subscribe("local_path",2,&Tracker::local_path_callback,this);
			//current_vel_sub = nh.subscribe("/vehicle_state",2, &Tracker::current_vel_callback, this);
			current_vel_sub = nh.subscribe("filter_encoder_data",2, &Tracker::current_vel_callback, this);
			recommend_vel_sub = nh.subscribe("recommend_vel",2, &Tracker::recommend_vel_callback, this);
			mission_state_sub = nh.subscribe("mission_state", 2, &Tracker::missionstateCallback, this);
			gear_state_sub = nh.subscribe("gear_state", 2, &Tracker::gear_state_callback, this);
			vehicle_state_sub = nh.subscribe("/vehicle_state", 2, &Tracker::vehicle_state_callback, this);
			
			nh.getParam("/P_gain", P_gain);
			nh.getParam("/I_gain", I_gain);
			nh.getParam("/D_gain", D_gain);
			nh.getParam("/upper_radius", upper_radius);
			nh.getParam("/lower_radius", lower_radius);
			nh.getParam("/max_vel_increase", max_vel_increase);
			nh.getParam("/max_obstacle_vel",max_obstacle_vel);
			nh.getParam("/max_parking_vel",max_parking_vel);
		}

		// setter function
		void set_steering_angle(const double angle) {steering_angle.data = angle;}
		void set_curr_local_path(const Path& path) {curr_local_path=path;}
		// getter function
		const Float32 get_steering_angle() {return steering_angle;}
		const Path get_current_path() {return curr_local_path;}
		// callback function
		void local_path_callback(const Path::ConstPtr msg);
		void current_vel_callback(const core_msgs::Encoderfilter::ConstPtr msg);
		void recommend_vel_callback(const Float32::ConstPtr msg);
		void missionstateCallback(const std_msgs::UInt32 & msg);
		void gear_state_callback(const std_msgs::UInt32 & msg);
		void vehicle_state_callback(const core_msgs::VehicleState & msg);

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

		// PID control
		double calculate_desired_vel();
		void calculate_input_signal();
		void vehicle_output_signal();

		//test
		//void print_p(){std::cout<<P_gain<<std::endl;};
};

void Tracker::local_path_callback(const Path::ConstPtr msg)
{
	//set_curr_local_path(msg);
	curr_local_path = Path();
	curr_local_path.header = msg->header;

	int i{0};
	while(1)
	{
		if(msg->poses.size()==0)
		{
			break;
		}
		else if(msg->poses[i].header.seq != 0)
		{
			curr_local_path.poses.push_back(msg->poses[i]);
		}
		else
		{
			curr_local_path.poses.push_back(msg->poses[i]);
			break;
		}
		i++;
	}
	
	if (curr_local_path.poses.size()==0){
		cout << "no path!!!!\n";
		
		core_msgs::Control msg;
		msg.is_auto = 1;
		msg.estop = 0;
		msg.gear = 0;
		msg.brake = 100;
		msg.speed = 0;
		msg.steer = 0;

		// while bracking, pid should be reset
		integral_error = 0;
		desired_vel_before = 0;

		car_signal_pub.publish(msg);
		return;
	}

	// when vehivle state is estop or manual_control mode reset integral error and desired_vel_before
	
	/*
	if (curr_vehicle_state.estop==1 || curr_vehicle_state.is_auto != 1)
	{
		core_msgs::Control msg;
		
		if (curr_vehicle_state.estop==1){
			cout << "estop!!!!\n";
			msg.is_auto = 1;
			msg.estop = 1;
		}

		if (curr_vehicle_state.is_auto != 1){
			cout << "manual_control mode!!!!\n";
			msg.is_auto = 0;
			msg.estop = 0;
		}

		msg.gear = 0;
		msg.brake = 100;
		msg.speed = 0;
		msg.steer = 0;

		// while bracking, pid should be reset
		integral_error = 0;
		desired_vel_before = 0;

		car_signal_pub.publish(msg);
		return;
	}
	*/
	/*
	if (curr_vehicle_state.estop==true || curr_vehicle_state.is_auto != true)
	{
		// while bracking, pid should be reset
		integral_error = 0;
		desired_vel_before = 0;
		return;
	}
	*/

	determind_steering_angle();
	calculate_input_signal();
	vehicle_output_signal();

	cout << "current_vel : " << current_vel << endl;
	cout << "look_ahead_point : (" << look_ahead_point.x << "," << look_ahead_point.y << ")\n";
	cout << "steering_angle : " << steering_angle << endl;
	//cout << "duration time : " << (time-clock())/double(CLOCKS_PER_SEC) << endl << endl;
}


void Tracker::missionstateCallback(const std_msgs::UInt32 & msg){
        int mask = 0b1111;
        int data = msg.data;
        motion = data & mask;
        light = (data>>4) & mask;
        task = (data>>8) & mask;
        //parking_space = (data>>12) & mask;
        parking_space = 1;
	cout << "motion : " << motion << " light : " << light << " task : " << task << endl;
}


void Tracker::recommend_vel_callback(const Float32::ConstPtr msg)
{
	recommend_vel = msg->data;
}

void Tracker::current_vel_callback(const core_msgs::Encoderfilter::ConstPtr msg){
	current_vel = msg->filtered_encoder;
	slope = msg->slope;
}


void Tracker::gear_state_callback(const std_msgs::UInt32 & msg){
	if(prev_gear_state != msg.data)
		integral_error = 0;
	prev_gear_state = msg.data;
}
		

void Tracker::vehicle_state_callback(const core_msgs::VehicleState & msg){
	curr_vehicle_state.is_auto = msg.is_auto;
	curr_vehicle_state.estop = msg.estop;
	curr_vehicle_state.gear = msg.gear;
	curr_vehicle_state.brake = msg.brake;
	curr_vehicle_state.speed = msg.speed;
	curr_vehicle_state.steer = msg.steer;
	curr_vehicle_state.encoder = msg.encoder;
	curr_vehicle_state.alive = msg.alive;
	curr_vehicle_state.header.stamp = msg.header.stamp;
}

// input : curren_vel, variables related to look ahead area, curr_local_path
// output : look_ahead_point
void Tracker::set_look_ahead_point()
{
	double major_axis_radius{determind_major_axis_radius()};
	int idx{0};
	double check_outside{0};
	while(1)
	{
		// end of path --> lower radius and iter from the start
		if (curr_local_path.poses[idx].header.seq == 0)
		{
			if (major_axis_radius < 5){
				if(major_axis_radius <  1.1) {
					look_ahead_point.x = 0;
					look_ahead_point.y = 1; 
					break;
				}
				major_axis_radius = 1;
			}
			else
				major_axis_radius /= 1.5;
			idx = 0;
			continue;
		}

		check_outside = (curr_local_path.poses[idx].pose.position.x)*(curr_local_path.poses[idx].pose.position.x)/((major_axis_radius/look_ahead_oval_ratio)* (major_axis_radius/look_ahead_oval_ratio))
			+ (curr_local_path.poses[idx].pose.position.y)*(curr_local_path.poses[idx].pose.position.y)/((major_axis_radius)*(major_axis_radius));
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
	double temp_angle; 
	cout << "curr_local_path.header.stamp.sec = " << curr_local_path.header.stamp.sec << endl;
	bool is_front_gear = (curr_local_path.header.stamp.sec < 2);
	cout << "is_front_gear : " << is_front_gear << endl;
	// curr_local_path.header.stamp.sec & 0b10 != 0b10 : front gear
	// curr_local_path.header.stamp.sec & 0b10 == 0b10 : reverse gear (only for parking motion with backward motion
	
	Point temp{Point()};
	temp.x = -look_ahead_point.y;
	temp.y = look_ahead_point.x;


	if (is_front_gear){
		//rotational_center.x = look_ahead_point.x/2.0 - look_ahead_point.y*(-1.05*100/3.0-look_ahead_point.y/2.0)/double(look_ahead_point.x);
		rotational_center.y = look_ahead_point.y/2.0 + look_ahead_point.x*(1.05*100/3.0+look_ahead_point.x/2.0)/double(look_ahead_point.y);
		//rotational_center.y = -1.05*100/3.0;
		rotational_center.x = -1.05*100/3.0;
		//curvature = 1/(sqrt(rotational_center.x*rotational_center.x+rotational_center.y*rotational_center.y));
		curvature = 1/(sqrt(rotational_center.x*rotational_center.x+rotational_center.y*rotational_center.y));
		//rotational_radius = 1/curvature;
		rotational_radius = 1/curvature;
		//temp_angle =  atan2(-rotational_center.y,-rotational_center.x);
		temp_angle =  atan2(-rotational_center.x,rotational_center.y);
		cout << "temp_angle : " << temp_angle*180.0/M_PI << endl;
	}
	else{
		//rotational_center.x = look_ahead_point.x/2.0 - look_ahead_point.y*look_ahead_point.y/double(look_ahead_point.x*2.0);
		rotational_center.y = look_ahead_point.y/2.0 + look_ahead_point.x*look_ahead_point.x/double(look_ahead_point.y*2.0);
		//rotational_center.y = 0;
		rotational_center.x = 0;
		//curvature = 1/(sqrt(rotational_center.x*rotational_center.x+rotational_center.y*rotational_center.y));
		curvature = 1/(sqrt(rotational_center.x*rotational_center.x+rotational_center.y*rotational_center.y));
		//rotational_radius = 1/curvature;
		rotational_radius = 1/curvature;
		//temp_angle =  atan2(1.05*100/3.0,rotational_center.x);
		temp_angle =  atan2(1.05*100/3.0,-rotational_center.y);
		cout << "temp_angle : " << temp_angle*180.0/M_PI << endl;
	}

	/////////////////////// old version
	/*
	if (!is_front_gear){
		look_ahead_point.x *= -1.0;
		look_ahead_point.y *= -1.0;
	}
	*/

	///////////////////// old version
	/*
	rotational_center.x = look_ahead_point.x/2.0 - look_ahead_point.y*(-1.05*100/3.0-look_ahead_point.y/2.0)/double(look_ahead_point.x);
	rotational_center.y = -1.05*100/3.0;
	curvature = 1/(sqrt(rotational_center.x*rotational_center.x+rotational_center.y*rotational_center.y));
	rotational_radius = 1/curvature;
	temp_angle =  atan2(-rotational_center.y,rotational_center.x);
	*/

	// temp_angle < 3.141592/2.0 : right turn
	// temp_angle > 3.141592/2.0 : left turn

	if (temp_angle < 3.141592/2.0){
		nonslip_steering_angle = temp_angle*180/3.141592;
	}
	else{
		nonslip_steering_angle = (temp_angle-3.141592)*180/3.141592;
	}


	//////////////////// old version
	/*
	if (temp_angle < 3.141592/2.0){
		if (is_front_gear){
			nonslip_steering_angle = temp_angle*180/3.141592;
		}
		else
			nonslip_steering_angle = (temp_angle-3.141592)*180/3.141592;
	}
	else{
		if (is_front_gear){
			nonslip_steering_angle = (temp_angle-3.141592)*180/3.141592;
		}
		else
			nonslip_steering_angle = temp_angle*180/3.141592;
	}
	*/

	/*
	if (temp_angle < 3.141592/2.0){
		nonslip_steering_angle = temp_angle*180/3.141592;
	}
	else{
		nonslip_steering_angle = (temp_angle-3.141592)*180/3.141592;
	}
	*/

}

// input : curvature, current_vel (!!!!!!!!!! discussion is required. choose between current_vel vs goal_vel)
// output : steering_angle
void Tracker::adjust_steering_angle()
{
	// experimental result should be reflected
	//left 
	if(nonslip_steering_angle<1e-6)
	{
		steering_angle.data = -84.359 * pow(rotational_radius*0.03, -1.029);
		cout<<"rotational_radius: "<<rotational_radius<<endl;
		cout<<"---"<<endl;
	}
	//right
	else
	{
		steering_angle.data = 83.556 * pow(rotational_radius*0.03, -0.99);
		cout<<"rotational_radius: "<<rotational_radius<<endl;
		
		cout<<"---"<<endl;
	}
	// total
	//float sign = (nonslip_steering_angle>0)?1.0:-1.0;
	//steering_angle.data = sign * 79.12363452 * pow(rotational_radius, -0.98117930922);	

	if (task != PARKING){	
		if (steering_angle.data < -28){
			cout << "calculated steering angle is too large\n";       
			steering_angle.data = -28;
		}
		else if (steering_angle.data >28){
			steering_angle.data = 28;
			cout << "calculated steering angle is too large\n";      
		}	
	}
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
	major_axis_radius = lower_radius + (upper_radius-lower_radius)*current_vel/5;
	return major_axis_radius;
}


// input : recommend_vel, curvature
// output : desired_vel
double Tracker::calculate_desired_vel(){
	double look_ahead_multiplier{1};
	double curvature_multiplier{1};
	//look_ahead_multiplier = sqrt(sqrt(look_ahead_point.x*look_ahead_point.x+look_ahead_point.y*look_ahead_point.y)/100.0);
	look_ahead_multiplier = sqrt(look_ahead_point.x*look_ahead_point.x+look_ahead_point.y*look_ahead_point.y)/100.0;
	look_ahead_multiplier = (look_ahead_multiplier>1+1E-6)? 1.0:look_ahead_multiplier;
	curvature_multiplier = 1 - 1.0/pow((max(1.0/curvature/33.0,2.5)-1.5),0.75);
	desired_vel_after =  recommend_vel*curvature_multiplier*look_ahead_multiplier; // should be changed

	/*	
	look_ahead_multiplier = sqrt(look_ahead_point.x*look_ahead_point.x+look_ahead_point.y*look_ahead_point.y)/100.0;
	look_ahead_multiplier = (look_ahead_multiplier>1+1E-6)? 1.0:look_ahead_multiplier;
	curvature_multiplier = 1 - 1.0/pow((1.0/curvature-1.5),1);
	desired_vel_after =  recommend_vel*curvature_multiplier*look_ahead_multiplier; // should be changed
	
	if (abs(desired_vel_buff-desired_vel_after)>0.0001) //&& recommend_vel_change_check == false)
	{
		desired_vel_before = desired_vel_buff;
		desired_vel_buff = desired_vel_after;
	}
	*/

	if (task == OBSTACLE_STATIC || task == OBSTACLE_SUDDEN)
		desired_vel_after = min(desired_vel_after,max_obstacle_vel);
	if (task == PARKING)
		desired_vel_after = min(desired_vel_after,max_parking_vel);

	cout << "look_ahead_multiplier : " << look_ahead_multiplier << endl;
	cout << "curvature_multiplier : " << curvature_multiplier << endl;
	cout << "desired_vel_before : " << desired_vel_before << endl;
	cout << "desired_vel_after : " << desired_vel_after << endl;

	cout << "max_vel_increase : " << max_vel_increase << endl;

	if (desired_vel_after > desired_vel_before + max_vel_increase + 1E-6)
		desired_vel_after = desired_vel_before + max_vel_increase;

	cout << "adjusted_desired_vel_after : " << desired_vel_after << endl;

	desired_vel_before = desired_vel_after;
	return desired_vel_after;
}

// input : current_vel, recommed_vel, curvature
// output : control Pid input
void Tracker::calculate_input_signal(){
	static bool first = true;
	if (first == true)
	{
		time = clock();
		first = false;
		return;
	}
	
	// when gear_state is 1 -> rear gear, velocity input should be negetive.
	if (prev_gear_state==1)
		current_vel = abs(current_vel);

	error = calculate_desired_vel() - current_vel;
	integral_error = integral_error + error * (double(clock() - time)/(double)CLOCKS_PER_SEC);
	integral_error = (integral_error>0)? integral_error:0;
	differential_error = -slope;
	pid_input = P_gain * error + I_gain * integral_error + D_gain * differential_error + desired_vel_after;
	// pid_input's limit is 6  
	if (pid_input > 6){
		pid_input = 6;
	}
	Prev_error = error;
	cout << "dt : " << (clock()-time)/(double)CLOCKS_PER_SEC << endl;
	time = clock();
	cout << "error : " << error << endl;
	cout << "integral_error : " << integral_error << endl;
	cout << "differential_error : " << differential_error << endl;
	cout << "pid_input : " << pid_input << endl; 
}

void Tracker::vehicle_output_signal(){
	core_msgs::Control msg;

	//if ( desired_vel_before > desired_vel_after){
	if ( current_vel > desired_vel_after + 0.000001){
		decel_check = 1;
	}
	
	cout << "current_vel : " << current_vel << endl;
//	cout << "decel_level : " << decel_level << endl;
	cout << "decel_check : " << decel_check << endl;

	if(decel_check == 1){
		cout << "decceleration\n";
		if (desired_vel_after < 0.2){
			msg.brake = 50;
			integral_error = 0;
			msg.speed = 0;
			decel_check = 0;
		}
		else{
			msg.brake = 0;
			msg.speed = (pid_input>0)?pid_input:0;
			decel_check = 0;
		}
	}
	else {
		cout << "acceleration\n";
		msg.brake = 0;
		msg.speed = (pid_input>0)?pid_input:0;
	}

	msg.is_auto = 1;
	msg.estop = 0;

	cout<<"curr seq : "<<curr_local_path.header.stamp.sec<<"\n\n"; 
	// forward motion
	if ((curr_local_path.header.stamp.sec & 0b10) != 0b10){
		msg.gear = 0;
	}
	// backward motion
	else{
		msg.gear = 2;
	}

	/////////////////////////////////////////////check//////////
	//msg.speed = 0;
	////////////////////////////////////////////////////////////
	
	// if look_ahead_dist is less than 50cm, set steer to 0 to prevent sudden tilting of the vehicle
	double look_ahead_dist;
	look_ahead_dist = sqrt(look_ahead_point.x*look_ahead_point.x+look_ahead_point.y*look_ahead_point.y);
	if (task != PARKING){
	/*	if (look_ahead_dist<100 && (motion==HALT_MOTION || task==OBSTACLE_SUDDEN))
			msg.steer = get_steering_angle().data/5.0;*/
		if (motion==HALT_MOTION)// || task==OBSTACLE_SUDDEN)
			msg.steer = get_steering_angle().data/(look_ahead_dist>200?1:(-(look_ahead_dist-20)*4.0/180.0 + 5));
		else if (task==OBSTACLE_SUDDEN)
			msg.steer = get_steering_angle().data/(look_ahead_dist>200?1:(-(look_ahead_dist-20)*2.0/80.0 + 3));
		else
			msg.steer = get_steering_angle().data;
		car_signal_pub.publish(msg);
	}
	else {
		msg.steer = get_steering_angle().data;
		car_signal_pub.publish(msg);
	}
}



int main(int argc, char *argv[])
{

	ros::init(argc,argv,"path_traker");
	ros::NodeHandle nh;

	Tracker tracker{Tracker()};
	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}
