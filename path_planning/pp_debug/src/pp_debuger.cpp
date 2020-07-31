#include "ros/ros.h"
#include "defs.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "core_msgs/Control.h"
#include "nav_msgs/Path.h"

class RosNode{
private:
	ros::NodeHandle n;
	ros::Subscriber mission_state_sub;
	ros::Subscriber car_signal_sub;

	int sector,task, light, motion;
	int gear;
public:
	RosNode(){
		mission_state_sub = n.subscribe("mission_state", 5, &RosNode::missionstateCallback, this);
		car_signal_sub = n.subscribe("car_signal", 5, &RosNode::carsignalCallback, this);
		sector = 0;
		task = 0;
		light = 15;
		motion = 0;
	}

	void missionstateCallback(const std_msgs::UInt32 & msg){
		int mask = 0xF;
		int data = msg.data;
		motion = data & mask;
		light = (data>>4) & mask;
		task = (data>>8) & mask;
		sector = (data>>12) & mask;

		print_debug(sector,task,light,motion);
	}

	void carsignalCallback(const core_msgs::Control & msg){
		ROS_INFO("is_auto : %d", msg.is_auto);
		ROS_INFO("estop : %d", msg.estop);
		ROS_INFO("gear : %d", msg.gear);
		ROS_INFO("brake : %d", msg.brake);
		ROS_INFO("speed : %f", msg.speed);
		ROS_INFO("steer : %f", msg.steer);
	}

	void print_debug(int sector, int task, int light, int motion){
		switch(sector){
			case A : ROS_INFO("sector : A");
				break;
			case B : ROS_INFO("sector : B");
				break;
			case C : ROS_INFO("sector : C");
				break;
			case D : ROS_INFO("sector : D");
				break;
			case E : ROS_INFO("sector : E");
				break;
			case X : ROS_INFO("sector : X");
				break;
		}
		
		switch(task){
			case DRIVING_SECTION : ROS_INFO("task : DRIVING_SECTION");
				break;
			case INTERSECTION_STRAIGHT : ROS_INFO("task : INTERSECTION_STRAIGHT");
				break;
			case INTERSECTION_LEFT : ROS_INFO("task : INTERSECTION_LEFT");
				break;
			case INTERSECTION_RIGHT : ROS_INFO("task : INTERSECTION_RIGHT");
				break;
			case INTERSECTION_STRAIGHT_UNSIGNED : ROS_INFO("task : INTERSECTION_STRAIGHT_UNSIGNED");
				break;
			case INTERSECTION_LEFT_UNSIGNED : ROS_INFO("task : INTERSECTION_LEFT_UNSIGNED");
				break;
			case INTERSECTION_RIGHT_UNSIGNED : ROS_INFO("task : INTERSECTION_RIGHT_UNSIGNED");
				break;
			case OBSTACLE_STATIC : ROS_INFO("task : OBSTACLE_STATIC");
				break;
			case OBSTACLE_SUDDEN : ROS_INFO("task : OBSTACLE_SUDDEN");
				break;
			case CROSSWALK : ROS_INFO("task : CROSSWALK");
				break;
			case PARKING : ROS_INFO("task : PARKING");
				break;
		}

		if(isSign(light, 0)) ROS_INFO("light : GREEN_LIGHT");
		if(isSign(light, 1)) ROS_INFO("light : LEFT_LIGHT");
		if(isSign(light, 2)) ROS_INFO("light : YELLOW_LIGHT");
		if(isSign(light, 3)) ROS_INFO("light : RED_LIGHT");
		
		switch(motion){
			case FORWARD_MOTION : ROS_INFO("motion : FORWARD_MOTION");
				break;
			case FORWARD_SLOW_MOTION : ROS_INFO("motion : FORWARD_SLOW_MOTION");
				break;
			case HALT_MOTION : ROS_INFO("motion : HALT_MOTION");
				break;
			case LEFT_MOTION : ROS_INFO("motion : LEFT_MOTION");
				break;
			case RIGHT_MOTION : ROS_INFO("motion : RIGHT_MOTION");
				break;
			case PARKING_MOTION : ROS_INFO("motion : PARKING_MOTION");
				break;
		}
	}

	inline int isSign(int _light_state, int sign_num) {return ((_light_state)>>sign_num)&1;}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp_debuger");
	RosNode rosnode;
	ros::spin();
	return 0;
}

/*
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/UInt32.h"
#include "core_msgs/UInt32WithHeader.h"
#include <nav_msgs/Path.h>

using namespace message_filters;

void callback(const core_msgs::UInt32WithHeader::ConstPtr & mission_state_msg, const core_msgs::UInt32WithHeader::ConstPtr & gear_state_msg)
{
			int mask = 0xF;
			int data = mission_state_msg->data;
			int motion = data & mask;
			int light = (data>>4) & mask;
			int task = (data>>8) & mask;
			float recommend_vel = (data>>12)/4.0;
			ROS_INFO("task : %d", task);

			ROS_INFO("gear : %d", gear_state_msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pp_debuger");

  ros::NodeHandle nh;

  message_filters::Subscriber<core_msgs::UInt32WithHeader> mission_state_sub(nh, "mission_state", 5);
  message_filters::Subscriber<core_msgs::UInt32WithHeader> gear_state_sub(nh, "gear_state", 5);
  TimeSynchronizer<core_msgs::UInt32WithHeader, core_msgs::UInt32WithHeader> sync(mission_state_sub, gear_state_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
*/
