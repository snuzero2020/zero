#include <vector>
#include "ros/ros.h"
#include "rrt_star.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
//
//enum taskState{
//	DRIVING_SECTION,
//	INTERSECTION_STRAIGHT,
//	INTERSECTION_LEFT,
//	INTERSECTION_RIGHT,
//	INTERSECTION_STRAIGHT_UNSIGNED,
//	INTERSECTION_LEFT_UNSIGNED,
//	INTERSECTION_RIGHT_UNSIGNED,
//	OBSTACLE_STATIC,
//	OBSTACLE_SUDDEN,
//	CROSSWALK,
//	PARKING
//};
//
//// if LEFT_LIGHT && RED_LIGHT, then light_state = 1010 (bit) = 10
//enum lightState{
//	GREEN_LIGHT,
//	LEFT_LIGHT,
//	YELLOW_LIGHT,
//	RED_LIGHT
//};
//
//enum motionState{
//	FORWARD_MOTION,
//	FORWARD_SLOW_MOTION,
//	HALT_MOTION,
//	LEFT_MOTION,
//	RIGHT_MOTION,
//	PARKING_MOTION
//};
//
//enum parkingState{
//    SEARCHING_PARKING_SPOT,
//    PARKING_SPOT_0,
//    PARKING_SPOT_1,
//    PARKING_SPOT_2,
//    PARKING_SPOT_3,
//    PARKING_SPOT_4,
//    PARKING_SPOT_5
//}
//

class RosNode{
private:
	ros::NodeHandle n;
	ros::Subscriber light_state_sub;
	ros::Subscriber task_state_sub;
	ros::Publisher mission_state_pub;
	ros::Publisher recommend_vel_pub;
	int light_state;
	bool debug;
public:
	RosNode(){
		light_state_sub = n.subscribe("light_state", 50, &RosNode::lightstateCallback, this);
		task_state_sub = n.subscribe("task_state_with_std_vel", 50, &RosNode::taskstateCallback, this);
		mission_state_pub = n.advertise<std_msgs::UInt32>("mission_state", 50);
		recommend_vel_pub = n.advertise<std_msgs::Float32>("recommend_vel", 50);
		light_state = -1;

		debug = false;
	}

	inline int isSign(int _light_state, int sign_num) {return ((_light_state)>>sign_num)&1;}

	void lightstateCallback(const std_msgs::UInt32 & msg){
		light_state = (int)msg.data;
		if(debug) ROS_INFO("light_state : %d",msg.data);
	}

	void taskstateCallback(const std_msgs::UInt32 & msg){

		int task_state = (int)msg.data & 0x1111;
		int std_vel = (int)msg.data >> 4;
		float recommend_vel = (float)std_vel;

		if(debug) ROS_INFO("task_state : %d",msg.data);

		int motion_state;
		switch(task_state){
			case DRIVING_SECTION :
				motion_state = FORWARD_MOTION;
				break;

			case INTERSECTION_STRAIGHT :
				if(light_state == 0) motion_state = FORWARD_SLOW_MOTION;
				else if(isSign(light_state,GREEN_LIGHT)) motion_state = FORWARD_MOTION;
				else motion_state = HALT_MOTION;
				break;

			case INTERSECTION_LEFT :
				if(light_state == 0) motion_state = FORWARD_SLOW_MOTION;
				else if(isSign(light_state,LEFT_LIGHT)) motion_state = LEFT_MOTION;
				else motion_state = HALT_MOTION;
				break;

			case INTERSECTION_RIGHT :
				motion_state = RIGHT_MOTION;
				break;


			case INTERSECTION_STRAIGHT_UNSIGNED:
				motion_state = FORWARD_MOTION;
				break;

			case INTERSECTION_LEFT_UNSIGNED:
				motion_state = LEFT_MOTION;
				break;

			case INTERSECTION_RIGHT_UNSIGNED:
				motion_state = RIGHT_MOTION;
				break;

			case OBSTACLE_STATIC :
				motion_state = FORWARD_SLOW_MOTION;
				break;

			case OBSTACLE_SUDDEN :
				motion_state = FORWARD_SLOW_MOTION;
				break;

			case CROSSWALK :
				if(light_state == 0) motion_state = FORWARD_SLOW_MOTION;
				else if(isSign(light_state,GREEN_LIGHT)) motion_state = FORWARD_MOTION;
				else motion_state = HALT_MOTION;
				break;

			case PARKING :
				motion_state = PARKING_MOTION;
				break;
		}

		if(motion_state == HALT_MOTION) recommend_vel /= 2.0;
		else if(motion_state == LEFT_MOTION || motion_state == RIGHT_MOTION) recommend_vel /= 2.0;

		std_msgs::UInt32 mission_state;
		mission_state.data = (task_state<<8) | (light_state<<4) | motion_state;
		mission_state_pub.publish(mission_state);		
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_recognizer");
	RosNode rosnode;
	ros::spin();
	return 0;
}
