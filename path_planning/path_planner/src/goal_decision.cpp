#include "rrt_star.h"
#include "defs.h"
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

const static int OBSTACLE = 100;
//
//enum taskState{
//	DRIVING_SECTION,
//	INTERSECTION_STRAIGHT,
//	INTERSECTION_LEFT,
//	INTERSECTION_RIGHT,
//	INTERSECTION_STRAIGHT_UNSIGNED,
//    INTERSECTION_LEFT_UNSIGNED,
//    INTERSECTION_RIGHT_UNSIGNED,
//	OBSTACLE_STATIC,
//	OBSTACLE_SUDDEN,
//	CROSSWALK,
//	PARKING
//};
//
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
//	SEARCHING_PARKING_SPOT,
//    PARKING_SPOT_0,
//    PARKING_SPOT_1,
//    PARKING_SPOT_2,
//    PARKING_SPOT_3,
//    PARKING_SPOT_4,
//    PARKING_SPOT_5
//}
//
//
Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int parking_space, bool & parking_complished_changed, bool & unparking_complished_changed){
	///////////////////////////////////////////////
	static bool parking_complished = false;
	static bool unparking_complished = false;
	///////////////////////////////////////////////
	const double angle{0};
	const int x{0};
	const int y{0};

	// flag info : main(0), sub (1, = other lane), left(2), right(3), halt(4), parking(5~10)
	bool flag[12];
	for(int i = 0;i<5;i++) flag[i] = false;
	switch(motion){
		case FORWARD_MOTION :
		case FORWARD_SLOW_MOTION :
			flag[0] = true;
			flag[1] = true;
			flag[4] = true;
			break;
		case HALT_MOTION :
			flag[0] = true;
			flag[1] = true;
			break;
		case LEFT_MOTION :
			flag[0] = true;
			flag[1] = true;
			flag[2] = true;
			break;
		case RIGHT_MOTION :
			flag[0] = true;
			flag[1] = true;
			flag[3] = true;
			break;
		//////////////////////////////
		case PARKING_MOTION :
			switch(parking_space){
				case SEARCHING_PARKING_SPOT :
					flag[0] = true;
					flag[1] = true;
					break;
				case PARKING_SPOT_0 :
					flag[5] = true;
					break;
				case PARKING_SPOT_1 :
					flag[6] = true;
					break;
				case PARKING_SPOT_2 :
					flag[7] = true;
					break;
				case PARKING_SPOT_3 :
					flag[8] = true;
					break;
				case PARKING_SPOT_4 :
					flag[9] = true;
					break;
				case PARKING_SPOT_5 :
					flag[10] = true;
					break;
			}
			break;
		//////////////////////////////
	}

	double look_ahead_radius;
	if(task==OBSTACLE_SUDDEN) look_ahead_radius = 100;
	else if(motion == FORWARD_SLOW_MOTION) look_ahead_radius = 50;
	else if(motion == LEFT_MOTION || motion == RIGHT_MOTION) look_ahead_radius = 100;
	///////////////////////////////////////
	else if(motion == PARKING_MOTION) look_ahead_radius = 30;
	///////////////////////////////////////
	else look_ahead_radius = 200;


	// get value index which is closest to look_ahead_radius (except sub path), get value_sub for sub path
	double value = -1, value_sub = -1;
	double key = 100000, key_sub = 100000;


	////// in case obstacle sudden, slam team must give sequence of path so that
	////// we can know where obstacle is, and determine goal
	////// by checking nearest(smallest) sequence index of obstacle 
	int nearest_obs_seq = 1000000;

	// assign appropriate index which is closest to look_ahead_radius 
	int sz = goals.size();
	for(int i = 0; i<sz;i++){
		geometry_msgs::PoseStamped poseStamped = goals[i];		

		int pose_flag = poseStamped.header.seq & 0xF;
		int pose_seq = poseStamped.header.seq>>4;

		// check flag
		if(!flag[pose_flag]) continue;

		double goal_angle = poseStamped.pose.orientation.z;
		double ang_diff = angle - goal_angle;
		ang_diff = min(abs(ang_diff), min(abs(ang_diff + 2 * M_PI), abs(ang_diff - 2 * M_PI)));
		
		// check if same dir
		if(motion != PARKING_MOTION){
			if(ang_diff > M_PI/2) continue;
		}
		else if(!parking_complished){
			if(ang_diff > M_PI/2) continue;
		}
		else{ // rear motion
			if(ang_diff < M_PI/2) continue;
		}
		
		double dx = poseStamped.pose.orientation.x;
		double dy = poseStamped.pose.orientation.y;

		// check obstacle
		if(costmap[(int)dx][(int)dy] >= OBSTACLE) {
			if(task == OBSTACLE_SUDDEN && pose_seq < nearest_obs_seq) nearest_obs_seq = pose_seq;
			continue;
		}

		double dist = sqrt((dx-x)*(dx-x) + (dy-y)*(dy-y));

		// not sub path
		if(flag[pose_flag]!=1){
			if(abs(dist - look_ahead_radius) > key) continue;
			key = abs(dist - look_ahead_radius);
			value = i;
		}

		// sub path
		else{
			if(abs(dist - look_ahead_radius) > key_sub) continue;
			key_sub = abs(dist - look_ahead_radius);
			value_sub = i;
		}
	}

	// if obstacle sudden, choose goal which is farthest and closer than obstacle
	if(task == OBSTACLE_SUDDEN){
		value = -1;
		value_sub = -1;
		for(int i = 0; i<sz; i++){
			geometry_msgs::PoseStamped poseStamped = goals[i];		

			int pose_flag = poseStamped.header.seq & 0xF;
			int pose_seq = poseStamped.header.seq>>4;

			// check flag
			if(!flag[pose_flag]) continue;

			double goal_angle = poseStamped.pose.orientation.z;
			double ang_diff = angle - goal_angle;
			ang_diff = min(abs(ang_diff), min(abs(ang_diff + 2 * M_PI), abs(ang_diff - 2 * M_PI)));
		
			// check if same dir
			if(ang_diff > M_PI/2) continue;
		
			double dx = poseStamped.pose.orientation.x;
			double dy = poseStamped.pose.orientation.y;

			// check obstacle
			if(costmap[(int)dx][(int)dy] >= OBSTACLE) continue;

			// check closer than obstacle
			if(pose_seq >= nearest_obs_seq) continue;

			double dist = sqrt((dx-x)*(dx-x) + (dy-y)*(dy-y));

			// not sub path
			if(flag[pose_flag]!=1){
				if(abs(dist - look_ahead_radius) > key) continue;
				key = abs(dist - look_ahead_radius);
				value = i;
			}
	
			// sub path
			else{
				if(abs(dist - look_ahead_radius) > key_sub) continue;
				key_sub = abs(dist - look_ahead_radius);
				value_sub = i;
			}
		}
	}

	// if we find goal, return
	if(value != -1){
		double gx = goals[value].pose.orientation.x;
		double gy = goals[value].pose.orientation.y;
		return Cor(gx,gy);
	}

	
	if(task == OBSTACLE_STATIC){
		// can't find goal (main and sub), just go straight
		if(value_sub == -1) {
			return Cor(0,1);		
		}

		// else return sub path
		double gx = goals[value_sub].pose.orientation.x;
		double gy = goals[value_sub].pose.orientation.y;
		return Cor(gx,gy);
	}
	else if(task == OBSTACLE_SUDDEN){
		return Cor(0,0); 
	}

	/////////////////////////////////////////
	// we check whether parking or unparking complished by no goal available
	// when parking complished or unparking complished stop!
	else if(motion==PARKING_MOTION){
		// parking complished
		if(parking_complished==false){
			parking_complished = true;
			parking_complished_changed = true;
			return Cor(0,0);
		}
		// unparking complished
		else{
			unparking_complished = true;
			unparking_complished_changed = true;
			return Cor(0,0);
		}
	}
	/////////////////////////////////////////
	// please don't go into this condition
	// when there is no goal available stop! ////////////////////////should be checked!!
	else{
		// stop!!!!
		return Cor(0,0);
	}
}


































