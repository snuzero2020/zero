#include "rrt_star.h"
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

const static int OBSTACLE = 100;

enum taskState{
	DRIVING_SECTION,
	INTERSECTION_STRAIGHT,
	INTERSECTION_LEFT,
	INTERSECTION_RIGHT,
	OBSTACLE_STATIC,
	OBSTACLE_SUDDEN,
	CROSSWALK,
	PARKING
};

enum lightState{
	LIGHT_GREEN,
	LIGHT_LEFT,
	LIGHT_YELLOW,
	LIGHT_RED
};

enum motionState{
	FORWARD_MOTION,
	FORWARD_MOTION_SLOW,
	HALT,
	LEFT_MOTION,
	RIGHT_MOTION
};

Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int x, int y, double angle){
	bool flag[5];
	for(int i = 0;i<5;i++) flag[i] = false;
	switch(motion){
		case FORWARD_MOTION :
		case FORWARD_MOTION_SLOW :
			flag[0] = true;
			flag[1] = true;
			flag[4] = true;
			break;
		case HALT :
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
	}

	double look_ahead_radius;
	if(motion == FORWARD_MOTION_SLOW) look_ahead_radius = 50;
	else if(motion == LEFT_MOTION || motion == RIGHT_MOTION) look_ahead_radius = 100;
	else look_ahead_radius = 200;


	// get value index which is closest to look_ahead_radius (except sub path), get value_sub for sub path
	double value = -1, value_sub = -1;
	double key = 100000, key_sub = 100000;
	int sz = goals.size();
	for(int i = 0; i<sz;i++){
		geometry_msgs::PoseStamped poseStamped = goals[i];		

		// check flag
		if(!flag[poseStamped.header.seq]) continue;

		double goal_angle = poseStamped.pose.orientation.z;
		double ang_diff = angle - goal_angle;
		ang_diff = min(abs(ang_diff), min(abs(ang_diff + 2 * M_PI), abs(ang_diff - 2 * M_PI)));
		
		// check if same dir
		if(ang_diff > M_PI/2) continue;
		
		// check if on path
		double dx = poseStamped.pose.orientation.x;
		double dy = poseStamped.pose.orientation.y;
		if((dx - x) * cos(angle) + (dy-y) * sin(angle) <= 0) continue;

		// check obstacle
		if(costmap[(int)dx][(int)dy] >= OBSTACLE) continue;

		double dist = sqrt((dx-x)*(dx-x) + (dy-y)*(dy-y));

		// not sub path
		if(flag[poseStamped.header.seq]!=1){
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

	if(value != -1){
		double gx = goals[value].pose.orientation.x;
		double gy = goals[value].pose.orientation.y;
		return Cor(gx,gy);
	}


	if(task == OBSTACLE_STATIC){
		if(value_sub == -1) {
			return Cor(100,1);		
		}
		double gx = goals[value_sub].pose.orientation.x;
		double gy = goals[value_sub].pose.orientation.y;
		return Cor(gx,gy);
	}
	else if(task == OBSTACLE_SUDDEN){
		return Cor(100,1);
	}
	else{
		// stop!!!!
		return Cor(-1, -1);
	}
}





















