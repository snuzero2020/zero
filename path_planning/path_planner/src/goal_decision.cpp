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
Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int parking_space, bool & parking_complished_changed, bool & unparking_complished_changed, int gear_state){
	static bool parking_complished = false;
	static bool unparking_complished = false;

	int cost_scale;
	int lidar_detect_dist;
	double almost_obstacle_ratio;
	
	ros::param::get("/lidar_detect_dist",lidar_detect_dist);
	ros::param::get("/cost_scale",cost_scale);
	ros::param::get("/almost_obstacle_ratio",almost_obstacle_ratio);

	bool go_sub_path = false;
	const double angle{0.0};
	const int x{0};
	const int y{0};

	printf("parking space : %d\n",parking_space);

	// flag info : main(0), sub (1, = other lane), left(2), right(3), straight(4), parking(5~10)
	bool flag[12];
	for(int i = 0;i<12;i++) flag[i] = false;
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
	}

	printf("flag : ");
	for(bool flag_check : flag) cout << flag_check << " ";
	cout << endl;

	double look_ahead_radius;
	if(task==OBSTACLE_SUDDEN) look_ahead_radius = 200;
	else if(motion == LEFT_MOTION || motion == RIGHT_MOTION) look_ahead_radius = 130;
	else if(motion == PARKING_MOTION){
	       if(parking_space == SEARCHING_PARKING_SPOT)
	       	       look_ahead_radius = 100;
	       else
		       look_ahead_radius = 40;
	}
	else if(motion == HALT_MOTION) look_ahead_radius = 80;
	else look_ahead_radius = 200;


	// get value index which is closest to look_ahead_radius (except sub path), get value_sub for sub path
	double value = -1, value_sub = -1;
	double key = 100000, key_sub = 100000;


	////// in case obstacle sudden, slam team must give sequence of path so that
	////// we can know where obstacle is, and determine goal
	////// by checking nearest(smallest) sequence index of goal point on obstacle 
	int nearest_obs_seq = 1000000;

	// assign appropriate index which is closest to look_ahead_radius 
	int sz = goals.size();
	int main_count = 0;
	for(int i = 0; i<sz;i++){
		geometry_msgs::PoseStamped poseStamped = goals[i];		
		int pose_flag = poseStamped.header.seq & 0b1111;
		//cout << "flag is " << pose_flag << endl;
		//int pose_flag = 0;
		int pose_seq = poseStamped.header.seq>>4;
		if(pose_flag == 0) main_count++;

		//cout << "(goal decision) flag check!\n";
		// check flag
		if(!flag[pose_flag]) continue;

		double goal_angle = poseStamped.pose.position.z;
		double ang_diff = angle - goal_angle;
		ang_diff = min(abs(ang_diff), min(abs(ang_diff + 2 * M_PI), abs(ang_diff - 2 * M_PI)));
		
		//cout << "(goal decision) heading check!\n";
		// check if same dir
		if(motion != PARKING_MOTION){
			if(task != INTERSECTION_STRAIGHT){
				if(ang_diff > M_PI/2) continue;
			}
			else{
				if(ang_diff > M_PI/3) continue;
			}
		}
		else if(!parking_complished){
			if(ang_diff > M_PI/2) continue;
		}
		else{ // rear motion
			if(ang_diff > M_PI/2) continue;
		}

		double dx = poseStamped.pose.position.x;
		double dy = poseStamped.pose.position.y;
	
		// if goal point is too near, than ignore the point.
		if(dx < 20 && (task == OBSTACLE_SUDDEN || motion == HALT_MOTION)) continue;
		if(dx < 10 && (task == PARKING)) continue;

		// check obstacle
		
		double dist = sqrt((dx-x)*(dx-x) + (dy-y)*(dy-y));
		
		if(costmap[(int)dx][(int)dy+costmap.size()/2] >= OBSTACLE) {
			if(task == OBSTACLE_SUDDEN && pose_seq < nearest_obs_seq) nearest_obs_seq = pose_seq;
	
///////////////////////////////////////
			int almost_obstacle{OBSTACLE-static_cast<int>(cost_scale*almost_obstacle_ratio)};
			if(task == OBSTACLE_STATIC && dist < look_ahead_radius && pose_flag == 0 && costmap[(int)dx][(int)dy+costmap.size()/2] >= almost_obstacle){
			       	cout << "(" << dx << ',' << dy << ") cost : " << costmap[(int)dx][(int)dy+costmap.size()/2] << "\n";
				go_sub_path = true;
			}
///////////////////////////////////////
			continue;
		}


		// not sub path
		//if(flag[pose_flag]!=1){
		if(pose_flag!=1){
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
		//cout << "(goal decision) all check pass!\n";
	}
	
	if(task == OBSTACLE_STATIC){
		if(main_count == 0) go_sub_path = true;
		else if (value != -1){
			double gx = goals[value].pose.position.x;
			double gy = goals[value].pose.position.y;
			double marcher_x{0.0};
			double marcher_y{0.0};
			double step_size{1.0};
			double goal_dist{sqrt(gx*gx+gy*gy)};
			double dx = step_size*gx/goal_dist;
			double dy = step_size*gy/goal_dist;
			int step_times_static{0};
			while (goal_dist > step_times_static*step_size)
			{
				marcher_x += dx;
				marcher_y += dy;
				step_times_static++;
				if(costmap[(int)marcher_x][(int)marcher_y+costmap.size()/2]>(OBSTACLE-static_cast<int>(cost_scale*almost_obstacle_ratio)))
					go_sub_path = true;
			}
		}
	}




	printf("nearest_obs_seq : %d\n",nearest_obs_seq);
	// if obstacle sudden, choose goal which is farthest and closer than obstacle
	if(task == OBSTACLE_SUDDEN){
		value = -1;
		value_sub = -1;
		key = 100000; key_sub = 100000;
		for(int i = 0; i<sz; i++){
			geometry_msgs::PoseStamped poseStamped = goals[i];		
			int pose_flag = poseStamped.header.seq & 0b1111;
			int pose_seq = poseStamped.header.seq>>4;

			// check flag
			if(!flag[pose_flag]) continue;

			double goal_angle = poseStamped.pose.position.z;
			double ang_diff = angle - goal_angle;
			ang_diff = min(abs(ang_diff), min(abs(ang_diff + 2 * M_PI), abs(ang_diff - 2 * M_PI)));
		
			// check if same dir
			if(ang_diff > M_PI/2) continue;
		
			double dx = poseStamped.pose.position.x;
			double dy = poseStamped.pose.position.y;

			// check obstacle
			if(costmap[(int)dx][(int)dy+costmap.size()/2] >= OBSTACLE) continue;

			// check closer than obstacle
			if(pose_seq >= nearest_obs_seq - lidar_detect_dist) continue;

			double dist = sqrt((dx-x)*(dx-x) + (dy-y)*(dy-y));

			// not sub path
			if(pose_flag!=1){
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



///////////////////////////////
	// if OBSTACLE_STATIC and find obstacle in look_ahead_radius, then go sub path
	if(task == OBSTACLE_STATIC && go_sub_path){

		cout << "looking sub path!\n";

		// can't find goal, just go straight
		if(value_sub == -1) return Cor(0,1);
			
		// else return sub path
		double gx = goals[value_sub].pose.position.x;
		double gy = goals[value_sub].pose.position.y;
		cout << "current goal seq : " << (goals[value_sub].header.seq >> 4) << endl;
		cout << "current goal flag : " << (goals[value_sub].header.seq & 0b1111) << endl;
		cout << "current goal point : (" << gx << ',' << gy << ")\n";
		return Cor(gx,gy);
	}
///////////////////////////////



	// if we find goal, return
	if(value != -1){
		double gx = goals[value].pose.position.x;
		double gy = goals[value].pose.position.y;
		cout << "current goal seq : " << (goals[value_sub].header.seq >> 4) << endl;
		cout << "current goal flag : " << (goals[value].header.seq & 0b1111) << endl;
		cout << "current goal point : (" << gx << ',' << gy << ")\n";
		return Cor(gx,gy);
	}

	
	if(task == OBSTACLE_SUDDEN){
		return Cor(0,0); 
	}
	
	// we check whether parking or unparking complished by no goal available
	// when parking complished or unparking complished stop!
	else if(motion==PARKING_MOTION){
		cout << "parking_complished : " << parking_complished << endl;
		cout << "unparking_complished : " << unparking_complished << endl;
		// parking complished
		if(parking_complished==false){
			parking_complished = true;
			parking_complished_changed = true;
			return Cor(0,0);
		}
		// unparking complished
		else{
			if (gear_state == 0)
				unparking_complished = true;
			if (gear_state == 1)
				unparking_complished_changed = true;
			return Cor(0,0);
		}
	}
	// please don't go into this condition
	// when there is no goal available stop! ////////////////////////should be checked!!
	else{
		// stop!!!!
		return Cor(0,0);
	}
}
