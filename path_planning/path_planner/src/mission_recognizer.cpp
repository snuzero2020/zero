#include <vector>
#include "ros/ros.h"
#include "rrt_star.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

using namespace std;

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
		//ros::Subscriber task_state_sub;
		ros::Subscriber sector_info_sub;
		ros::Publisher mission_state_pub;
		ros::Publisher recommend_vel_pub;
		int light_state;
		bool debug;

	public:
		Checker sector_pass_checker;
		vectpr<Checker> checker_container;

		float recommend_vel_info[] = {3,2,2,2,1};

		RosNode(){
			light_state_sub = n.subscribe("light_state", 50, &RosNode::lightstateCallback, this);
			//task_state_sub = n.subscribe("task_state_with_std_vel", 50, &RosNode::taskstateCallback, this);
			sector_state_sub = n.subscribe("/sector_info", 50, &RosNode::sectorInfoCallback, this);
			mission_state_pub = n.advertise<std_msgs::UInt32>("mission_state", 50);
			recommend_vel_pub = n.advertise<std_msgs::Float32>("recommend_vel", 50);
			light_state = -1;

			vector<int> A_task{DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,
				DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,
				DRIVING_SECTION,DRIVING_SECTION};
			vector<int> B_task{INTERSECTION_RIGHT,INTERSECTION_STRAIGHT,INTERSECTION_LEFT};
			vector<int> C_task{INTERSECTION_RIGHT_UNSIGNED,INTERSECTION_RIGHT_UNSIGNED};
			vector<int> D_task{INTERSECTION_RIGHT_UNSIGNED,DRIVING_SECTION};
			vector<int> E_task{PARKING};

			check_container.resize(5);

			check_container[0] = Checker(A_task.size());
			check_container[0].state = A_task;
			check_container[1] = Checker(B_task.size());
			check_container[1].state = B_task;
			check_container[2] = Checker(C_task.size());
			check_container[2].state = C_task;
			check_container[3] = Checker(D_task.size());
			check_container[3].state = D_task;
			check_container[4] = Checker(E_task.size());
			check_container[4].state = E_task;

			vector<int> sector_order{A,B,A,C,A,D,A,B,A,C,A,B,A,D,A,E};
			sector_pass_checker = Checker(sector_order.size());
			sector_pass_checker.state = sector_order;

			debug = false;
		}

		inline int isSign(int _light_state, int sign_num) {return ((_light_state)>>sign_num)&1;}

		void lightstateCallback(const std_msgs::UInt32 & msg){
			light_state = (int)msg.data;
			if(debug) ROS_INFO("light_state : %d",msg.data);
		}

		void sectorInfoCallback(const std_msgs::UInt32 & msg){

			int task_state = task_state_determiner(static_cast<int>(msg.data));
			float recommend_vel = recommend_vel_info[msg.data];

			if(debug) ROS_INFO("task_state : %d",msg.data);

			int motion_state;
			moiton_state_determiner(motion_state,task_state,light_state);

			
			std_msgs::UInt32 mission_state;
			mission_state.data =(((int)recommend_vel*4)<<12) | (task_state<<8) | (light_state<<4) | motion_state;
			mission_state_pub.publish(mission_state);		
		}

		void motion_state_determiner(int &motion_state, int task_state, int light_state){

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
		}

		int task_state_determiner(int sector_info)
		{
			static int cnt = 0;
			static int prev_sector = -1;

			int task_state = checker_container[sector_info].get_present_task();
			if(prev_sector != sector_info){
				sector_info = prev_sector;
				cnt = 0;
			}
			else{
				cnt++;
				if(cnt == 20){
					check_container[sector_pass_checker.get_present_task()].check_prior_task();
					sector_pass_checker.check_prior_task();
				}
			}
			return task_state;
		}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_recognizer");
	RosNode rosnode;
	ros::spin();
	return 0;
}
