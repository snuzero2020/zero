#include <vector>
#include "defs.h"
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
	//private:
	public:
		ros::NodeHandle n;
		ros::Subscriber light_state_sub;
		//ros::Subscriber task_state_sub;
		ros::Subscriber sector_info_sub;
		ros::Publisher mission_state_pub;
		ros::Publisher recommend_vel_pub;

		int light_state;
		double min_weight{0.5};
		double max_weight{1.0};
		double go_sign_threshold{0.5};
		bool debug;

	public:
		Checker sector_pass_checker;
		vector<Checker> checker_container;

		float recommend_vel_info[6] = {1,1,1,1,1,1};
		int buff_length{10};
		vector<int> light_state_buff;
		RosNode(){
			light_state_sub = n.subscribe("light_state", 50, &RosNode::lightstateCallback, this);
			//task_state_sub = n.subscribe("task_state_with_std_vel", 50, &RosNode::taskstateCallback, this);
			sector_info_sub = n.subscribe("/sector_info", 50, &RosNode::sectorInfoCallback, this);
			mission_state_pub = n.advertise<std_msgs::UInt32>("mission_state", 50);
			recommend_vel_pub = n.advertise<std_msgs::Float32>("recommend_vel", 50);

			for(int i = 0 ; i<buff_length; i++) light_state_buff.push_back(0);
			light_state = 0;

			
			vector<int> A_task{DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION
						,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION};	
			vector<int> B_task{INTERSECTION_RIGHT,INTERSECTION_STRAIGHT,INTERSECTION_LEFT};
			vector<int> C_task{INTERSECTION_RIGHT_UNSIGNED,INTERSECTION_RIGHT_UNSIGNED};
			vector<int> D_task{INTERSECTION_RIGHT_UNSIGNED,DRIVING_SECTION};
			vector<int> E_task{PARKING};
			
			
			/*
			vector<int> A_task{DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION
						,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION};	
			vector<int> B_task{INTERSECTION_STRAIGHT_UNSIGNED,INTERSECTION_LEFT_UNSIGNED};
			vector<int> C_task{INTERSECTION_RIGHT_UNSIGNED};
			vector<int> D_task{INTERSECTION_RIGHT_UNSIGNED,DRIVING_SECTION};
			vector<int> E_task{PARKING};

*/
			/*
			vector<int> A_task{DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION
						,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION,DRIVING_SECTION};	
			vector<int> B_task{INTERSECTION_RIGHT_UNSIGNED,INTERSECTION_STRAIGHT_UNSIGNED,INTERSECTION_LEFT_UNSIGNED};
			vector<int> C_task{INTERSECTION_RIGHT_UNSIGNED,INTERSECTION_RIGHT_UNSIGNED};
			vector<int> D_task{INTERSECTION_RIGHT_UNSIGNED,DRIVING_SECTION};
			vector<int> E_task{PARKING};
*/

			checker_container.resize(6, Checker());

//////////////////////////////////////////
			checker_container[A] = Checker(A_task.size());
			checker_container[A].state_list = A_task;
			checker_container[B] = Checker(B_task.size());
			checker_container[B].state_list = B_task;
			checker_container[C] = Checker(C_task.size());
			checker_container[C].state_list = C_task;
			checker_container[D] = Checker(D_task.size());
			checker_container[D].state_list = D_task;
			checker_container[E] = Checker(E_task.size());
			checker_container[E].state_list = E_task;

			//vector<int> sector_order{X,A,D,A,B,A,C,A,B,A,D,A,E};
			//vector<int> sector_order{X,A,B,A,B,A,B,A,B,A,B,A};
			vector<int> sector_order{X,A,B,A,C,A,D,A,B,A,C,A,B,A,D,A,E};
			sector_pass_checker = Checker(sector_order.size());
			sector_pass_checker.state_list = sector_order;

			debug = true;
		}

		inline int isSign(int _light_state, int sign_num) {return ((_light_state)>>sign_num)&1;}

		void lightstateCallback(const std_msgs::UInt32 & msg){
			for(int i{0}; i<light_state_buff.size()-1; i++) light_state_buff[i]=light_state_buff[i+1];
			light_state_buff[light_state_buff.size()-1] = (int)msg.data;
			if(debug) ROS_INFO("light_state : %d",msg.data);
		}

		void sectorInfoCallback(const std_msgs::UInt32 & msg){
			int task_state = task_state_determiner(static_cast<int>(msg.data));
			int motion_state;

			std_msgs::Float32 recommend_vel_msg;
			recommend_vel_msg.data = recommend_vel_info[sector_pass_checker.get_present_task()];
			recommend_vel_pub.publish(recommend_vel_msg);

			light_state_determiner(task_state);
			motion_state_determiner(motion_state,task_state,light_state);

			if(debug) print_debug((int)msg.data, task_state, light_state, motion_state);

			// sector, task, light, motion (each 4 bits)
			std_msgs::UInt32 mission_state;
			mission_state.data =(((int)msg.data)<<12) | (task_state<<8) | (light_state<<4) | motion_state;
			mission_state_pub.publish(mission_state);		
		}

		void light_state_determiner(int task_state){
			double determinant{0};
			bool go_sign{false};
			if (task_state == INTERSECTION_STRAIGHT || task_state == INTERSECTION_LEFT || task_state == INTERSECTION_RIGHT){
				for (int i{0}; i<light_state_buff.size(); ++i){
					switch (task_state){
						case INTERSECTION_STRAIGHT:
							if(isSign(light_state_buff[i],GREEN_LIGHT)) go_sign = true;
							else go_sign = false;
							break;
						case INTERSECTION_LEFT:
							if(isSign(light_state_buff[i],LEFT_LIGHT)) go_sign = true;
							else go_sign = false;
							break;
						case INTERSECTION_RIGHT:
							if(isSign(light_state_buff[i],GREEN_LIGHT)) go_sign = true;
							else go_sign = false;
							break;
					}
					if(go_sign)
						determinant += min_weight + (max_weight-min_weight)*i/(double)(light_state_buff.size()); 
					else	
						determinant -= min_weight + (max_weight-min_weight)*i/(double)(light_state_buff.size());
				}
				if (determinant > go_sign_threshold){
					switch (task_state){
						case INTERSECTION_STRAIGHT:
							light_state = 0x0001;
							break;
						case INTERSECTION_LEFT:
							light_state = 0x0010;
							break;
						case INTERSECTION_RIGHT:
							light_state = 0x0001;
							break;
					}
				}
				else
					light_state = 0x1100;
			}
			else
				light_state = 0;
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
					if(light_state == 0) motion_state = FORWARD_SLOW_MOTION;
					else if(isSign(light_state,GREEN_LIGHT)) motion_state = RIGHT_MOTION;
					else motion_state = HALT_MOTION;
					//motion_state = RIGHT_MOTION;
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
				prev_sector = sector_info;
				cnt = 0;
			}
			else{
				++cnt;
				if(cnt == 20){
					checker_container[sector_pass_checker.get_present_task()].check_prior_task();
					sector_pass_checker.check_prior_task();
					cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
					cout << "!!!!!!!!!1!!!!!check!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
				}
			}

			return task_state;
		}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_recognizer");
	RosNode rosnode;
	ROS_INFO("start");
	
	ros::spin();

/*
	// code for first global way point driving
	/////////////////////////////////////////////////////
	ros::Rate loop(100);
	while(1)
	{
			ros::spinOnce();
			std_msgs::Float32 recommend_vel_msg;
			//recommend_vel_msg.data = rosnode.recommend_vel_info[rosnode.sector_pass_checker.get_present_task()];
			recommend_vel_msg.data = 2;
			rosnode.recommend_vel_pub.publish(recommend_vel_msg);
			std_msgs::UInt32 mission_state;
			mission_state.data = 0;
			rosnode.mission_state_pub.publish(mission_state);		
		loop.sleep();
	}
	/////////////////////////////////////////////////////
*/
	return 0;
}
