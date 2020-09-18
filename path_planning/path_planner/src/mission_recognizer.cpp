#include <vector>
#include "defs.h"
#include "ros/ros.h"
#include "rrt_star.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
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
		ros::Subscriber parking_complished_sub;
		ros::Publisher mission_state_pub;
		//ros::Publisher recommend_vel_pub;

		int light_state;
		double min_weight{0.5};
		double max_weight{1.0};
		double go_sign_threshold{0.5};

	public:
		Checker sector_pass_checker;
		vector<Checker> checker_container;
		vector<Sector_Task> origin_sector_task_order;
		vector<Sector_Task> sector_task_order;
		
		int mission_start;
		int isKcity;
		
		float recommend_vel_info[13] = {1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5};
		int buff_length{10};
		vector<int> light_state_buff;
		RosNode(){
			light_state_sub = n.subscribe("light_state", 2, &RosNode::lightstateCallback, this);
			//task_state_sub = n.subscribe("task_state_with_std_vel", 50, &RosNode::taskstateCallback, this);
			sector_info_sub = n.subscribe("/sector_info", 2, &RosNode::sectorInfoCallback, this);
			parking_complished_sub = n.subscribe("parking_complished", 2, &RosNode::parkingcomplishedCallback, this);
			mission_state_pub = n.advertise<std_msgs::UInt32>("mission_state", 2);
			//recommend_vel_pub = n.advertise<std_msgs::Float32>("recommend_vel", 2);

			for(int i = 0 ; i<buff_length; i++) light_state_buff.push_back(0);
			light_state = 0;

			ros::param::get("/mission_start", mission_start);
			ros::param::get("/isKcity", isKcity);

			
			if (isKcity){
				origin_sector_task_order={
					Sector_Task(X,0),
                                        Sector_Task(A,PARKING),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(B,INTERSECTION_LEFT_UNSIGNED),
                                        Sector_Task(A,OBSTACLE_STATIC),
                                        Sector_Task(D,INTERSECTION_RIGHT_UNSIGNED),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(E,INTERSECTION_STRAIGHT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(F,INTERSECTION_LEFT),
                                      	Sector_Task(A,OBSTACLE_SUDDEN),
                                      	Sector_Task(G,INTERSECTION_RIGHT),
					Sector_Task(A,OBSTACLE_STATIC),
                                        Sector_Task(H,INTERSECTION_STRAIGHT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(J,INTERSECTION_LEFT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(I,INTERSECTION_LEFT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(H,INTERSECTION_RIGHT),
					Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(G,INTERSECTION_STRAIGHT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(E,INTERSECTION_STRAIGHT),
                                        Sector_Task(A,DRIVING_SECTION),
                                        Sector_Task(B,INTERSECTION_STRAIGHT_UNSIGNED),
                                        Sector_Task(A,DRIVING_SECTION)
				};
			}
			else{
				origin_sector_task_order={
					
					Sector_Task(X,0),
					//Sector_Task(A,DRIVING_SECTION),
					Sector_Task(A,OBSTACLE_STATIC),
					//Sector_Task(A,OBSTACLE_SUDDEN),
					Sector_Task(D,INTERSECTION_RIGHT),
					Sector_Task(A,DRIVING_SECTION),
					Sector_Task(C,INTERSECTION_RIGHT_UNSIGNED),
					Sector_Task(A,DRIVING_SECTION),
					Sector_Task(B,INTERSECTION_RIGHT_UNSIGNED),
					//Sector_Task(A,OBSTACLE_STATIC),
					Sector_Task(A,DRIVING_SECTION),
					Sector_Task(D,INTERSECTION_STRAIGHT),
					Sector_Task(A,DRIVING_SECTION),
					Sector_Task(C,INTERSECTION_RIGHT_UNSIGNED),
					Sector_Task(A,DRIVING_SECTION),
					Sector_Task(D,INTERSECTION_LEFT),
					//Sector_Task(A,DRIVING_SECTION),
					Sector_Task(A,OBSTACLE_SUDDEN),
					Sector_Task(B,DRIVING_SECTION),
					Sector_Task(A,PARKING),
					Sector_Task(A,DRIVING_SECTION)
				
				};
			}
/*
			vector<int> passed_sector_count;
			int sector_cnt{0};
			int temp_sector{A};

			while(sector_cnt != mission_start){
				for (int seq{1}; seq < mission_start; ++seq){
					if (sector_order[seq].sector == temp_sector)
						sector_cnt++;
				}
				passed_sector_count.push_back(sector_cnt);
				sector_cnt = 0;
				temp_sector++;
			}
*/
			if(mission_start!=0)sector_task_order.push_back(Sector_Task(X,0));
			for(int i = mission_start;i<origin_sector_task_order.size();i++){
				sector_task_order.push_back(Sector_Task(origin_sector_task_order[i].sector,origin_sector_task_order[i].task));
			}


			sector_pass_checker = Checker();
			for (int sector_seq{0}; sector_seq < sector_task_order.size(); ++sector_seq)
				sector_pass_checker.push_back(sector_task_order[sector_seq].sector);
			

			checker_container.resize(15, Checker());

//////////////////////////////////////////
			for (int mission_seq{0}; mission_seq < sector_task_order.size(); mission_seq++)
				checker_container[sector_task_order[mission_seq].sector].push_back(sector_task_order[mission_seq].task);

		}

		inline int isSign(int _light_state, int sign_num) {return ((_light_state)>>sign_num)&1;}

		void lightstateCallback(const std_msgs::Int32 & msg){
			for(int i{0}; i<light_state_buff.size()-1; i++) light_state_buff[i]=light_state_buff[i+1];
			light_state_buff[light_state_buff.size()-1] = (int)msg.data;
		}

		void parkingcomplishedCallback(const std_msgs::UInt32 & msg){
			checker_container[sector_pass_checker.get_present_task()].check_prior_task();
			sector_pass_checker.check_prior_task();
		
		}
		void sectorInfoCallback(const std_msgs::Int32 & msg){
			int light = light_state_buff[light_state_buff.size()-1];
			printf("light : %s%s%s%s", 
					isSign(light, 3)?"\x1b[41m      \x1b[0m":"      ",
					isSign(light, 2)?"\x1b[43m      \x1b[0m":"      ",
					isSign(light, 1)?"\x1b[32m /___ \x1b[0m":"      ",
					isSign(light, 0)?"\x1b[42m      \x1b[0m":"      "
				);
			printf("light : %s%s%s%s", 
					isSign(light, 3)?"\x1b[41m      \x1b[0m":"      ",
					isSign(light, 2)?"\x1b[43m      \x1b[0m":"      ",
					isSign(light, 1)?"\x1b[32m \\    \x1b[0m":"      ",
					isSign(light, 0)?"\x1b[42m      \x1b[0m":"      "
				);

			int first_unchecked_sector = sector_pass_checker.get_present_task();
			int second_unchecked_sector = sector_pass_checker.get_next_task();
			printf("first_unchecked_sector : %d\tsecond_unchecked_sector : %d\n",first_unchecked_sector, second_unchecked_sector);

			if((msg.data != -1) && ((msg.data&0b1111) != first_unchecked_sector) && ((msg.data&0b1111) != second_unchecked_sector)){
				/////////////////
				printf("\x1b[41m\n\x1b[0m");
				printf("\x1b[41m\n\x1b[0m");
				printf("\x1b[41m\n\x1b[0m");
				printf("unexpected strange undesireable weird sector : %d",msg.data);
				printf("\x1b[41m\n\x1b[0m");
				printf("\x1b[41m\n\x1b[0m");
				printf("\n\x1b[0m");
			}


			// print sector order and present sector
			
			for(int i = 0;i<sector_pass_checker.size;i++){
				if(sector_pass_checker.check_list[i]) printf("\x1b[42m");
				else printf("\x1b[0m");
				int sec = sector_pass_checker.state_list[i];
				if(sec==X) printf("X ");
				else printf("%c ",'A'+sec);
			}
			printf("\n\n");

			// print each task
			
			for(int i = 0;i<sector_pass_checker.size;i++){
				if(sector_pass_checker.check_list[i]) printf("\x1b[42m");
				else printf("\x1b[0m");
				printf("sector : %c  ", (sector_task_order[i].sector==X)?'X':sector_task_order[i].sector+'A');
				printf("     ");
				int task = sector_task_order[i].task;
				switch(task){
					case DRIVING_SECTION : printf("task : DRIVING_SECTION");
							       break;
					case INTERSECTION_STRAIGHT : printf("task : INTERSECTION_STRAIGHT");
								     break;
					case INTERSECTION_LEFT : printf("task : INTERSECTION_LEFT");
								 break;
					case INTERSECTION_RIGHT : printf("task : INTERSECTION_RIGHT");
								  break;
					case INTERSECTION_STRAIGHT_UNSIGNED : printf("task : INTERSECTION_STRAIGHT_UNSIGNED");
									      break;
					case INTERSECTION_LEFT_UNSIGNED : printf("task : INTERSECTION_LEFT_UNSIGNED");
									  break;
					case INTERSECTION_RIGHT_UNSIGNED : printf("task : INTERSECTION_RIGHT_UNSIGNED");
									   break;
					case OBSTACLE_STATIC : printf("task : OBSTACLE_STATIC");
							       break;
					case OBSTACLE_SUDDEN : printf("task : OBSTACLE_SUDDEN");
							       break;
					case CROSSWALK : printf("task : CROSSWALK");
							 break;
					case PARKING : printf("task : PARKING");
						       break;
				}
				printf("\x1b[0m\n");						
			}
			printf("\n");

			int task_state;
			if(msg.data == -1){

				task_state = task_state_determiner(-1);

			}
			else{
				task_state = task_state_determiner(static_cast<int>(msg.data));
			}
			
			int motion_state;
			bool _2far2return{(msg.data != -1) && ((msg.data & 0b10000) == 0b10000)};

			/*
			std_msgs::Float32 recommend_vel_msg;
			recommend_vel_msg.data = recommend_vel_info[sector_pass_checker.get_present_task()];
			recommend_vel_pub.publish(recommend_vel_msg);
			*/

			light_state_determiner(task_state,_2far2return);
			motion_state_determiner(motion_state,task_state,light_state);

			print_debug(task_state, motion_state);

			// sector, task, light, motion (each 4 bits)
			std_msgs::UInt32 mission_state;
			mission_state.data =(((int)msg.data)<<12) | (task_state<<8) | (light_state<<4) | motion_state;
			cout << " light : " << light_state << " (mission_recognizer)\n";
			mission_state_pub.publish(mission_state);		
		}

		void light_state_determiner(int task_state, bool _2far2return){
			if(_2far2return){
				if(task_state == INTERSECTION_STRAIGHT){
					light_state = 0b0001;
				}
				else if(task_state == INTERSECTION_LEFT){
					light_state = 0b0010;
				}
				else if(task_state == INTERSECTION_RIGHT){
					light_state = 0b0001;
				}
				else{
					light_state = 0b0000;
				}
				return;
			}
			
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
							if(isSign(light_state_buff[i],GREEN_LIGHT) || isSign(light_state_buff[i],RED_LIGHT)) go_sign = true;
							else go_sign = false;
							break;
					}
					if(go_sign)
						determinant += min_weight + (max_weight-min_weight)*i/(double)(light_state_buff.size()); 
					else	
						determinant -= min_weight + (max_weight-min_weight)*i/(double)(light_state_buff.size());
				}

				cout << "determinant vs go_sign_threshold " << determinant << " vs " << go_sign_threshold << endl;
				
				if (determinant > go_sign_threshold){
					switch (task_state){
						case INTERSECTION_STRAIGHT:
							light_state = 0b0001;
							break;
						case INTERSECTION_LEFT:
							light_state = 0b0010;
							break;
						case INTERSECTION_RIGHT:
							light_state = 0b0001;
							break;
					}
				}
				else{
					light_state = 0b1100;
				}
			}
			else
				light_state = 0;
		}


		void print_debug(int task, int motion){
			/*switch(sector){
				case A : printf("sector : A");
					break;
				case B : printf("sector : B");
					break;
				case C : printf("sector : C");
					break;
				case D : printf("sector : D");
					break;
				case E : printf("sector : E");
					break;
				case X : printf("sector : X");
					break;
			}
*/
			switch(task){
				case DRIVING_SECTION : printf("task : \t DRIVING_SECTION");
					break;
				case INTERSECTION_STRAIGHT : printf("task : \t INTERSECTION_STRAIGHT");
					break;
				case INTERSECTION_LEFT : printf("task : \t INTERSECTION_LEFT");
					break;
				case INTERSECTION_RIGHT : printf("task : \t INTERSECTION_RIGHT");
					break;
				case INTERSECTION_STRAIGHT_UNSIGNED : printf("task : \t INTERSECTION_STRAIGHT_UNSIGNED");
					break;
				case INTERSECTION_LEFT_UNSIGNED : printf("task : \t INTERSECTION_LEFT_UNSIGNED");
					break;
				case INTERSECTION_RIGHT_UNSIGNED : printf("task : \t INTERSECTION_RIGHT_UNSIGNED");
					break;
				case OBSTACLE_STATIC : printf("task : \t OBSTACLE_STATIC");
					break;
				case OBSTACLE_SUDDEN : printf("task : \t OBSTACLE_SUDDEN");
					break;
				case CROSSWALK : printf("task : \t CROSSWALK");
					break;
				case PARKING : printf("task : \t PARKING");
					break;
			}
			printf("\n");

/*
			printf("light : %s%s%s%s", 
					isSign(light, 3)?"\x1b[41m  \x1b[0m":"    ",
					isSign(light, 2)?"\x1b[43m  \x1b[0m":"    ",
					isSign(light, 1)?"\x1b[32m /____\x1b[0m":"    ",
					isSign(light, 0)?"\x1b[42m  \x1b[0m":"    "
				);
			printf("light : %s%s%s%s", 
					isSign(light, 3)?"\x1b[41m  \x1b[0m":"    ",
					isSign(light, 2)?"\x1b[43m  \x1b[0m":"    ",
					isSign(light, 1)?"\x1b[32m \\   \x1b[0m":"    ",
					isSign(light, 0)?"\x1b[42m  \x1b[0m":"    "
				);
*//*
			if(isSign(light, 0)) printf("light : GREEN_LIGHT");
			if(isSign(light, 1)) printf("light : LEFT_LIGHT");
			if(isSign(light, 2)) printf("light : YELLOW_LIGHT");
			if(isSign(light, 3)) printf("light : RED_LIGHT");
*/
			switch(motion){
				case FORWARD_MOTION : printf("motion : FORWARD_MOTION");
					break;
				case FORWARD_SLOW_MOTION : printf("motion : FORWARD_SLOW_MOTION");
					break;
				case HALT_MOTION : printf("motion : HALT_MOTION");
					break;
				case LEFT_MOTION : printf("motion : LEFT_MOTION");
					break;
				case RIGHT_MOTION : printf("motion : RIGHT_MOTION");
					break;
				case PARKING_MOTION : printf("motion : PARKING_MOTION");
					break;
			}
			printf("\n");
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
			if(sector_info == -1) sector_info = prev_sector;
			bool _2far2return = ((sector_info & 0b10000) == 0b10000);
			sector_info &= 0b1111;
			int task_state = checker_container[sector_info].get_present_task();

			printf("prev_sector : %c\t sector_info : %c%c\t cnt : %d\n",
					((prev_sector==-1)?'X' :('A'+prev_sector)),
					('A'+sector_info),
					(_2far2return) ? '\'' : ' ',
					cnt
				);

			if(prev_sector != sector_info){
				prev_sector = sector_info;
				cnt = 0;
			}
			else{
				++cnt;
				if(cnt == 20){
					checker_container[sector_pass_checker.get_present_task()].check_prior_task();
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
	printf("start");
	
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
