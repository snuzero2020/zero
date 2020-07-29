#ifndef DEFS
#define DEFS

#include <vector>
using namespace std;

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

enum sector
{
    A,
    B,
    C,
    D,
    E,

// for sector_pass_checker's first check, (it is not real sector)
    X
};

class Checker
{
    public:
        vector<int> state_list;
        vector<bool> check_list;
	int size;
		
	Checker(){Checker(0);}

        Checker(int _size)
        :check_list(vector<bool>(_size,false)), state_list(vector<int>(_size,0)), size(_size) {}

		int get_present_task()
		{
			int idx=0;
			for(bool check : check_list){
				if(check) ++idx;
				else return state_list[idx];
			}
			return -1;
		}

		void check_prior_task(){
			for(int i = 0; i<size; i++){
				if(check_list[i]) continue;
				else{
					check_list[i] = true;
					break;
				}
			}
		}
};

#endif
