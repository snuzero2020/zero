#ifndef DEFS
#define DEFS

#include <iostream>
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

const int SEARCHING_PARKING_SPOT = -1;
enum parkingState{
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
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
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

	// return the first unchecked state
	int get_present_task()
	{
		int idx=0;
		for(bool check : check_list){
			if(check) ++idx;
			else return state_list[idx];
		}
		return -1;
	}
	
	// return next task
	int get_next_task()
	{
		int idx=0;
		for(bool check : check_list){
			if(check) ++idx;
			else break;
		}
		if(idx+1 >= size) return -2;
		else return state_list[idx+1];
	}


	// check the first unchecked state
	void check_prior_task(){
		for(int i = 0; i<size; i++){
			if(check_list[i]) continue;
			else{
				check_list[i] = true;
				break;
			}
		}
	}

	void push_back(int task)
	{
		size++;
		state_list.push_back(task);
		check_list.push_back(false);
	}
};

class Sector_Task
{
	public:
		int sector;
		int task;

		Sector_Task(int _sector, int _task) :sector(_sector), task(_task) {}
};

#endif
