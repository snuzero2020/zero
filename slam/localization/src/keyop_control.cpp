#include <termios.h>
#include <signal.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include "ros/ros.h"
#include "localization/Keyop.h"

#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_Z 0x7A

#define COMMAND_TIMEOUT_SEC 0.2

// at full joystick depression you'll go this fast
double max_speed = 2.00;
double max_turn = 60.0*M_PI/180.0;

class KEY_Node{
private:
    localization::Keyop keyop;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    double speed;
    double angle;

public:
    KEY_Node(){
        pub_ = n_.advertise<localization::Keyop>("keyop",1);
        speed = 0;
        angle = 0;
    }
    ~KEY_Node(){ }
    void keyboardLoop();
    void stopCar(){
        keyop.key = 'z';
        pub_.publish(keyop);
    }
};


KEY_Node* key;
int kfd = 0;
struct termios cooked, raw;
bool done;
ros::Time init_time;

int main(int argc, char** argv){
    ros::init(argc, argv, "keyop_control");
    ros::Time::init();
    init_time = ros::Time::now();
    KEY_Node key;
    boost::thread t = boost::thread(boost::bind(&KEY_Node::keyboardLoop, &key));
    ros::spin();
    t.interrupt();
    t.join();
    key.stopCar();
    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}

void KEY_Node::keyboardLoop(){
    char keyboard_input;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("w/x : increase/decrease the speed of the car by 1");
    puts("a/d : increase the steering angle of the car by 1, counterclockwise, clockwise, respectively");
    puts("s   : stop the car but, it makes only the spped of the car to zero, not steering angle");
    puts("z   : exit the program, it doesn't work now... we need to debug it..");
    puts("---------------------------");
    puts("        w     ");
    puts("   a    s    d");
    puts("   z    x     ");
    puts("---------------------------");
    
    bool enable = true;
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    while(enable){
        boost::this_thread::interruption_point();
        ros::Duration dr;

        // get the next event from the keyboard
        int num;
        if((num = poll(&ufd, 1, 250)) < 0){
            perror("poll():");
            return;
        }
        else if(num > 0){
            if(read(kfd, &keyboard_input, 1) < 0){
                perror("read():");
                return;
            }
        }
        else continue;

        dr = ros::Time::now() - init_time;
        keyop.header.stamp.sec = dr.sec;
        keyop.header.stamp.nsec = dr.nsec;

	    if(keyboard_input == KEYCODE_W){
		    keyop.key = KEYCODE_W;
            speed += 1.0;
            ROS_INFO("Press Key W, current speed : %.2f, angle : %.2f", speed, angle);
		    pub_.publish(keyop);
	    }
        else if(keyboard_input == KEYCODE_X){
            keyop.key = KEYCODE_X;
            speed -= 1.0;
            ROS_INFO("Press Key X, current speed : %.2f, angle : %.2f", speed, angle);
            pub_.publish(keyop);
        }
        else if(keyboard_input == KEYCODE_A){
            keyop.key = KEYCODE_A;
            angle += 1.0;
            ROS_INFO("Press Key A, current speed : %.2f, angle : %.2f", speed, angle);
            pub_.publish(keyop);
        }
        else if(keyboard_input == KEYCODE_D){
            keyop.key = KEYCODE_D;
            angle -= 1.0;
            ROS_INFO("Press Key D, current speed : %.2f, angle : %.2f", speed, angle);
            pub_.publish(keyop);
        }
        else if(keyboard_input == KEYCODE_S){
            keyop.key = KEYCODE_S;
            speed = 0.0;
            angle = 0.0;
            ROS_INFO("Press Key S, current speed : %.2f, angle : %.2f", speed, angle);
            pub_.publish(keyop);
        }
        else if(keyboard_input == KEYCODE_Z){
            keyop.key = KEYCODE_Z;
            ROS_INFO("Press Key Z, current speed : %.2f, angle : %.2f", speed, angle);
            ROS_INFO("End up the program");
            pub_.publish(keyop);
            enable = false;
        }
        else{
            ROS_INFO("Press wrong input...");
        }
    }
}