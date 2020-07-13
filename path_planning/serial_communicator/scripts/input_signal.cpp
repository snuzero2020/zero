#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Header.h"
#include "core_msgs/Control.h"
#include "async_comm/serial.h"
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

class Signal {
    public:
        bool is_auto = 1;
        bool estop = 0;
        int brake = 1;
        int gear = 0;
        float speed = 0;
        float steer = 0;

        void callback (const core_msgs::Control& msg){
            is_auto = msg.is_auto;
            estop = msg.estop;
            brake = msg.brake;
            gear = msg.gear;
            speed = msg.speed;
            steer = msg.steer;
        }
};

int main(int argc, char **argv){
    
    ros::init(argc, argv, "input_signal");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<core_msgs::Control>("/calibrated_control", 1000);
    Signal(signal);
    ros::Subscriber sub = nh.subscribe("/car_signal", 1000, &Signal::callback, &signal);
    ros::Rate loop_rate(10);

    //std::cout << "1" << std::endl;

    while (ros::ok()){
        //std::cout << "2" << std::endl;
        core_msgs::Control msg;

        msg.is_auto = signal.is_auto;
        msg.estop = signal.estop;
        msg.gear = signal.gear;
        msg.brake = signal.brake;
        msg.speed = signal.speed;
        msg.steer = signal.steer;
        
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

