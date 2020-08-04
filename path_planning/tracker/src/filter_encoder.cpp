#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include "core_msgs/VehicleState.h"
#include "core_msgs/Encoderfilter.h"

class Filter{

    public: 
        ros::NodeHandle nh;

        double encoder_data[3];
        double time[3];
        int check = 0;
        double threshold = 0.1;

        ros::Publisher filter_encoder_pub;
        ros::Subscriber encoder_sub;

        Filter(){
            filter_encoder_pub = nh.advertise<core_msgs::Encoderfilter>("filter_encoder_data", 1000);
            encoder_sub = nh.subscribe("/vehicle_state",100, &Filter::encoder_sub_callback, this);
        }

        void encoder_sub_callback(const core_msgs::VehicleState::ConstPtr);
        void determind_filter_encoder();

};

void Filter::encoder_sub_callback(const core_msgs::VehicleState::ConstPtr msg){
    
    //std::cout << "speed : " << msg->speed << std::endl;

    encoder_data[0] = encoder_data[1];
    encoder_data[1] = encoder_data[2];
    encoder_data[2] = msg->speed;
    time[0] = time[1];
    time[1] = time[2];
    time[2] = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
    //std::cout << msg->header.stamp.sec << "  "  << msg->header.stamp.nsec / 1000000000.0 << std::endl;

}

void Filter::determind_filter_encoder(){
    if (check == 0){
        if (encoder_data[0] - encoder_data[1] > threshold){
            if(encoder_data[2] - encoder_data[1] > threshold){
                encoder_data[1] = (encoder_data[0] + encoder_data[2])/2.0;
            }
        }
    }

    core_msgs::Encoderfilter msg;
    msg.filtered_encoder = encoder_data[1];
    if (abs(time[2] - time[0]) > 0.001){
        msg.slope = (encoder_data[2] - encoder_data[0]) / (time[2] - time[0]);
    }
    else{
        msg.slope = (encoder_data[2] - encoder_data[0]) / 0.001;
    }

    std::cout << msg.slope << std::endl;
    msg.time = time[1];

    filter_encoder_pub.publish(msg);
    //std::cout << "filtered_encoder : " << msg.time << std::endl;
}

int main(int argc, char *argv[])
{

	ros::init(argc,argv,"filter_encoder");
	ros::NodeHandle nh;
    
    Filter filter{Filter()};
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
        filter.determind_filter_encoder();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}