#include "ros/ros.h"
#include "ros/time.h"
#include "localization/Data.h"
#include "localization/Gps.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

class GPS{
    public:
    GPS(){
        pub_ = n_.advertise<localization::Gps>("gps", 1000);
        sub_ = n_.subscribe("data", 1, &GPS::callback, this);
        time = -1;
    }

    void callback(const localization::Data::ConstPtr& msg){
        localization::Gps rt;
        double  t = msg->header.stamp.toSec();
        if(t-time<0.1) return;
        time = t;
        rt.header = msg->header;
        rt.x = msg->x + d(gen);
        rt.y = msg->y + d(gen);
        pub_.publish(rt);
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double time;
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<float> d{0,1}; 
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gps");
    GPS gpsobject;
    ros::spin();
}