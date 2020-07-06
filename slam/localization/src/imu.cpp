#include "ros/ros.h"
#include "ros/time.h"
#include "localization/Data.h"
#include "localization/Imu.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

class IMU{
    public:
    IMU(){
        pub_ = n_.advertise<localization::Imu>("imu", 1000);
        sub_ = n_.subscribe("data", 1, &IMU::callback, this);
        time = -1;
    }

    void callback(const localization::Data::ConstPtr& msg){
        localization::Imu rt;
        double  t = msg->header.stamp.toSec();
        if(t-time<0.04) return;
        time = t;
        rt.header = msg->header;
        rt.local_ax = msg->local_ax + a(gen);
        rt.local_ay = msg->local_ay + a(gen);
        rt.theta = msg->theta + th(gen);
        rt.omega = msg->omega + o(gen);
        pub_.publish(rt);
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double time;
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<float> a{0, 0.2};
    std::normal_distribution<float> th{0, 1};
    std::normal_distribution<float> o{0, 1};
};

int main(int argc, char **argv){
    ros::init(argc, argv, "imu");
    IMU imuobject;
    ros::spin();
}