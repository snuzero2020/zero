#include "ros/ros.h"
#include "ros/time.h"
#include "localization/Data.h"
#include "std_msgs/Float32.h"
#include <iostream>

const double PI  =3.141592653589793238463;

class Evaluator{
    public:
    Evaluator(){
        pub_0 = n_.advertise<localization::Data>("err", 1000);
        pub_1 = n_.advertise<std_msgs::Float32>("delay", 1000);
        sub_0 = n_.subscribe("data", 1, &Evaluator::callback_0, this);
        sub_1 = n_.subscribe("filtered_data", 1, &Evaluator::callback_1, this);
        reset_err();
    }

    void callback_0(const localization::Data& msg){
        data=msg;
    }

    void callback_1(const localization::Data& msg){
        delay.data += (msg.header.stamp - data.header.stamp).toSec();
        err.x += (data.x - msg.x)*(data.x - msg.x);
        err.y += (data.y - msg.y)*(data.y - msg.y);
        err.theta += remainderf((data.theta - msg.theta),2*PI)*remainderf((data.theta - msg.theta),2*PI);
        err.omega += (data.omega - msg.omega)*(data.omega - msg.omega);
        err.local_ax += (data.local_ax - msg.local_ax)*(data.local_ax - msg.local_ax);
        err.local_ay += (data.local_ay - msg.local_ay)*(data.local_ay - msg.local_ay);
        err.v += (data.v - msg.v)*(data.v - msg.v);
        err.vx += (data.vx - msg.vx)*(data.vx - msg.vx);
        err.vy += (data.vy - msg.vy)*(data.vy - msg.vy);
        n += 1;

        if(msg.header.stamp.toSec()>5*(iter+1)){
            delay.data/=n;
            pub_1.publish(delay);
            err.header=msg.header;
            err.x=sqrt(err.x/n);
            err.y=sqrt(err.y/n);
            err.theta=sqrt(err.theta/n);
            err.omega=sqrt(err.omega/n);
            err.local_ax=sqrt(err.local_ax/n);
            err.local_ay=sqrt(err.local_ay/n);
            err.v=sqrt(err.v/n);
            err.vx=sqrt(err.vx/n);
            err.vy=sqrt(err.vy/n);
            pub_0.publish(err);
            reset_err();
            n=0;
            iter+=1;   
        }
        
    }

    void reset_err(){
        delay.data = 0;
        err.x = 0;
        err.y = 0;
        err.theta = 0;
        err.omega = 0;
        err.local_ax = 0;
        err.local_ay = 0;
        err.v = 0;
        err.vx = 0;
        err.vy = 0;
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_0;
    ros::Publisher pub_1;
    ros::Subscriber sub_0;
    ros::Subscriber sub_1;
    localization::Data data;
    localization::Data err;
    std_msgs::Float32 delay;
    double iter = 0;
    double n = 0;

};

int main(int argc, char **argv){
    ros::init(argc, argv, "imu");
    Evaluator evalobject;
    ros::spin();
}