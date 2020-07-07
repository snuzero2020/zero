#include "ros/ros.h"
#include "ros/time.h"
#include <localization/Data.h>
#include <math.h>

const double PI  =3.141592653589793238463;

int main(int argc, char **argv){
    ros::init(argc, argv, "simulation");
    ros::NodeHandle nh;
    ros::Time::init();
    ros::Time init_time = ros::Time::now();
    ros::Duration dr;
    ros::Publisher data_pub = nh.advertise<localization::Data>("data", 1000);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        localization::Data msg;
        dr = ros::Time::now() - init_time;
        msg.header.stamp.sec = dr.sec; 
        msg.header.stamp.nsec = dr.nsec;
        float t = dr.sec + 1e-9*dr.nsec;
        ROS_INFO("t : %lf", t);
        while(t>=40){
            t = t - 40;
        }
        if(t<10){
            msg.x = t*t;
            msg.y = 0.0;
            msg.theta = 0.0;
            msg.omega = 0.0;
            msg.v = 2*t;
            msg.vx = 2*t;
            msg.vy = 0.0;
            msg.local_ax = 2.0;
            msg.local_ay = 0.0;
        }else if(t<20){
            double pi = PI*(20-t)*(20-t)/100;
            msg.x = 100+100/PI*sin(pi);
            msg.y = 100/PI+100/PI*cos(pi);
            msg.theta = (100-(20-t)*(20-t))/100*PI;
            msg.omega = (20-t)/50*PI;
            msg.v = 2*(20-t);
            msg.vx = msg.v*cos(msg.theta);
            msg.vy = msg.v*sin(msg.theta);
            msg.local_ax = -2.0;
            msg.local_ay = PI/100*msg.v*msg.v;
        }else if(t<30){
            msg.x = 100-(t-20)*(t-20);
            msg.y = 200/PI;
            msg.theta = PI;
            msg.omega = 0;
            msg.v = 2*(t-20);
            msg.vx = -2*(t-20);
            msg.vy = 0;
            msg.local_ax = 2.0;
            msg.local_ay = 0;
        }else{
            double pi = PI*(40-t)*(40-t)/100;
            msg.x = -100/PI*sin(pi);
            msg.y = 100/PI-100/PI*cos(pi);
            msg.theta = (200-(40-t)*(40-t))/100*PI;
            msg.omega = (40-t)/50*PI;
            msg.v = 2*(40-t);
            msg.vx = msg.v*cos(msg.theta);
            msg.vy = msg.v*sin(msg.theta);
            msg.local_ax = -2.0;
            msg.local_ay = PI/100*msg.v*msg.v;
        }
        data_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}