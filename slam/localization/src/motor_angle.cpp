#include "ros/ros.h"
#include "ros/time.h"
#include "localization/FloatStamp.h"

class MotorAngle{
public:
    MotorAngle(){
        prv_time = cur_time = 0.0;
        prv_angle = cur_angle = 0.0;
        pub_ = n_.advertise<localization::FloatStamp>("motor_angle",1000);
        sub_ = n_.subscribe("/camera_velocity", 1, &MotorAngle::callback, this);
    }
    void callback(const localization::FloatStamp::ConstPtr& msg){
        localization::FloatStamp rt;
        cur_time = msg->header.stamp.toSec();
        cur_angle = prv_angle + msg->data * (cur_time - prv_time);
        prv_time = cur_time;
        prv_angle = cur_angle;
        rt.header = msg->header;
        rt.data = cur_angle;
        pub_.publish(rt);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double prv_time, cur_time;
    double prv_angle, cur_angle;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_angle");
    MotorAngle MAobject;
    ros::spin();
    return 0;
}