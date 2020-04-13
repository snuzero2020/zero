#include "ros/ros.h"
#include "ros/time.h"
#include "localization/FloatStamp.h"
#include "geometry_msgs/TwistStamped.h"

class DepthCamera{
public:
    DepthCamera(){
        sub_motor_ = motor_.subscribe("/motor_angle", 1, &DepthCamera::call_motor, this);
        sub_car_ = car_.subscribe("/global_position", 1, &DepthCamera::call_car, this);    
    }
    void call_motor(const localization::FloatStamp::ConstPtr& msg){
        cur_motor.header = msg->header;
        cur_motor.data = msg->data;
    }
    void call_car(const geometry_msgs::TwistStamped::ConstPtr& msg){
        cur_car.header = msg->header;
        cur_car.twist = msg->twist;
    }
    localization::FloatStamp get_motor() { return cur_motor; }
    geometry_msgs::TwistStamped get_car() { return cur_car; }
private:
    ros::NodeHandle motor_;
    ros::NodeHandle car_;
    ros::Subscriber sub_motor_;
    ros::Subscriber sub_car_;
    localization::FloatStamp cur_motor;
    geometry_msgs::TwistStamped cur_car;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "depth_camera");
    DepthCamera DCobject;
    ros::NodeHandle n;
    ros::Publisher pub_ = n.advertise<geometry_msgs::TwistStamped>("depth_camera", 1000);
    ros::Rate loop_rate(50);
    while(ros::ok()){
        geometry_msgs::TwistStamped msg;
        msg = DCobject.get_car();
        msg.twist.angular.z += DCobject.get_motor().data;
        pub_.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}