#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

class GlobalVelocity{
    public:
    GlobalVelocity(){
        vel = 0.0;
        p_theta = 0.0;
        c_theta = 0.0;
        p_time = 0.0;
        c_time = 0.0;
        pub_ = n_.advertise<geometry_msgs::TwistStamped>("velocity/from_keyop",1000);
        sub_ = n_.subscribe("/relative_velocity/from_keyop", 1, &GlobalVelocity::callback, this);
    }
    void callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        /*
        ROS_INFO("time: %.2f | speed: %.2f | angle: %.2f", msg->header.stamp.toSec(),
                                                           msg->twist.linear.x, 
                                                           msg->twist.angular.z);
        */
        geometry_msgs::TwistStamped rt;
        c_time = msg->header.stamp.toSec();
        c_theta = p_theta + msg->twist.angular.z * (c_time-p_time);
        vel = msg->twist.linear.x;
        p_theta = c_theta;
        p_time = c_time;
        rt.header = msg->header;
        rt.twist.linear.x = vel * cos(c_theta);
        rt.twist.linear.y = vel * sin(c_theta);
        rt.twist.angular.z = c_theta;
        pub_.publish(rt);
    }
    private:
    double vel, p_theta, c_theta;
    double p_time, c_time;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "global_velocity");
    GlobalVelocity GVobject;
    ros::spin();
}