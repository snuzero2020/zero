#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"

class GlobalPose{
    public:
    GlobalPose(){
        prv.linear.x = 0.0;
        prv.linear.y = 0.0;
        prv.angular.z = 0.0;
        cur.linear.x = 0.0;
        cur.linear.y = 0.0;
        cur.angular.z = 0.0;
        p_time = 0.0;
        c_time = 0.0;
        pub_ = n_.advertise<geometry_msgs::TwistStamped>("global_position", 1000);
        sub_ = n_.subscribe("velocity/from_keyop", 1, &GlobalPose::callback, this);
    }
    void callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
        geometry_msgs::TwistStamped rt;
        c_time = msg->header.stamp.toSec();
        cur.linear.x = prv.linear.x + msg->twist.linear.x * (c_time - p_time);
        cur.linear.y = prv.linear.y + msg->twist.linear.y * (c_time - p_time);
        prv = cur;
        p_time = c_time;
        rt.header = msg->header;
        rt.twist = cur;
        rt.twist.angular.z = msg->twist.angular.z;
        pub_.publish(rt);
    }
    private:
    geometry_msgs::Twist prv, cur;
    double p_time, c_time;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "global_position");
    GlobalPose GPobject;
    ros::spin();
}