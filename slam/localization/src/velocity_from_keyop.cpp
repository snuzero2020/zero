#include "ros/ros.h"
#include "ros/time.h"
#include "localization/Keyop.h"
#include "geometry_msgs/TwistStamped.h"
#include "localization/FloatStamp.h"

#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_Z 0x7A
#define LINEAR 2.0
#define ANGULAR 0.1

class VelocityFromKeyop{
public:
    VelocityFromKeyop(){
        speed = 0.0;
        angle = 0.0;
        sub_ = n_.subscribe("/keyop", 1, &VelocityFromKeyop::callback, this);    
    }
    void callback(const localization::Keyop::ConstPtr& msg){
        //ROS_INFO("INPUT KEY %c", msg->key);
        switch(msg->key){
            case KEYCODE_W:
                speed += 1.0;
                break;
            case KEYCODE_X:
                speed -= 1.0;
                break;
            case KEYCODE_A:
                angle += 1.0;
                break;
            case KEYCODE_D:
                angle -= 1.0;
                break;
            case KEYCODE_S:
                speed = 0.0;
                angle = 0.0;
                break;
        }
    }
    geometry_msgs::Twist get_velocity(){
        geometry_msgs::Twist rt;
        //ROS_INFO("speed: %.2f, | angle: %.2f", speed, angle);
        rt.linear.x = speed * LINEAR;
        rt.angular.z = angle * ANGULAR;
        return rt;
    }
private:
    double speed;
    double angle;
    ros::NodeHandle n_;
    ros::Subscriber sub_;
};



int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_from_keyop");
    ros::Time::init();
    ros::Time init_time = ros::Time::now();
    ros::Duration dr;
    ros::NodeHandle n_car;
    ros::NodeHandle n_cam;
    ros::Publisher vel_pub = n_car.advertise<geometry_msgs::TwistStamped>("relative_velocity/from_keyop", 1000);
    ros::Publisher cam_vel_pub = n_cam.advertise<localization::FloatStamp>("camera_velocity",1000);
    VelocityFromKeyop VFKobject;
    ros::Rate loop_rate(50);
    while(ros::ok()){
        geometry_msgs::TwistStamped msg;
        localization::FloatStamp msg_cam;
        dr = ros::Time::now() - init_time;
        msg.twist = VFKobject.get_velocity();
        msg.header.stamp.sec = dr.sec; 
        msg.header.stamp.nsec = dr.nsec;
        msg_cam.header.stamp = msg.header.stamp;
        msg_cam.data = 1.0;
        vel_pub.publish(msg);
        cam_vel_pub.publish(msg_cam);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}