
#include "ros/ros.h"
#include "ros/time.h"
#include "localization/FloatStamp.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_motor");
  ros::Time::init();
  ros::Time init_time = ros::Time::now();
  ros::NodeHandle n;
  ros::Publisher camera_motor_pub = n.advertise<localization::FloatStamp>("motor", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    localization::FloatStamp msg;
    ros::Duration dr;
    msg.data = M_PI;
    dr = ros::Time::now() - init_time;
    msg.header.stamp.sec = dr.sec;
    msg.header.stamp.nsec = dr.nsec;
    //ROS_INFO("data :%f", msg.data);
    camera_motor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}