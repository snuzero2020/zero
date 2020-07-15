#include "ros/ros.h"
#include "ros/time.h"
#include "localization/Imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "UnixtimeToSec.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <math.h>

class IMU_Decoder{
    public:
    IMU_Decoder(){
        pub_ = data_.advertise<localization::Imu>("imu", 1000);
        sub_data_ = data_.subscribe("/imu/data", 1, &IMU_Decoder::callback_data, this);
        sub_mag_ = mag_.subscribe("/imu/mag", 1, &IMU_Decoder::callback_mag, this);
    }
    
    void callback_data(const sensor_msgs::Imu::ConstPtr& msg){
        localization::Imu rt;

        rt.header = msg->header;
        rt.header.stamp.sec = UnixtimeToSec(msg->header.stamp.sec);
        rt.local_ax = msg->linear_acceleration.x;
        rt.local_ay = msg->linear_acceleration.y;
        rt.omega = msg->angular_velocity.z;
        rt.theta = theta_;
        pub_.publish(rt);
    }

    void callback_mag(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
        double theta_mag = std::atan2(msg->vector.x,msg->vector.y);
        theta_ = M_PI/2 - (theta_mag + theta_diff_);
    }

    private:
    ros::NodeHandle data_;
    ros::NodeHandle mag_;
    ros::Publisher pub_;
    ros::Subscriber sub_data_;
    ros::Subscriber sub_mag_;
    double theta_;
    double theta_diff_ = -8.60/180*M_PI;  //difference between magnetic north and truth north
};

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_decoder");
    IMU_Decoder IMUObject;
    ros::spin();
}
