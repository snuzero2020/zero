#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "slam/Gps.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

typedef pair<double,double> pdd;


class IMU_Collector{
    public:
    IMU_Collector(){
        sub_data = nh_.subscribe("/imu/data", 10, &IMU_Collector::callback_data, this);
        sub_mag = nh_.subscribe("/imu/mag", 10, &IMU_Collector::callback_mag, this);
        count_ = -20;
        countm_ = -20;
    }
    
    void callback_data(const sensor_msgs::Imu::ConstPtr& msg){
        count_ ++;
        if(count_<0){
            return ;
        }

        if(count_ % 4000 == 0){
            double a_H = sqrt(msg->linear_acceleration.x*msg->linear_acceleration.x + msg->linear_acceleration.y*msg->linear_acceleration.y);
            double a_V = msg->linear_acceleration.z;
            ROS_INFO("a_H : %lf, a_V : %lf, a : %lf", a_H, a_V, sqrt(a_H*a_H+a_V*a_V));
        }

        // data_x_.push_back(msg->linear_acceleration.x);
        // data_y_.push_back(msg->linear_acceleration.y);
        // data_z_.push_back(msg->linear_acceleration.z);
        
        // if(count_ % 100 == 0){
        //     int n = count_ / 100;
        //     sort(data_x_.begin(),data_x_.end());
        //     sort(data_y_.begin(), data_y_.end());
        //     sort(data_z_.begin(), data_z_.end());
        //     double sum_x = 0, sum_y = 0, sum_z = 0;
        //     for(int i =10*n;i<90*n;i++){
        //         sum_x += data_x_.at(i);
        //         sum_y += data_y_.at(i);
        //         sum_z += data_z_.at(i);
        //     }
        //     sum_x = sum_x /(80*n);
        //     sum_y = sum_y /(80*n);
        //     sum_z = sum_z /(80*n);

        //     ROS_INFO("Current Data (%5d step) : (%lf,%lf,%lf)", count_, sum_x, sum_y, sum_z);
        // }
    }

    void callback_mag(const sensor_msgs::MagneticField::ConstPtr& msg){
        countm_ ++;
        if(countm_<0){
            return ;
        }

        if(countm_ % 4000 == 0){
            double mag_H = sqrt(msg->magnetic_field.x*msg->magnetic_field.x + msg->magnetic_field.y*msg->magnetic_field.y);
            double mag_V = msg->magnetic_field.z;
            ROS_INFO("mag_H : %lf, mag_V : %lf, mag : %lf", mag_H, mag_V, sqrt(mag_H*mag_H+mag_V*mag_V));
        }

        // mag_x_.push_back(msg->magnetic_field.x);
        // mag_y_.push_back(msg->magnetic_field.y);
        // mag_z_.push_back(msg->magnetic_field.z);

        // if(countm_ % 100 == 0){
        //     int n = countm_ / 100;
        //     sort(mag_x_.begin(),mag_x_.end());
        //     sort(mag_y_.begin(), mag_y_.end());
        //     sort(mag_z_.begin(), mag_z_.end());
        //     double sum_x = 0, sum_y = 0, sum_z = 0;
        //     for(int i =10*n;i<90*n;i++){
        //         sum_x += mag_x_.at(i);
        //         sum_y += mag_y_.at(i);
        //         sum_z += mag_z_.at(i);
        //     }
        //     sum_x = sum_x /(80*n);
        //     sum_y = sum_y /(80*n);
        //     sum_z = sum_z /(80*n);

        //     ROS_INFO("Current Mag (%5d step) : (%lf,%lf,%lf)", count_, sum_x, sum_y, sum_z);
        // }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_data;
    ros::Subscriber sub_mag;
    int count_;
    int countm_;
    vector<double> data_x_;
    vector<double> data_y_;
    vector<double> data_z_;
    vector<double> mag_x_;
    vector<double> mag_y_;
    vector<double> mag_z_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"imu_collector");
    IMU_Collector imu_collector;
    ros::spin();
}
