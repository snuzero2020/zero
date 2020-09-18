#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"

#include "ros/ros.h"

using namespace std;

class IMU_Collector{
    public:
    IMU_Collector(){
        sub_ = nh_.subscribe("/imu/data", 10, &IMU_Collector::callback, this);
        count_ = 0;
    }
    
    void callback(const sensor_msgs::Imu::ConstPtr& msg){
        count_ ++;
        geometry_msgs::Quaternion q = msg->orientation;
        data_roll_.push_back(std::atan2( 2*(q.x*q.w+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y) ));
        data_pitch_.push_back(asin(2*(q.y*q.w-q.x*q.z)));
        data_yaw_.push_back(std::atan2( 2*(q.x*q.y+q.z*q.w), 1-2*(q.y*q.y+q.z*q.z) ));
        
        if(count_ % 100 == 0){
            int n = count_ / 100;
            sort(data_roll_.begin(),data_roll_.end());
            sort(data_pitch_.begin(), data_pitch_.end());
            sort(data_yaw_.begin(), data_yaw_.end());
            double sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
            for(int i =10*n;i<90*n;i++){
                sum_roll += data_roll_.at(i);
                sum_pitch += data_pitch_.at(i);
                sum_yaw += data_yaw_.at(i);
            }
            sum_roll = sum_roll /(80*n);
            sum_pitch = sum_pitch /(80*n);
            sum_yaw = sum_yaw /(80*n);
            
            ROS_INFO("[%5d step]", count_);
            ROS_INFO("current roll, pitch : (%lf,%lf)", sum_roll, sum_pitch);
            ROS_INFO("current yaw : %lf", sum_yaw);
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int count_;
    vector<double> data_roll_;
    vector<double> data_pitch_;
    vector<double> data_yaw_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"IMU_collector");
    IMU_Collector IMU_collector;
    ros::spin();
}
