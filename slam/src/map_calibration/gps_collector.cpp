#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "slam/Data.h"

#include "ros/ros.h"

using namespace std;


typedef pair<double,double> pdd;


class GPS_Collector{
    public:
    GPS_Collector(){
        sub_ = nh_.subscribe("/filtered_data", 10, &GPS_Collector::callback, this);
        count_ = 0;
    }
    
    void callback(const slam::Data::ConstPtr& msg){
        count_ ++;
        data_x_.push_back(msg->x);
        data_y_.push_back(msg->y);
        data_t_.push_back(msg->theta);
        
        if(count_ % 100 == 0){
            int n = count_ / 100;
            sort(data_x_.begin(),data_x_.end());
            sort(data_y_.begin(), data_y_.end());
            sort(data_t_.begin(), data_t_.end());
            double sum_x = 0, sum_y = 0, sum_t = 0;
            for(int i =10*n;i<90*n;i++){
                sum_x += data_x_.at(i);
                sum_y += data_y_.at(i);
                sum_t += data_t_.at(i);
            }
            sum_x = sum_x /(80*n);
            sum_y = sum_y /(80*n);
            sum_t = sum_t /(80*n);
            
            ROS_INFO("[%5d step]", count_);
            ROS_INFO("current coordinate : (%lf,%lf)", count_, sum_x, sum_y);
            ROS_INFO("current heading : %lf", sum_t);
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int count_;
    vector<double> data_x_;
    vector<double> data_y_;
    vector<double> data_t_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"gps_collector");
    GPS_Collector gps_collector;
    ros::spin();
}
