#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "nmea_msgs/Sentence.h"

#include "slam/Gps.h"

#include "ros/ros.h"

using namespace std;


typedef pair<double,double> pdd;


class GPS_Collector{
    public:
    GPS_Collector(){
        sub_ = nh_.subscribe("/gps", 10, &GPS_Collector::callback, this);
        count_ = 0;
    }
    
    void callback(const slam::Gps::ConstPtr& msg){
        count_ ++;
        data_x_.push_back(msg->x);
        data_y_.push_back(msg->y);
        
        if(count_ % 100 == 0){
            int n = count_ / 100;
            sort(data_x_.begin(),data_x_.end());
            sort(data_y_.begin(), data_y_.end());
            double sum_x = 0, sum_y = 0;
            for(int i =10*n;i<90*n;i++){
                sum_x += data_x_.at(i);
                sum_y += data_y_.at(i);
            }
            sum_x = sum_x /(80*n);
            sum_y = sum_y /(80*n);

            ROS_INFO("Current Coordinate (%5d step) : (%lf,%lf)", count_, sum_x, sum_y);
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int count_;
    vector<double> data_x_;
    vector<double> data_y_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"gps_collector");
    GPS_Collector gps_collector;
    ros::spin();
}
