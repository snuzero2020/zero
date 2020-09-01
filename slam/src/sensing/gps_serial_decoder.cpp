#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "sensor_msgs/NavSatFix.h"
#include "slam/Gps.h"

#include "ros/ros.h"

#include "UTM.h"

using namespace std;

class GPS_Decoder{
    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bool is_fail_mode;
    int fail_count;
    ros::Time latest_mode_time;
    
    public:
    GPS_Decoder(){
        pub_ = n_.advertise<slam::Gps>("/gps", 2);
        sub_ = n_.subscribe("/fix", 2, &GPS_Decoder::callback, this);
        is_fail_mode = false;
        fail_count = 0;
        latest_mode_time = ros::Time::now();
    }	

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        ros::Time t0 = ros::Time::now();
        slam::Gps rt;
        try{
            if( (is_fail_mode)==(msg->status.status<1) ){
                latest_mode_time = t0;
            }else if( (t0-latest_mode_time).toSec()>10.0 ){
                is_fail_mode = !(is_fail_mode);
                if(is_fail_mode){
                    fail_count++;
                    ROS_INFO("gps fail count : %d", fail_count);
                }
                latest_mode_time = t0;
            }
            if(msg->status.status<0){ROS_ERROR("gps fix failed"); return;}
            if(msg->status.status<1){ROS_WARN("gps no rtk");} 
            if( isnan(msg->latitude)!=0 || isnan(msg->longitude)!=0 ){ROS_ERROR("gps nan"); return;}    
            vector<double> xy(2);
            LatLonToUTMXY(msg->latitude, msg->longitude, 52, xy.at(0), xy.at(1));
            
            rt.header = msg->header;
            rt.x = xy.at(0);
            rt.y = xy.at(1);
            rt.pos_err = msg->position_covariance[0] + msg->position_covariance[4];
            pub_.publish(rt);
        }catch(...){
            ROS_ERROR("gps error catched");
            return;
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_serial_decoder");
	GPS_Decoder GPSObject;
    ros::spin();
}