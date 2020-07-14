#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include <iostream>
#include <string>
using namespace std;

void msgCallback(const nmea_msgs::Sentence::ConstPtr& msg) {
    string st = msg->sentence;

    if (st.substr(1, 5) == "GNGGA") {
        string time;
        string lat;
        string lon;
    
        time = st.substr(7, 9);
        lat = st.substr(17, 12);
        lon = st.substr(32, 13);

        cout << time << " " << lat << " " << lon << endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_subscriber");

    ros::NodeHandle nh;
    ros::Subscriber gps_sub = nh.subscribe("nmea_sentence", 100, msgCallback);

    ros::spin();

    return 0;
}