#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include "localization/Gps.h"
#include <iostream>
#include <string>
#include <vector>
#include "UTM.h"
using namespace std;


class GPS_Decoder{
    public:
    GPS_Decoder(){
	pub_ = n_.advertise<localization::Gps>("gps", 1000);
	sub_ = n_.subscribe("/nmea_sentence", 1, &GPS_Decoder::callback, this);
    }

    void callback(const nmea_msgs::Sentence::ConstPtr& msg){
	localization::Gps rt;
	string s = msg->sentence;
        string delimiter = ",";
        vector<string> tokens;
        string token;
        size_t pos = 0;

        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            tokens.push_back(token);
            s.erase(0, pos + delimiter.length());
        }
        if(tokens[0] == "$GNGGA"){
            double time_raw,lat_raw,lon_raw;
            double time, lat, lon;
            time_raw = stod(tokens[1]);
            lat_raw = stod(tokens[2]);
            lon_raw = stod(tokens[4]);
            int hour = (int)time_raw/10000;
            int min = (int)time_raw/100 - hour*100;
            double sec = time_raw -(hour*10000+min*100);

            time = hour*3600 + min*60 + sec;

            lat = (int)lat_raw/100;
            lon = (int)lon_raw/100;

            lat += (lat_raw-lat*100)/60;
            lon += (lon_raw-lon*100)/60;
            
	    vector<float> xy(2);
	    LatLonToUTMXY(lat, lon, 52, xy.at(0), xy.at(1));
	    
	    rt.x = xy.at(0);
	    rt.y = xy.at(1);
	    rt.header.stamp.sec= int(time);
	    rt.header.stamp.nsec= (time-rt.header.stamp.sec)*1e9;
	    

	    pub_.publish(rt);
        }
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_subscriber");
    GPS_Decoder GPSObject;
    ros::spin();
}
