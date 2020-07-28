#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include "slam/Gps.h"
#include "std_msgs/Duration.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "UTM.h"
using namespace std;

double QI_err;
double QI_warn;
double Satellites_err;
double Satellites_warn;
double HDOP_err;
double HDOP_warn;

class GPS_Decoder{
    public:
    GPS_Decoder(){
        pub_ = n_.advertise<slam::Gps>("gps", 2);
        sub_ = n_.subscribe("/nmea_sentence", 2, &GPS_Decoder::callback, this);
        pub_delay = n_.advertise<std_msgs::Duration>("delay_gps", 10000);
    }	

    void callback(const nmea_msgs::Sentence::ConstPtr& msg){
        ros::Time t0 = ros::Time::now();
        slam::Gps rt;
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

	//edited by boseol
        if(tokens[0] == "$GNGGA"){
            double time_raw, lat_raw, lon_raw;
            double time, lat, lon;
            try{
                if(stoi(tokens[6])<QI_err){ROS_ERROR("failedByQualityIndicator(No Satellite), GPS Quality Indicator: %s", tokens[6].c_str()); return;}
                if(stoi(tokens[6])<QI_warn){ROS_WARN("warnByQualityIndicator(Non-RTK), GPS Quality Indicator: %s", tokens[6].c_str());}
                if(stoi(tokens[7])<Satellites_err){ROS_ERROR("failedBySatellitesN, The number of Satellite: %s", tokens[7].c_str()); return;}
                if(stoi(tokens[7])<Satellites_warn){ROS_WARN("warnBySatellitesN, The number of Satellite: %s", tokens[7].c_str());}
                if(stod(tokens[8])>HDOP_err || stod(tokens[8])==0.0){ROS_ERROR("failedByHDOP, HDOP value: %s", tokens[8].c_str()); return;}
                if(stod(tokens[8])>HDOP_warn){ROS_WARN("warnByHDOP, HDOP value: %s", tokens[8].c_str());} 

                {   //check checksum 
                    s = msg->sentence;
                    delimiter = "$";
                    pos = s.find(delimiter);
                    s.erase(0, pos + delimiter.length());
                    delimiter = "*";
                    pos = s.find(delimiter);
                    token = s.substr(0, pos);
                    s.erase(0, pos + delimiter.length());
                    int checksum = 0;
                    for (string::iterator iter = token.begin(); iter != token.end(); ++iter){
                        checksum ^= (int)(*iter);
                    }
                    int c;
                    stringstream ss;
                    ss << std::hex << s;
                    ss >> c;
                    if(checksum != c){ROS_ERROR("FailedByChecksum, %d != %d == 0x%s", checksum, c, s.c_str()); return;}
                }

                // time_raw = stod(tokens[1]);
                lat_raw = stod(tokens[2]);
                lon_raw = stod(tokens[4]);
            }catch(...){
                ROS_ERROR("failedByNotANumber");
                return;
            }
            // int hour = (int)time_raw/10000;
            // int min = (int)time_raw/100 - hour*100;
            // double sec = time_raw -(hour*10000+min*100);
            // time = hour*3600 + min*60 + sec;

            lat = (int)lat_raw/100;
            lon = (int)lon_raw/100;

            lat += (lat_raw-lat*100)/60;
            lon += (lon_raw-lon*100)/60;
            
            vector<double> xy(2);
            LatLonToUTMXY(lat, lon, 52, xy.at(0), xy.at(1));
            
            rt.header = msg->header;
            rt.x = xy.at(0);
            rt.y = xy.at(1);
            //rt.header.stamp.sec= int(time);
            //rt.header.stamp.nsec= (time-rt.header.stamp.sec)*1e9;
            //ros::Time tm = ros::Time::now();
            //rt.header.stamp.sec = tm.sec;
            //rt.header.stamp.nsec = tm.nsec;

            pub_.publish(rt);

            std_msgs::Duration dt;
            dt.data = ros::Time::now() - t0;
            pub_delay.publish(dt);
        }
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_delay;
    ros::Subscriber sub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_decoder");
	ros::param::get("/QI_err", QI_err);
	ros::param::get("/QI_warn", QI_warn);
	ros::param::get("/Satellites_err", Satellites_err);
	ros::param::get("/Satellites_warn", Satellites_warn);
	ros::param::get("/HDOP_err", HDOP_err);
	ros::param::get("/HDOP_warn", HDOP_warn);
    GPS_Decoder GPSObject;
    ros::spin();
}
