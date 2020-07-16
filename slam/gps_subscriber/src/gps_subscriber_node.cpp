#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include "localization/Gps.h"
#include "opencv2/opencv.hpp" // Map Show
#include <iostream>
#include <string>
#include <vector>
#include "UTM.h"
#include "XYToPixel.h" // Map Show
using namespace std;


class GPS_Decoder{
    public:
    GPS_Decoder(){
	pub_ = n_.advertise<localization::Gps>("gps", 1000);
	sub_ = n_.subscribe("/nmea_sentence", 1, &GPS_Decoder::callback, this);
    img = cv::imread("src/zero/slam/gps_subscriber/src/map.png", 1); // Map Show
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
            if(stoi(tokens[6])<1){ROS_INFO("failedByQualityIndicator"); cout<<tokens[6]<<endl; return;}
            if(stoi(tokens[6])<2){ROS_INFO("warnByQualityIndicator"); cout<<tokens[6]<<endl;}
            if(stoi(tokens[7])<3){ROS_INFO("failedByStatellitesN"); cout<<tokens[7]<<endl; return;}
            if(stoi(tokens[7])<8){ROS_INFO("warnByStatellitesN"); cout<<tokens[7]<<endl;}
            if(stod(tokens[8])>10.0){ROS_INFO("failedByHDOP"); cout<<tokens[8]<<endl; return;}
            if(stod(tokens[8])>3.5){ROS_INFO("warnByHDOP"); cout<<tokens[8]<<endl;}
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
                if(checksum != c){ROS_INFO("FailedByChecksum"); cout<<checksum<<"!="<<c<<"==0x"<<s<<endl; return;}
            }
            double time_raw, lat_raw, lon_raw;
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
            
            vector<double> xy(2);
            LatLonToUTMXY(lat, lon, 52, xy.at(0), xy.at(1));
            
            rt.x = xy.at(0);
            rt.y = xy.at(1);
            //rt.header.stamp.sec= int(time);
            //rt.header.stamp.nsec= (time-rt.header.stamp.sec)*1e9;
            ros::Time tm = ros::Time::now();
            rt.header.stamp.sec = tm.sec;
            rt.header.stamp.nsec = tm.nsec;

            // ----------------------- map show ---------------
            int pixel_x, pixel_y;
            
            XYToPixel(img, rt.x, rt.y, pixel_x, pixel_y, 2);
            cv::line(img, cv::Point(pixel_x, pixel_y), cv::Point(pixel_x+10, pixel_y+10), cv::Scalar(0, 255, 0), 10);
            cout << pixel_x << ", " << pixel_y << std::endl;
            if (n == 300) {
                cv::imwrite("src/zero/slam/gps_subscriber/src/path.png", img);
                cout << "saved" << endl;
            }

            n++;
            std::cout << n << std::endl;

            // ----------------------- map show ----------------
            
            pub_.publish(rt);
        }
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    cv::Mat img; // Map Show
    int n = 0; // Map Show
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_subscriber");
    GPS_Decoder GPSObject;
    ros::spin();
}
