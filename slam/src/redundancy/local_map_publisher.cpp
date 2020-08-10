#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include "slam/Data.h"
#include "slam/Pixel.h"
#include "std_msgs/Duration.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "UTM.h"
using namespace std;


class LocalMapPublisher{
    public:
    LocalMapPublisher(){
        pub_ = nh_.advertise<slam::Pixel>("/position/pixel",1000);
        sub_ = nh_.subscribe("/filtered_data",1, &XYToPixel::callback1, this);
        sub_ = nh2_.subscribe("/position/pixel", dkfjdkl);callback2
        cv::imread ..... costmap.png;
    }

    void callback1(){
        heading update;
        return;
    }

    void callback2(){
        pixelx pixely
        
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double heading;
    cv::Mat global_map;
}