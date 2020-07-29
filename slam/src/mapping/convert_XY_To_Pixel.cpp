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


class XYToPixel{
    public:
    XYToPixel(){
        pub_ = nh_.advertise<slam::Pixel>("/position/pixel",1000);
        sub_ = nh_.subscribe("/filtered_data",1, &XYToPixel::callback, this);
        a1=33.358382978338263;
        b1=-0.043763656504531664;
        c1=-9765560.7805824;
        a2=-0.043763656504529812;
        b2=-33.358382978338255;
        c2=138047583.23571858;
    }

    void callback(const slam::Data::ConstPtr& msg){
        int pixel_x = int(a1*msg->x+b1*msg->y+c1);
        int pixel_y = int(a2*msg->x+b2*msg->y+c2);
        ROS_INFO("%d %d", pixel_x, pixel_y);
        slam::Pixel rt;
        rt.header = msg->header;
        rt.x = pixel_x;
        rt.y = pixel_y;
        pub_.publish(rt);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double a1,b1,c1,a2,b2,c2;
};


int main(int argc, char** argv){
    ros::init(argc,argv,"convert_XY_To_Pixel");
    XYToPixel xy_to_pixel;
    ros::spin();
}
