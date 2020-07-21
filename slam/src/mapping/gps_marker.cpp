#include "ros/ros.h"
#include "slam/Gps.h"
#include "slam/Data.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include "XYToPixel.h"
#include "ros/package.h"
using namespace std;


class GPS_Decoder{
    public:
    GPS_Decoder(){
	sub_gps = n_.subscribe("/gps", 10000, &GPS_Decoder::callback_gps, this);
    sub_filtered = n_.subscribe("/filtered_data", 10000, &GPS_Decoder::callback_filtered, this);
    stringstream path_stream;
    path_stream << ros::package::getPath("slam") << "/src/mapping/map.png";
    img = cv::imread(path_stream.str(), 1);
    ROS_INFO("Image loaded");
    }

    void callback_gps(const slam::Gps::ConstPtr& msg){
	    int pixel_x, pixel_y;
        
        XYToPixel(img, msg->x, msg->y, pixel_x, pixel_y, 2);
        ROS_INFO("%dth point: (%lf, %lf) = (%d, %d)", n, msg->x, msg->y, pixel_x, pixel_y);

        if (n != 1) {
            if (sqrt(pow(prev_pixel_x - pixel_x, 2) + pow(prev_pixel_x - pixel_x, 2)) < 200 | saving == true) {
                cv::line(img, cv::Point(prev_pixel_x, prev_pixel_y), cv::Point(pixel_x, pixel_y), cv::Scalar(0, 0, 255), 3);
                cv::circle(img, cv::Point(filtered_pixel_x, filtered_pixel_y), 3, cv::Scalar(255, 0, 0), -1);
                prev_pixel_x = pixel_x;
                prev_pixel_y = pixel_y;

                saving = false;
            } else if (saving == false) {
                ROS_WARN("%dth point: outlier", n);
                ROS_WARN("%dth point: Distance from very previous point(px): %lf", n, sqrt(pow(prev_pixel_x - pixel_x, 2) + pow(prev_pixel_x - pixel_x, 2)));
            }

            if (n == 11000) {
                stringstream path_stream;
                path_stream << ros::package::getPath("slam") << "/src/mapping/path" << (n) << ".png";

                cv::imwrite(path_stream.str(), img);
                ROS_INFO("Image saved");
                saving = true;
            }
        } else {
            prev_pixel_x = pixel_x;
            prev_pixel_y = pixel_y;
        }
        n++;
    }

    void callback_filtered(const slam::Data::ConstPtr& msg){
	    int pixel_x, pixel_y;
        
        XYToPixel(img, msg->x, msg->y, pixel_x, pixel_y, 2);
        filtered_pixel_x = pixel_x;
        filtered_pixel_y = pixel_y;
    }

    private:
    ros::NodeHandle n_;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_filtered;
    cv::Mat img;
    int n = 1;
    int prev_pixel_x = 0;
    int prev_pixel_y = 0;
    int filtered_pixel_x = -1;
    int filtered_pixel_y = -1;
    bool saving = false;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_marker");
    GPS_Decoder GPSObject;
    ros::spin();
}
